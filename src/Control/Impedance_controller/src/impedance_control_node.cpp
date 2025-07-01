#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <deque>
#include <Eigen/Dense>

class ImpedanceController : public rclcpp::Node {
public:
    ImpedanceController() : Node("impedance_control_node") {
        // Khai báo tham số riêng cho từng động cơ
        this->declare_parameter<std::vector<double>>("kp", {4.0, 4.0});
        this->declare_parameter<std::vector<double>>("kd", {0.5, 0.5});
        this->declare_parameter<std::vector<double>>("m", {0.0, 0.0});
        this->declare_parameter<std::vector<double>>("torque_limit", {4.0, 4.0});
        this->declare_parameter<std::vector<double>>("q_desired", {-1.6, 0.0});
        this->declare_parameter<std::vector<double>>("qd_dot_desired", {0.0, 0.0});
        this->declare_parameter<std::vector<double>>("qd_ddot_desired", {0.0, 0.0});

        // Lấy giá trị tham số
        auto kp_vec = this->get_parameter("kp").as_double_array();
        auto kd_vec = this->get_parameter("kd").as_double_array();
        auto m_vec = this->get_parameter("m").as_double_array();
        auto torque_limit_vec = this->get_parameter("torque_limit").as_double_array();
        auto q_desired_vec = this->get_parameter("q_desired").as_double_array();
        auto qd_dot_desired_vec = this->get_parameter("qd_dot_desired").as_double_array();
        auto qd_ddot_desired_vec = this->get_parameter("qd_ddot_desired").as_double_array();

        // Khởi tạo vector tham số
        Kp << kp_vec[0], kp_vec[1];
        Kd << kd_vec[0], kd_vec[1];
        M << m_vec[0], m_vec[1];
        TORQUE_LIMIT << torque_limit_vec[0], torque_limit_vec[1];
        q_desired_ << q_desired_vec[0], q_desired_vec[1];
        qd_dot_desired_ << qd_dot_desired_vec[0], qd_dot_desired_vec[1];
        qd_ddot_desired_ << qd_ddot_desired_vec[0], qd_ddot_desired_vec[1];

        // Khởi tạo các vector khác
        q_current_.setZero();
        q_dot_current_.setZero();
        q_ddot_current_.setZero();
        q_dot_last_.setZero();
        use_impedance_.fill(false);
        last_time_ = this->now();

        // Subscriber cho 2 động cơ sử dụng lambda
        position_sub_[0] = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/position_0", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->positionCallback(msg, 0);
            });
        velocity_sub_[0] = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/velocity_0", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->velocityCallback(msg, 0);
            });
        position_sub_[1] = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/position_1", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->positionCallback(msg, 1);
            });
        velocity_sub_[1] = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/velocity_1", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->velocityCallback(msg, 1);
            });

        // Publisher cho torque
        torque_pub_[0] = this->create_publisher<std_msgs::msg::Float32>("/motor/torque_0", 10);
        torque_pub_[1] = this->create_publisher<std_msgs::msg::Float32>("/motor/torque_1", 10);

        // Timer để gửi torque định kỳ
        timer_ = this->create_wall_timer(std::chrono::milliseconds(2), 
            std::bind(&ImpedanceController::publishTorque, this));

        // Callback để cập nhật tham số động
        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&ImpedanceController::parameterCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ImpedanceController started for 2 motors");
        RCLCPP_INFO(this->get_logger(), "Kp: [%.2f, %.2f], Kd: [%.2f, %.2f], M: [%.2f, %.2f], Torque_Limit: [%.2f, %.2f]", 
                    Kp(0), Kp(1), Kd(0), Kd(1), M(0), M(1), TORQUE_LIMIT(0), TORQUE_LIMIT(1));
        RCLCPP_INFO(this->get_logger(), "Desired q: [%.2f, %.2f]", q_desired_(0), q_desired_(1));
    }
    // Method to set desired state
    void setDesiredState(const Eigen::Vector2d& q_d, const Eigen::Vector2d& qd_dot, const Eigen::Vector2d& qd_ddot) {
        q_desired_ = q_d;
        qd_dot_desired_ = qd_dot;
        qd_ddot_desired_ = qd_ddot;
        use_impedance_.fill(false);
    }
    
    Eigen::Vector2d computeTorque(const Eigen::Vector2d& q_current, const Eigen::Vector2d& q_dot_current, 
                                  const Eigen::Vector2d& q_ddot_current, const Eigen::Vector2d& q_desired,
                                  const Eigen::Vector2d& qd_dot_desired, const Eigen::Vector2d& qd_ddot_desired,
                                  std::array<bool, 2>& use_impedance) {
        const double REV_TO_RAD = M_PI / 4.0;
        const double POSITION_TOLERANCE = M_PI / 50.0;
        const double DEFAULT_TORQUE = 0.5;

        Eigen::Vector2d q_rad = q_current * REV_TO_RAD;
        Eigen::Vector2d qd_rad = q_dot_current * REV_TO_RAD;
        Eigen::Vector2d qd_ddot_rad = q_ddot_current * REV_TO_RAD;
        Eigen::Vector2d q_desired_rad = q_desired * REV_TO_RAD;
        Eigen::Vector2d qd_dot_desired_rad = qd_dot_desired * REV_TO_RAD;
        Eigen::Vector2d qd_ddot_desired_rad = qd_ddot_desired * REV_TO_RAD;

        Eigen::Vector2d position_error = q_desired_rad - q_rad;
        Eigen::Vector2d torque;

        for (int i = 0; i < 2; ++i) {
            if (std::abs(position_error(i)) <= POSITION_TOLERANCE) {
                use_impedance[i] = true;
            }

            if (!use_impedance[i]) {
                torque(i) = (position_error(i) > 0) ? DEFAULT_TORQUE : -DEFAULT_TORQUE;
            } else {
                double velocity_error = qd_dot_desired_rad(i) - qd_rad(i);
                double acceleration_error = qd_ddot_desired_rad(i) - qd_ddot_rad(i);

                if (std::abs(position_error(i)) <= POSITION_TOLERANCE) {
                    torque(i) = 0.0;
                } else {
                    torque(i) = Kp(i) * position_error(i) + Kd(i) * velocity_error + M(i) * acceleration_error;
                    if (std::abs(position_error(i)) < POSITION_TOLERANCE * 3.0 && std::abs(qd_rad(i)) < 0.05) {
                        torque(i) *= 0.5;
                    }
                }
            }
            torque(i) = std::clamp(torque(i), -TORQUE_LIMIT(i), TORQUE_LIMIT(i));
        }
        return torque;
    }

private:
    Eigen::Vector2d Kp, Kd, M, TORQUE_LIMIT;
    Eigen::Vector2d q_desired_, qd_dot_desired_, qd_ddot_desired_;
    Eigen::Vector2d q_current_, q_dot_current_, q_ddot_current_, q_dot_last_;
    std::array<bool, 2> use_impedance_;
    rclcpp::Time last_time_;

    std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 2> position_sub_;
    std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 2> velocity_sub_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 2> torque_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

    void positionCallback(const std_msgs::msg::Float32::SharedPtr msg, int index) {
        q_current_(index) = msg->data;
    }

    void velocityCallback(const std_msgs::msg::Float32::SharedPtr msg, int index) {
        q_dot_current_(index) = msg->data;
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt > 0.0) {
            double q_ddot_raw = (q_dot_current_(index) - q_dot_last_(index)) / dt;
            const double alpha = 0.2;
            q_ddot_current_(index) = alpha * q_ddot_raw + (1 - alpha) * q_ddot_current_(index);
        }
        q_dot_last_(index) = q_dot_current_(index);
    }

    void publishTorque() {
        Eigen::Vector2d torque = computeTorque(q_current_, q_dot_current_, q_ddot_current_,
                                               q_desired_, qd_dot_desired_, qd_ddot_desired_,
                                               use_impedance_);

        int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error opening CAN socket");
            return;
        }

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, "can0");
        ioctl(sock, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error binding CAN socket");
            close(sock);
            return;
        }

        struct can_frame frame;
        for (int i = 0; i < 2; ++i) {
            float torque_float = static_cast<float>(torque(i));
            uint8_t torque_bytes[sizeof(float)];
            std::memcpy(torque_bytes, &torque_float, sizeof(float));
            frame.can_id = (i == 0) ? 0x0E : (1 << 5) | 0x0E;
            frame.can_dlc = sizeof(float);
            std::memcpy(frame.data, torque_bytes, sizeof(float));
            if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame to Node %d", i);
            } else {
                RCLCPP_INFO(this->get_logger(), "Sent Torque to Node %d: %.5f Nm (Impedance: %d)", 
                            i, torque(i), use_impedance_[i]);
            }

            auto torque_msg = std_msgs::msg::Float32();
            torque_msg.data = torque(i);
            torque_pub_[i]->publish(torque_msg);
        }
        close(sock);
    }

    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name() == "kp") {
                auto vec = param.as_double_array();
                Kp << vec[0], vec[1];
                RCLCPP_INFO(this->get_logger(), "Updated Kp to [%.2f, %.2f]", vec[0], vec[1]);
            } else if (param.get_name() == "kd") {
                auto vec = param.as_double_array();
                Kd << vec[0], vec[1];
                RCLCPP_INFO(this->get_logger(), "Updated Kd to [%.2f, %.2f]", vec[0], vec[1]);
            } else if (param.get_name() == "m") {
                auto vec = param.as_double_array();
                M << vec[0], vec[1];
                RCLCPP_INFO(this->get_logger(), "Updated M to [%.2f, %.2f]", vec[0], vec[1]);
            } else if (param.get_name() == "torque_limit") {
                auto vec = param.as_double_array();
                TORQUE_LIMIT << vec[0], vec[1];
                RCLCPP_INFO(this->get_logger(), "Updated TORQUE_LIMIT to [%.2f, %.2f]", vec[0], vec[1]);
            } else if (param.get_name() == "q_desired") {
                auto vec = param.as_double_array();
                setDesiredState(Eigen::Vector2d(vec[0], vec[1]), qd_dot_desired_, qd_ddot_desired_);
                RCLCPP_INFO(this->get_logger(), "Updated q_desired to [%.2f, %.2f]", vec[0], vec[1]);
            } else if (param.get_name() == "qd_dot_desired") {
                auto vec = param.as_double_array();
                setDesiredState(q_desired_, Eigen::Vector2d(vec[0], vec[1]), qd_ddot_desired_);
                RCLCPP_INFO(this->get_logger(), "Updated qd_dot_desired to [%.2f, %.2f]", vec[0], vec[1]);
            } else if (param.get_name() == "qd_ddot_desired") {
                auto vec = param.as_double_array();
                setDesiredState(q_desired_, qd_dot_desired_, Eigen::Vector2d(vec[0], vec[1]));
                RCLCPP_INFO(this->get_logger(), "Updated qd_ddot_desired to [%.2f, %.2f]", vec[0], vec[1]);
            }
        }
        return result;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImpedanceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}






