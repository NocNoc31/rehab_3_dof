
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

class ImpedanceController : public rclcpp::Node {
public:
    ImpedanceController() : Node("impedance_control_node"), 
                            Kp(0.5), Kd(0.08), M(0.003),
                            q_dot_last_0_(0.0), q_ddot_current_0_(0.0),
                            q_dot_last_1_(0.0), q_ddot_current_1_(0.0),
                            last_time_0_(this->now()), last_time_1_(this->now()) {
        // Subscriber cho Node ID 0
        position_sub_0_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/position_0", 10, std::bind(&ImpedanceController::positionCallback0, this, std::placeholders::_1));
        velocity_sub_0_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/velocity_0", 10, std::bind(&ImpedanceController::velocityCallback0, this, std::placeholders::_1));

        // Subscriber cho Node ID 1
        position_sub_1_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/position_1", 10, std::bind(&ImpedanceController::positionCallback1, this, std::placeholders::_1));
        velocity_sub_1_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/velocity_1", 10, std::bind(&ImpedanceController::velocityCallback1, this, std::placeholders::_1));

        // Publisher cho torque
        torque_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("/motor/torque_0", 10);
        torque_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("/motor/torque_1", 10);

        // Đặt trạng thái mong muốn (có thể khác nhau cho mỗi node)
        setDesiredState(0, 4.0, 0.0, 0.0); // Node 0
        setDesiredState(1, 4.0, 0.0, 0.0); // Node 1

        // Timer để gửi torque định kỳ
        timer_ = this->create_wall_timer(std::chrono::milliseconds(2), 
            std::bind(&ImpedanceController::publishTorque, this));

        RCLCPP_INFO(this->get_logger(), "ImpedanceController started for Node ID 0 and Node ID 1");
    }

    void setDesiredState(int node_id, double q_d, double qd_dot, double qd_ddot) {
        if (node_id == 0) {
            q_desired_0_ = q_d;
            qd_dot_desired_0_ = qd_dot;
            qd_ddot_desired_0_ = qd_ddot;
        } else if (node_id == 1) {
            q_desired_1_ = q_d;
            qd_dot_desired_1_ = qd_dot;
            qd_ddot_desired_1_ = qd_ddot;
        }
    }

    double computeTorque(double q_current, double q_dot_current, double q_ddot_current, 
                         double q_desired, double qd_dot_desired, double qd_ddot_desired) {
        const double REV_TO_RAD = M_PI / 4.0;
        const double POSITION_TOLERANCE = M_PI / 36.0;
        const double TORQUE_LIMIT = 0.7;

        double q_rad = q_current * REV_TO_RAD;
        double qd_rad = q_dot_current * REV_TO_RAD;
        double qd_ddot_rad = q_ddot_current * REV_TO_RAD;

        double q_desired_rad = q_desired * REV_TO_RAD;
        double qd_dot_desired_rad = qd_dot_desired * REV_TO_RAD;
        double qd_ddot_desired_rad = qd_ddot_desired * REV_TO_RAD;

        double position_error = q_desired_rad - q_rad;
        double velocity_error = qd_dot_desired_rad - qd_rad;
        double acceleration_error = qd_ddot_desired_rad - qd_ddot_rad;

        if (std::abs(position_error) <= POSITION_TOLERANCE) {
            return 0.0; // Dừng khi gần vị trí mong muốn
        }

        // Tính toán mô-men với sai số gia tốc nhân với M
        double torque = Kp * position_error + Kd * velocity_error + M * acceleration_error;

        // Giảm lực nếu đã gần vị trí mong muốn và tốc độ thấp
        if (std::abs(position_error) < POSITION_TOLERANCE * 3.0 && std::abs(qd_rad) < 0.05) {
            torque *= 0.5;
        }

        // Giới hạn mô-men
        torque = std::clamp(torque, -TORQUE_LIMIT, TORQUE_LIMIT);
        return torque;
    }

private:
    double Kp, Kd, M;

    // Biến cho Node ID 0
    double q_desired_0_, qd_dot_desired_0_, qd_ddot_desired_0_;
    double q_current_0_ = 0.0, q_dot_current_0_ = 0.0, q_ddot_current_0_ = 0.0;
    double q_dot_last_0_ = 0.0;
    rclcpp::Time last_time_0_;

    // Biến cho Node ID 1
    double q_desired_1_, qd_dot_desired_1_, qd_ddot_desired_1_;
    double q_current_1_ = 0.0, q_dot_current_1_ = 0.0, q_ddot_current_1_ = 0.0;
    double q_dot_last_1_ = 0.0;
    rclcpp::Time last_time_1_;

    // Subscriber và Publisher
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_sub_0_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_0_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_sub_1_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_1_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_pub_0_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_pub_1_;
    rclcpp::TimerBase::SharedPtr timer_;

    void positionCallback0(const std_msgs::msg::Float32::SharedPtr msg) {
        q_current_0_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Node 0 position: %.3f turns", q_current_0_);
    }

    void velocityCallback0(const std_msgs::msg::Float32::SharedPtr msg) {
        q_dot_current_0_ = msg->data;

        rclcpp::Time now = this->now();
        double dt = (now - last_time_0_).seconds();
        last_time_0_ = now;

        if (dt > 0.0) {
            double q_ddot_raw = (q_dot_current_0_ - q_dot_last_0_) / dt;
            const double alpha = 0.2;
            q_ddot_current_0_ = alpha * q_ddot_raw + (1 - alpha) * q_ddot_current_0_;
        }

        q_dot_last_0_ = q_dot_current_0_;
        RCLCPP_INFO(this->get_logger(), "Node 0 velocity: %.3f turns/s", q_dot_current_0_);
    }

    void positionCallback1(const std_msgs::msg::Float32::SharedPtr msg) {
        q_current_1_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Node 1 position: %.3f turns", q_current_1_);
    }

    void velocityCallback1(const std_msgs::msg::Float32::SharedPtr msg) {
        q_dot_current_1_ = msg->data;

        rclcpp::Time now = this->now();
        double dt = (now - last_time_1_).seconds();
        last_time_1_ = now;

        if (dt > 0.0) {
            double q_ddot_raw = (q_dot_current_1_ - q_dot_last_1_) / dt;
            const double alpha = 0.2;
            q_ddot_current_1_ = alpha * q_ddot_raw + (1 - alpha) * q_ddot_current_1_;
        }

        q_dot_last_1_ = q_dot_current_1_;
        RCLCPP_INFO(this->get_logger(), "Node 1 velocity: %.3f turns/s", q_dot_current_1_);
    }

    void publishTorque() {
        // Tính torque cho Node 0
        double torque_0 = computeTorque(q_current_0_, q_dot_current_0_, q_ddot_current_0_,
                                        q_desired_0_, qd_dot_desired_0_, qd_ddot_desired_0_);
        // Tính torque cho Node 1
        double torque_1 = computeTorque(q_current_1_, q_dot_current_1_, q_ddot_current_1_,
                                        q_desired_1_, qd_dot_desired_1_, qd_ddot_desired_1_);

        // Gửi torque qua CAN
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

        // Gửi torque cho Node 0
        float torque_float_0 = static_cast<float>(torque_0);
        uint8_t torque_bytes_0[sizeof(float)];
        std::memcpy(torque_bytes_0, &torque_float_0, sizeof(float));
        frame.can_id = 0x0E; // Node 0: Set_Input_Torque
        frame.can_dlc = sizeof(float);
        std::memcpy(frame.data, torque_bytes_0, sizeof(float));
        if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame to Node 0");
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent Torque to Node 0: %.5f Nm over CAN", torque_0);
        }

        // Gửi torque cho Node 1
        float torque_float_1 = static_cast<float>(torque_1);
        uint8_t torque_bytes_1[sizeof(float)];
        std::memcpy(torque_bytes_1, &torque_float_1, sizeof(float));
        frame.can_id = (1 << 5) | 0x0E; // Node 1: 0x2E
        frame.can_dlc = sizeof(float);
        std::memcpy(frame.data, torque_bytes_1, sizeof(float));
        if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame to Node 1");
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent Torque to Node 1: %.5f Nm over CAN", torque_1);
        }

        // Publish torque qua ROS 2
        auto torque_msg_0 = std_msgs::msg::Float32();
        torque_msg_0.data = torque_0;
        torque_pub_0_->publish(torque_msg_0);

        auto torque_msg_1 = std_msgs::msg::Float32();
        torque_msg_1.data = torque_1;
        torque_pub_1_->publish(torque_msg_1);

        close(sock);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImpedanceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}