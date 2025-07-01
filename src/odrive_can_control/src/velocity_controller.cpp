#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <std_msgs/msg/float32.hpp>

class VelocityController : public rclcpp::Node {
public:
    VelocityController() : Node("velocity_controller") {
        RCLCPP_INFO(this->get_logger(), "Velocity Controller Node Started!");

        // Tạo socket CAN
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Không thể mở socket CAN!");
            rclcpp::shutdown();
            return;
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, "can0");
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Không thể lấy chỉ mục CAN interface!");
            close(sock_);
            rclcpp::shutdown();
            return;
        }

        struct sockaddr_can addr = {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Không thể kết nối CAN!");
            close(sock_);
            rclcpp::shutdown();
            return;
        }

        // Tạo subscriber cho vận tốc của Node 0
        vel_sub_0_ = this->create_subscription<std_msgs::msg::Float32>(
            "/odrive/velocity_0", 10,
            std::bind(&VelocityController::velocity_callback_0, this, std::placeholders::_1));

        // Tạo subscriber cho vận tốc của Node 1
        vel_sub_1_ = this->create_subscription<std_msgs::msg::Float32>(
            "/odrive/velocity_1", 10,
            std::bind(&VelocityController::velocity_callback_1, this, std::placeholders::_1));

        // Tạo timer gửi vận tốc (mỗi 100ms để phản hồi nhanh với đầu vào)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VelocityController::control_vel, this)
        );
    }

    ~VelocityController() {
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:
    void control_vel() {
        // Gửi vận tốc hiện tại cho cả hai node
        send_velocity_command(0, velocity_0_);
        send_velocity_command(1, velocity_1_);
    }

    void velocity_callback_0(const std_msgs::msg::Float32::SharedPtr msg) {
        velocity_0_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Nhận vận tốc %.2f cho Node 0 từ topic /odrive/velocity_0", velocity_0_);
    }

    void velocity_callback_1(const std_msgs::msg::Float32::SharedPtr msg) {
        velocity_1_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Nhận vận tốc %.2f cho Node 1 từ topic /odrive/velocity_1", velocity_1_);
    }

    void send_velocity_command(int node_id, float velocity) {
        struct can_frame frame;
        frame.can_id = (node_id << 5) | 0x0D; // Set_Input_Vel (0x0D)
        frame.can_dlc = 8;

        std::memcpy(frame.data, &velocity, sizeof(float));
        std::memset(frame.data + 4, 0, 4);

        if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh CAN cho động cơ %d!", node_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "Gửi vận tốc %.2f đến ODrive ID %d", velocity, node_id);
        }
    }

    int sock_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_0_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_1_;
    float velocity_0_ = 0.0; // Vận tốc mặc định cho Node 0
    float velocity_1_ = 0.0; // Vận tốc mặc định cho Node 1
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityController>());
    rclcpp::shutdown();
    return 0;
}