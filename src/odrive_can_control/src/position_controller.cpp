#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <std_msgs/msg/float32.hpp>

class PositionController : public rclcpp::Node {
public:
    PositionController() : Node("position_controller") {
        RCLCPP_INFO(this->get_logger(), "Position Controller Node Started!");

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

        // Khởi tạo chế độ cho cả hai node ODrive
        init_odrive_axis(0);
        init_odrive_axis(1);

        // Tạo subscriber cho vị trí của Node 0
        pos_sub_0_ = this->create_subscription<std_msgs::msg::Float32>(
            "/odrive/position_0", 10,
            std::bind(&PositionController::position_callback_0, this, std::placeholders::_1));

        // Tạo subscriber cho vị trí của Node 1
        pos_sub_1_ = this->create_subscription<std_msgs::msg::Float32>(
            "/odrive/position_1", 10,
            std::bind(&PositionController::position_callback_1, this, std::placeholders::_1));

        // Tạo timer gửi vị trí (mỗi 100ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PositionController::control_pos, this)
        );
    }

    ~PositionController() {
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:
    void init_odrive_axis(int node_id) {
        // Đặt Control Mode = Position Control (3), Input Mode = Pass-through (1)
        struct can_frame frame;
        frame.can_id = (node_id << 5) | 0x00B; // Set_Controller_Modes
        frame.can_dlc = 8;
        int32_t control_mode = 3; // Position Control
        int32_t input_mode = 1;   // Pass-through
        std::memcpy(frame.data, &control_mode, sizeof(int32_t));
        std::memcpy(frame.data + 4, &input_mode, sizeof(int32_t));
        if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh Set_Controller_Modes cho Node %d!", node_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "Đặt Position Control Mode cho Node %d", node_id);
        }

        // Đặt Axis State = Closed Loop Control (8)
        frame.can_id = (node_id << 5) | 0x007; // Set_Axis_Requested_State
        frame.can_dlc = 4;
        int32_t axis_state = 8; // Closed Loop Control
        std::memcpy(frame.data, &axis_state, sizeof(int32_t));
        std::memset(frame.data + 4, 0, 4);
        if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh Closed Loop Control cho Node %d!", node_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "Đặt Closed Loop Control cho Node %d", node_id);
        }
    }

    void control_pos() {
        // Gửi vị trí hiện tại cho cả hai node
        send_position_command(0, position_0_);
        send_position_command(1, position_1_);
    }

    void position_callback_0(const std_msgs::msg::Float32::SharedPtr msg) {
        position_0_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Nhận vị trí %.2f turns cho Node 0 từ topic /odrive/position_0", position_0_);
    }

    void position_callback_1(const std_msgs::msg::Float32::SharedPtr msg) {
        position_1_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Nhận vị trí %.2f turns cho Node 1 từ topic /odrive/position_1", position_1_);
    }

    void send_position_command(int node_id, float position) {
        struct can_frame frame;
        frame.can_id = (node_id << 5) | 0x0C; // Set_Input_Pos
        frame.can_dlc = 8;
        int16_t vel_ff = 0; // Vận tốc feedforward (0.01 turns/s)
        int16_t torque_ff = 0; // Mô-men xoắn feedforward (0.001 Nm)
        std::memcpy(frame.data, &position, sizeof(float));
        std::memcpy(frame.data + 4, &vel_ff, sizeof(int16_t));
        std::memcpy(frame.data + 6, &torque_ff, sizeof(int16_t));
        if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh vị trí CAN cho động cơ %d!", node_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "Gửi vị trí %.2f turns đến ODrive ID %d", position, node_id);
        }
    }

    int sock_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_sub_0_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_sub_1_;
    float position_0_ = 0.0; // Vị trí mặc định cho Node 0 (turns)
    float position_1_ = 0.0; // Vị trí mặc định cho Node 1 (turns)
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionController>());
    rclcpp::shutdown();
    return 0;
}