// #include <rclcpp/rclcpp.hpp>
// #include <can_odrive_interface/odrive_interface.hpp>
// #include <std_msgs/msg/float32.hpp>
// #include <cmath>

// const double REV_TO_RAD = M_PI / 4.0; // 1 turn = π/4 radian
// const double RAD_TO_REV = 4.0 / M_PI; // Nghịch đảo của REV_TO_RAD

// class TestPosition : public rclcpp::Node {
// public:
//     TestPosition() : Node("test_position") {
//         odrive_ = std::make_shared<can_odrive_interface::ODriveInterface>();
        
//         // Khởi tạo chế độ position cho Node 1
//         odrive_->InitPositionMode(1);

//         // Tạo timer để gửi lệnh vị trí mỗi 2 giây
//         timer_ = this->create_wall_timer(
//             std::chrono::seconds(2),
//             std::bind(&TestPosition::send_position, this));
        
//         RCLCPP_INFO(this->get_logger(), "Test Position Node started");
//     }

// private:
//     void send_position() {
//         static float position_rad = 1.57; // Vị trí ban đầu: 1.57 rad (≈ 90°)
//         float position_turns = position_rad * RAD_TO_REV; // Chuyển radian sang turns
//         odrive_->SendPositionCommand(1, position_turns);
//         RCLCPP_INFO(this->get_logger(), "Gửi vị trí %.2f rad (%.2f turns) cho Node 1", position_rad, position_turns);
//         position_rad += 1.57; // Tăng vị trí 1.57 rad (≈ 90°) mỗi lần
//     }

//     std::shared_ptr<can_odrive_interface::ODriveInterface> odrive_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<TestPosition>());
//     rclcpp::shutdown();
//     return 0;
// }






#include <rclcpp/rclcpp.hpp>
#include <can_odrive_interface/odrive_interface.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

const double REV_TO_RAD = M_PI / 4.0; // 1 turn = π/4 radian

class TestPosition : public rclcpp::Node {
public:
    TestPosition() : Node("test_position"), desired_position_rad_(0.0), position_received_(false) {
        odrive_ = std::make_shared<can_odrive_interface::ODriveInterface>();
        
        // Khởi tạo chế độ position cho Node 1
        // odrive_->InitPositionMode(1);

        // Tạo subscriber cho topic /desired_position
        position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/desired_position", 10,
            std::bind(&TestPosition::position_callback, this, std::placeholders::_1));

        // Tạo timer để gửi lệnh vị trí mỗi 2 giây
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TestPosition::send_position, this));
        
        RCLCPP_INFO(this->get_logger(), "Test Position Node started");
    }

private:
    void position_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        desired_position_rad_ = msg->data;
        position_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Nhận vị trí mong muốn: %.2f rad", desired_position_rad_);
    }

    void send_position() {
        if (!position_received_) {
            RCLCPP_WARN(this->get_logger(), "Chưa nhận vị trí từ topic /desired_position, không gửi lệnh");
            return;
        }

        float position_turns = desired_position_rad_ / REV_TO_RAD; // Chuyển radian sang turns
        odrive_->SendPositionCommand(1, position_turns);
        RCLCPP_INFO(this->get_logger(), "Gửi vị trí %.2f rad (%.2f turns) cho Node 1", 
                    desired_position_rad_, position_turns);
    }

    std::shared_ptr<can_odrive_interface::ODriveInterface> odrive_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_sub_;
    float desired_position_rad_;
    bool position_received_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPosition>());
    rclcpp::shutdown();
    return 0;
}













// *Encoder la tuyet doi khong thay doi khi tat vi tri encoder giu nguyen khong thay doi