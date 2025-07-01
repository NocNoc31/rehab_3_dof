// #include <rclcpp/rclcpp.hpp>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <net/if.h>
// #include <sys/ioctl.h>
// #include <sys/socket.h>
// #include <cstring>
// #include <unistd.h>
// #include <cmath>

// class ODriveInitializer : public rclcpp::Node {
// public:
//     ODriveInitializer() : Node("odrive_initializer") {
//         RCLCPP_INFO(this->get_logger(), "ODrive Initializer Node Started!");

//         this->declare_parameter<std::string>("control_mode_node_0", "velocity");
//         this->declare_parameter<std::string>("control_mode_node_1", "velocity");

//         std::string mode_node_0, mode_node_1;
//         this->get_parameter("control_mode_node_0", mode_node_0);
//         this->get_parameter("control_mode_node_1", mode_node_1);

//         sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//         if (sock_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Không thể mở socket CAN!");
//             rclcpp::shutdown();
//             return;
//         }

//         struct ifreq ifr;
//         strcpy(ifr.ifr_name, "can0");
//         if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Không thể lấy chỉ mục CAN interface!");
//             close(sock_);
//             rclcpp::shutdown();
//             return;
//         }

//         struct sockaddr_can addr = {};
//         addr.can_family = AF_CAN;
//         addr.can_ifindex = ifr.ifr_ifindex;

//         if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Không thể kết nối CAN!");
//             close(sock_);
//             rclcpp::shutdown();
//             return;
//         }

//         initialize_odrive(0, mode_node_0);
//         initialize_odrive(1, mode_node_1);

//         RCLCPP_INFO(this->get_logger(), "ODrive initialization completed!");
//     }

//     ~ODriveInitializer() {
//         if (sock_ >= 0) {
//             close(sock_);
//         }
//     }

// private:
//     void initialize_odrive(int node_id, const std::string& control_mode) {
//         RCLCPP_INFO(this->get_logger(), "Initializing Node %d with control mode: %s", node_id, control_mode.c_str());

//         // Đặt trạng thái IDLE trước khi cấu hình
//         if (send_can_frame(node_id, 0x07, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_IDLE for Node %d", node_id);
//             return;
//         }
//         RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_IDLE", node_id);
//         usleep(500000); // Đợi 0.5 giây

//         // Đặt chế độ TORQUE_CONTROL với INPUT_MODE_PASSTHROUGH (input_mode = 1)
//         if (send_can_frame(node_id, 0x0B, {0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to set TORQUE_CONTROL with PASSTHROUGH for Node %d", node_id);
//             return;
//         }
//         RCLCPP_INFO(this->get_logger(), "Node %d set to TORQUE_CONTROL with PASSTHROUGH", node_id);
//         usleep(500000); // Đợi 0.5 giây

//         // Kích hoạt trạng thái CLOSED_LOOP_CONTROL cho torque control
//         if (send_can_frame(node_id, 0x07, {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_CLOSED_LOOP_CONTROL for Node %d (TORQUE)", node_id);
//             return;
//         }
//         RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_CLOSED_LOOP_CONTROL (TORQUE)", node_id);
//         usleep(500000); // Đợi 0.5 giây

//         RCLCPP_INFO(this->get_logger(), "Node %d initialization completed with TORQUE_CONTROL", node_id);
//     }

//     int send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data) {
//         struct can_frame frame;
//         frame.can_id = (node_id << 5) | cmd_id;
//         frame.can_dlc = 8;

//         size_t i = 0;
//         for (auto byte : data) {
//             if (i < 8) frame.data[i++] = byte;
//         }
//         while (i < 8) frame.data[i++] = 0;

//         int bytes_written = write(sock_, &frame, sizeof(struct can_frame));
//         if (bytes_written != sizeof(struct can_frame)) {
//             RCLCPP_ERROR(this->get_logger(), "Lỗi gửi frame CAN cho node %d, cmd_id 0x%02x!", node_id, cmd_id);
//             return -1;
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Gửi frame CAN cho node %d, cmd_id 0x%02x", node_id, cmd_id);
//             return 0;
//         }
//     }

//     float get_pos_estimate(int node_id) {
//         if (send_can_frame(node_id, 0x09, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to request pos_estimate for Node %d", node_id);
//             return std::numeric_limits<float>::max();
//         }

//         struct can_frame frame;
//         int timeout = 200; // Tăng timeout lên 200ms để đảm bảo đọc chính xác
//         while (timeout--) {
//             int bytes_read = read(sock_, &frame, sizeof(struct can_frame));
//             if (bytes_read == sizeof(struct can_frame) && frame.can_id == ((node_id << 5) | 0x09)) {
//                 float pos_estimate;
//                 std::memcpy(&pos_estimate, frame.data, sizeof(float));
//                 return pos_estimate;
//             }
//             usleep(1000);
//         }
//         RCLCPP_ERROR(this->get_logger(), "Failed to read pos_estimate response for Node %d", node_id);
//         return std::numeric_limits<float>::max();
//     }

//     int sock_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ODriveInitializer>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }






#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>

class ODriveInitializer : public rclcpp::Node {
public:
    ODriveInitializer() : Node("odrive_initializer") {
        RCLCPP_INFO(this->get_logger(), "ODrive Initializer Node Started!");

        // Khai báo ROS 2 parameters để chọn chế độ điều khiển
        this->declare_parameter<std::string>("control_mode_node_0", "velocity"); // Mặc định torque cho node 0
        this->declare_parameter<std::string>("control_mode_node_1", "velocity"); // Mặc định velocity cho node 1

        // Lấy giá trị từ parameters
        std::string mode_node_0, mode_node_1;
        this->get_parameter("control_mode_node_0", mode_node_0);
        this->get_parameter("control_mode_node_1", mode_node_1);

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

        // Khởi tạo ODrive với chế độ được chọn
        initialize_odrive(0, mode_node_0);
        initialize_odrive(1, mode_node_1);

        RCLCPP_INFO(this->get_logger(), "ODrive initialization completed!");
        rclcpp::shutdown(); // Tắt node sau khi hoàn tất
    }

    ~ODriveInitializer() {
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:
    void initialize_odrive(int node_id, const std::string& control_mode) {
        // Log chế độ được chọn
        RCLCPP_INFO(this->get_logger(), "Initializing Node %d with control mode: %s", node_id, control_mode.c_str());

        // Đặt trạng thái IDLE
        if (send_can_frame(node_id, 0x07, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_IDLE for Node %d", node_id);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_IDLE", node_id);
        usleep(2000000); // Chờ 2 giây

        // Chạy hiệu chuẩn toàn phần
        if (send_can_frame(node_id, 0x07, {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_FULL_CALIBRATION_SEQUENCE for Node %d", node_id);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Node %d running AXIS_STATE_FULL_CALIBRATION_SEQUENCE", node_id);
        usleep(15000000); // Chờ 15 giây cho hiệu chuẩn

        // Chọn chế độ điều khiển dựa trên tham số
        uint8_t control_mode_value = 0x01; // Mặc định TORQUE_CONTROL
        if (control_mode == "velocity") {
            control_mode_value = 0x02; // VELOCITY_CONTROL
        } else if (control_mode != "torque") {
            RCLCPP_WARN(this->get_logger(), "Invalid control mode '%s' for Node %d, defaulting to TORQUE_CONTROL", 
                        control_mode.c_str(), node_id);
        }

        // Đặt chế độ điều khiển (TORQUE_CONTROL hoặc VELOCITY_CONTROL) và PASSTHROUGH
        if (send_can_frame(node_id, 0x0B, {control_mode_value, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set control mode for Node %d", node_id);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Node %d set to %s_CONTROL with PASSTHROUGH", 
                    node_id, (control_mode_value == 0x01 ? "TORQUE" : "VELOCITY"));
        usleep(2000000); // Chờ 2 giây

        // Kích hoạt CLOSED_LOOP_CONTROL
        if (send_can_frame(node_id, 0x07, {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_CLOSED_LOOP_CONTROL for Node %d", node_id);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_CLOSED_LOOP_CONTROL", node_id);
        usleep(2000000); // Chờ 2 giây

        RCLCPP_INFO(this->get_logger(), "Node %d initialization completed", node_id);
    }

    int send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data) {
        struct can_frame frame;
        frame.can_id = (node_id << 5) | cmd_id;
        frame.can_dlc = 8;

        size_t i = 0;
        for (auto byte : data) {
            if (i < 8) frame.data[i++] = byte;
        }
        while (i < 8) frame.data[i++] = 0;

        int bytes_written = write(sock_, &frame, sizeof(struct can_frame));
        if (bytes_written != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi gửi frame CAN cho node %d, cmd_id 0x%02x!", node_id, cmd_id);
            return -1; // Lỗi gửi
        } else {
            RCLCPP_INFO(this->get_logger(), "Gửi frame CAN cho node %d, cmd_id 0x%02x", node_id, cmd_id);
            return 0; // Thành công
        }
    }

    int sock_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ODriveInitializer>());
    rclcpp::shutdown();
    return 0;
}