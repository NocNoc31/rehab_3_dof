#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <termios.h>
#include <fcntl.h>

class ODriveCAN : public rclcpp::Node {
public:
    ODriveCAN() : Node("odrive_can_node"), running_(true) {
        RCLCPP_INFO(this->get_logger(), "ODrive CAN Node Started! Nhấn 'q' để dừng động cơ.");

        // 1️⃣ Tạo socket CAN
        sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Không thể mở socket CAN!");
            return;
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, "can0");  
        if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Không thể lấy chỉ mục CAN interface!");
            close(sock);
            return;
        }

        struct sockaddr_can addr = {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Không thể kết nối CAN!");
            close(sock);
            return;
        }

        // 2️⃣ Khởi tạo ODrive
        initialize_odrive(0); // Node ID 0
        initialize_odrive(1); // Node ID 1

        // 3️⃣ Tạo timer gửi lệnh định kỳ
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ODriveCAN::send_velocity_commands, this)
        );

        // 4️⃣ Bắt đầu luồng đọc Heartbeat
        reader_thread_ = std::thread(&ODriveCAN::read_can_messages, this);

        // 5️⃣ Bắt đầu luồng đọc phím
        keyboard_thread_ = std::thread(&ODriveCAN::read_keyboard, this);
    }

    ~ODriveCAN() {
        running_ = false;
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        if (sock >= 0) {
            close(sock);
        }
    }

    // Gửi lệnh vận tốc
    void send_velocity_commands() {
        send_velocity_command(0, 2.0);  // Node ID 0, vận tốc 2.0
        send_velocity_command(1, -2.0); // Node ID 1, vận tốc -2.0
    }

    // Hàm gửi vận tốc qua CAN
    void send_velocity_command(int node_id, float velocity) {
        struct can_frame frame;
        frame.can_id = (node_id << 5) | 0x0D; // Set_Input_Vel (0x0D)
        frame.can_dlc = 8;

        std::memcpy(frame.data, &velocity, sizeof(float));
        std::memset(frame.data + 4, 0, 4);

        if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh CAN cho động cơ %d!", node_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "Gửi vận tốc %.2f đến ODrive ID %d", velocity, node_id);
        }
    }

    // Hàm dừng cả hai động cơ
    void stop_motors() {
        RCLCPP_INFO(this->get_logger(), "Dừng cả hai động cơ...");
        send_velocity_command(0, 0.0); // Dừng Node 0
        send_velocity_command(1, 0.0); // Dừng Node 1
        // Tắt timer để không gửi vận tốc nữa
        timer_->cancel();
    }

private:
    // Khởi tạo ODrive
    void initialize_odrive(int node_id) {
        send_can_frame(node_id, 0x07, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // AXIS_STATE_IDLE
        usleep(2000000);

        send_can_frame(node_id, 0x07, {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        usleep(15000000);

        send_can_frame(node_id, 0x0B, {0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}); // VELOCITY_CONTROL (2), PASSTHROUGH (1)
        usleep(2000000);

        send_can_frame(node_id, 0x07, {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // AXIS_STATE_CLOSED_LOOP_CONTROL
        usleep(2000000);
    }

    // Gửi frame CAN
    void send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data) {
        struct can_frame frame;
        frame.can_id = (node_id << 5) | cmd_id;
        frame.can_dlc = 8;

        size_t i = 0;
        for (auto byte : data) {
            if (i < 8) frame.data[i++] = byte;
        }
        while (i < 8) frame.data[i++] = 0;

        if (write(sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi gửi frame CAN cho node %d, cmd_id 0x%02x!", node_id, cmd_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "Gửi frame CAN cho node %d, cmd_id 0x%02x", node_id, cmd_id);
        }
    }

    // Đọc thông điệp CAN
    void read_can_messages() {
        struct can_frame frame;
        while (running_ && rclcpp::ok()) {
            ssize_t nbytes = read(sock, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                RCLCPP_ERROR(this->get_logger(), "Lỗi đọc dữ liệu CAN!");
                continue;
            }

            if (nbytes == sizeof(struct can_frame)) {
                if (frame.can_id == 0x01) {
                    process_heartbeat(0, frame.data);
                } else if (frame.can_id == 0x21) {
                    process_heartbeat(1, frame.data);
                }
            }
        }
    }

    // Xử lý Heartbeat
    void process_heartbeat(int node_id, uint8_t* data) {
        uint32_t axis_error = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
        uint8_t axis_state = data[5];
        RCLCPP_INFO(this->get_logger(), "Heartbeat từ Node %d: Axis Error = 0x%08x, Axis State = %d",
                    node_id, axis_error, axis_state);
    }

    // Đọc phím từ bàn phím
    void read_keyboard() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt); // Lấy cấu hình terminal hiện tại
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // Tắt chế độ canonical và echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Áp dụng cấu hình mới

        while (running_ && rclcpp::ok()) {
            char c = getchar();
            if (c == 'q') {
                stop_motors();
                running_ = false; // Thoát chương trình sau khi dừng
                rclcpp::shutdown(); // Dừng ROS
                break;
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Khôi phục cấu hình terminal
    }

    int sock;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread reader_thread_;
    std::thread keyboard_thread_;
    bool running_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ODriveCAN>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}