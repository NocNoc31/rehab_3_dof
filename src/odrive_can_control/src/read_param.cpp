#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <std_msgs/msg/float32.hpp>

class ReadParam : public rclcpp::Node {
public:
    ReadParam() : Node("read_param"), running_(true) {
        RCLCPP_INFO(this->get_logger(), "Read Param Node Started!");

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

        // Tạo publisher cho Node 0
        pos_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("/motor/position_0", 10);
        vel_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("/motor/velocity_0", 10);

        // Tạo publisher cho Node 1
        pos_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("/motor/position_1", 10);
        vel_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("/motor/velocity_1", 10);

        // Tạo timer gửi yêu cầu đọc dữ liệu mỗi 100ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ReadParam::request_params, this));

        // Bắt đầu luồng đọc dữ liệu từ CAN
        reader_thread_ = std::thread(&ReadParam::read_can_messages, this);
    }

    ~ReadParam() {
        running_ = false;
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:
    void request_params() {
        // Gửi yêu cầu Get_Encoder_Estimates cho Node 0
        send_can_frame(0, 0x009, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});

        // Gửi yêu cầu Get_Encoder_Estimates cho Node 1
        send_can_frame(1, 0x009, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    }

    void send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data) {
        struct can_frame frame;
        frame.can_id = (node_id << 5) | cmd_id;
        frame.can_dlc = 8;

        size_t i = 0;
        for (auto byte : data) {
            if (i < 8) frame.data[i++] = byte;
        }
        while (i < 8) frame.data[i++] = 0;

        if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi gửi frame CAN cho node %d, cmd_id 0x%02x!", node_id, cmd_id);
        }
    }

    void read_can_messages() {
        struct can_frame frame;
        while (running_ && rclcpp::ok()) {
            ssize_t nbytes = read(sock_, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                RCLCPP_ERROR(this->get_logger(), "Lỗi đọc dữ liệu CAN!");
                continue;
            }

            if (nbytes == sizeof(struct can_frame)) {
                if (frame.can_id == (0 << 5 | 0x009)) { // Phản hồi từ Node 0
                    process_params(0, frame.data);
                } else if (frame.can_id == (1 << 5 | 0x009)) { // Phản hồi từ Node 1
                    process_params(1, frame.data);
                }
            }
        }
    }

    void process_params(int node_id, uint8_t* data) {
        // Theo tài liệu ODrive CAN: 
        // - Byte 0-3: Pos_Estimate (float32)
        // - Byte 4-7: Vel_Estimate (float32)
        float pos_estimate;
        float vel_estimate;
        std::memcpy(&pos_estimate, data, sizeof(float));
        std::memcpy(&vel_estimate, data + 4, sizeof(float));

        // Log dữ liệu
        RCLCPP_INFO(this->get_logger(), "Node %d: Pos_Estimate = %.3f turns, Vel_Estimate = %.3f turns/s",
                    node_id, pos_estimate, vel_estimate);

        // Publish dữ liệu
        std_msgs::msg::Float32 pos_msg;
        std_msgs::msg::Float32 vel_msg;
        pos_msg.data = pos_estimate;
        vel_msg.data = vel_estimate;

        if (node_id == 0) {
            pos_pub_0_->publish(pos_msg);
            vel_pub_0_->publish(vel_msg);
        } else if (node_id == 1) {
            pos_pub_1_->publish(pos_msg);
            vel_pub_1_->publish(vel_msg);
        }
    }

    int sock_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread reader_thread_;
    bool running_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_pub_0_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_pub_0_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_pub_1_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_pub_1_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadParam>());
    rclcpp::shutdown();
    return 0;
}