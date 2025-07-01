// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/float32.hpp>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <net/if.h>
// #include <sys/ioctl.h>
// #include <sys/socket.h>
// #include <poll.h>
// #include <cstring>
// #include <iostream>
// #include <unistd.h>

// #define KV 170.0  // Giá trị KV của động cơ

// class DataPubNode : public rclcpp::Node {
// public:
//     DataPubNode() : Node("data_pub") {
//         position_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/position", 10);
//         velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/velocity", 10);
//         iq_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/iq_setpoint", 10);
//         iq_measured_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/iq_measured", 10);
//         torque_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/torque_setpoint", 10);
//         torque_actual_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/torque_actual", 10);

//         socket_can_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//         if (socket_can_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to open CAN socket!");
//             rclcpp::shutdown();
//         }

//         struct ifreq ifr;
//         strcpy(ifr.ifr_name, "can0");
//         ioctl(socket_can_, SIOCGIFINDEX, &ifr);

//         struct sockaddr_can addr;
//         addr.can_family = AF_CAN;
//         addr.can_ifindex = ifr.ifr_ifindex;
//         if (bind(socket_can_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket!");
//             rclcpp::shutdown();
//         }

//         // Chạy vòng lặp đọc dữ liệu ngay khi có (poll)
//         thread_ = std::thread(&DataPubNode::read_can_data, this);
//     }

//     ~DataPubNode() {
//         if (thread_.joinable()) {
//             thread_.join();
//         }
//         close(socket_can_);
//     }

// private:
//     int socket_can_;
//     std::thread thread_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iq_setpoint_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iq_measured_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_setpoint_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_actual_pub_;

//     float bytes_to_float(const uint8_t* bytes) {
//         float value;
//         std::memcpy(&value, bytes, sizeof(float));
//         return value;
//     }

//     void read_can_data() {
//         struct pollfd pfd;
//         pfd.fd = socket_can_;
//         pfd.events = POLLIN;

//         while (rclcpp::ok()) {
//             int ret = poll(&pfd, 1, 10); // Timeout 10ms
//             if (ret > 0 && (pfd.revents & POLLIN)) {
//                 struct can_frame frame;
//                 int nbytes = read(socket_can_, &frame, sizeof(struct can_frame));
//                 if (nbytes > 0) {
//                     process_can_data(frame);
//                 }
//             }
//         }
//     }

//     void process_can_data(const struct can_frame& frame) {
//         if (frame.can_dlc != 8 && frame.can_id != 0x0E) {
//             RCLCPP_WARN(this->get_logger(), "Invalid CAN frame size");
//             return;
//         }

//         if (frame.can_id == 0x09) {
//             float position = bytes_to_float(frame.data);
//             float velocity = bytes_to_float(frame.data + 4);
//             publish_float(position_pub_, position);
//             publish_float(velocity_pub_, velocity);
//         } else if (frame.can_id == 0x014) {  // Iq data
//             float iq_setpoint = bytes_to_float(frame.data);
//             float iq_measured = bytes_to_float(frame.data + 4);

//             float torque_setpoint = (iq_setpoint * 8.27) / KV;
//             float torque_actual = (iq_measured * 8.27) / KV;

//             // float torque_setpoint = (iq_setpoint * 0.4300000071525574);
//             // float torque_actual = (iq_measured * 0.4300000071525574);

//             publish_float(iq_setpoint_pub_, iq_setpoint);
//             publish_float(iq_measured_pub_, iq_measured);
//             publish_float(torque_setpoint_pub_, torque_setpoint);
//             publish_float(torque_actual_pub_, torque_actual);
//         }
//     }

//     void publish_float(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& pub, float value) {
//         auto msg = std_msgs::msg::Float32();
//         msg.data = value;
//         pub->publish(msg);
//     }
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<DataPubNode>());
//     rclcpp::shutdown();
//     return 0;
// }




// /////TEST ID NODE = 1

// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/float32.hpp>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <net/if.h>
// #include <sys/ioctl.h>
// #include <sys/socket.h>
// #include <poll.h>
// #include <cstring>
// #include <iostream>
// #include <unistd.h>

// #define KV 170.0  // Giá trị KV của động cơ

// class DataPubNode : public rclcpp::Node {
// public:
//     DataPubNode() : Node("data_pub_node_1") {
//         // Publisher cho Node ID 1
//         position_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/position_1", 10);
//         velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/velocity_1", 10);
//         iq_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/iq_setpoint_1", 10);
//         iq_measured_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/iq_measured_1", 10);
//         torque_setpoint_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/torque_setpoint_1", 10);
//         torque_actual_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor/torque_actual_1", 10);

//         socket_can_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//         if (socket_can_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to open CAN socket!");
//             rclcpp::shutdown();
//         }

//         struct ifreq ifr;
//         strcpy(ifr.ifr_name, "can0");
//         ioctl(socket_can_, SIOCGIFINDEX, &ifr);

//         struct sockaddr_can addr;
//         addr.can_family = AF_CAN;
//         addr.can_ifindex = ifr.ifr_ifindex;
//         if (bind(socket_can_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket!");
//             rclcpp::shutdown();
//         }

//         // Chạy vòng lặp đọc dữ liệu ngay khi có (poll)
//         thread_ = std::thread(&DataPubNode::read_can_data, this);
//         RCLCPP_INFO(this->get_logger(), "DataPubNode started for Node ID 1");
//     }

//     ~DataPubNode() {
//         if (thread_.joinable()) {
//             thread_.join();
//         }
//         close(socket_can_);
//     }

// private:
//     int socket_can_;
//     std::thread thread_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iq_setpoint_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iq_measured_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_setpoint_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_actual_pub_;

//     float bytes_to_float(const uint8_t* bytes) {
//         float value;
//         std::memcpy(&value, bytes, sizeof(float));
//         return value;
//     }

//     void read_can_data() {
//         struct pollfd pfd;
//         pfd.fd = socket_can_;
//         pfd.events = POLLIN;

//         while (rclcpp::ok()) {
//             int ret = poll(&pfd, 1, 1); // Giảm timeout xuống 1ms để tăng tần suất
//             if (ret > 0 && (pfd.revents & POLLIN)) {
//                 struct can_frame frame;
//                 int nbytes = read(socket_can_, &frame, sizeof(struct can_frame));
//                 if (nbytes > 0) {
//                     process_can_data(frame);
//                 }
//             }
//         }
//     }

//     void process_can_data(const struct can_frame& frame) {
//         // Kiểm tra độ dài frame (trừ lệnh gửi torque 0x2E cho Node 1)
//         if (frame.can_dlc != 8 && frame.can_id != ((1 << 5) | 0x0E)) {
//             RCLCPP_WARN(this->get_logger(), "Invalid CAN frame size or ID: 0x%03x", frame.can_id);
//             return;
//         }

//         int node_id = frame.can_id >> 5; // Lấy Node ID
//         int command_id = frame.can_id & 0x1F; // Lấy Command ID

//         // Chỉ xử lý dữ liệu từ Node ID 1
//         if (node_id != 1) {
//             return;
//         }

//         if (command_id == 0x09) { // Get_Encoder_Estimates (0x29 cho Node 1)
//             float position = bytes_to_float(frame.data);
//             float velocity = bytes_to_float(frame.data + 4);
//             publish_float(position_pub_, position);
//             publish_float(velocity_pub_, velocity);
//             RCLCPP_INFO(this->get_logger(), "Node 1: Pos = %.3f turns, Vel = %.3f turns/s", position, velocity);
//         } 
//         else if (command_id == 0x14) { // Get_Iq (0x34 cho Node 1)
//             float iq_setpoint = bytes_to_float(frame.data);
//             float iq_measured = bytes_to_float(frame.data + 4);
//             float torque_setpoint = (iq_setpoint * 8.27) / KV;
//             float torque_actual = (iq_measured * 8.27) / KV;

//             publish_float(iq_setpoint_pub_, iq_setpoint);
//             publish_float(iq_measured_pub_, iq_measured);
//             publish_float(torque_setpoint_pub_, torque_setpoint);
//             publish_float(torque_actual_pub_, torque_actual);
//             RCLCPP_INFO(this->get_logger(), "Node 1: Iq_set = %.3f A, Iq_meas = %.3f A, Torque_set = %.3f Nm, Torque_act = %.3f Nm", 
//                         iq_setpoint, iq_measured, torque_setpoint, torque_actual);
//         }
//     }

//     void publish_float(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& pub, float value) {
//         auto msg = std_msgs::msg::Float32();
//         msg.data = value;
//         pub->publish(msg);
//     }
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<DataPubNode>());
//     rclcpp::shutdown();
//     return 0;
// }





////Both 


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <poll.h>
#include <cstring>
#include <iostream>
#include <unistd.h>

#define KV 170.0  // Giá trị KV của động cơ

class DataPubNode : public rclcpp::Node {
public:
    DataPubNode() : Node("data_pub") {
        // Publisher cho Node ID 0
        position_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("motor/position_0", 10);
        velocity_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("motor/velocity_0", 10);
        iq_setpoint_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("motor/iq_setpoint_0", 10);
        iq_measured_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("motor/iq_measured_0", 10);
        torque_setpoint_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("motor/torque_setpoint_0", 10);
        torque_actual_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("motor/torque_actual_0", 10);

        // Publisher cho Node ID 1
        position_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("motor/position_1", 10);
        velocity_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("motor/velocity_1", 10);
        iq_setpoint_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("motor/iq_setpoint_1", 10);
        iq_measured_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("motor/iq_measured_1", 10);
        torque_setpoint_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("motor/torque_setpoint_1", 10);
        torque_actual_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("motor/torque_actual_1", 10);

        socket_can_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_can_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN socket!");
            rclcpp::shutdown();
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, "can0");
        ioctl(socket_can_, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socket_can_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket!");
            rclcpp::shutdown();
        }

        // Chạy vòng lặp đọc dữ liệu ngay khi có (poll)
        thread_ = std::thread(&DataPubNode::read_can_data, this);
        RCLCPP_INFO(this->get_logger(), "DataPubNode started for Node ID 0 and Node ID 1");
    }

    ~DataPubNode() {
        if (thread_.joinable()) {
            thread_.join();
        }
        close(socket_can_);
    }

private:
    int socket_can_;
    std::thread thread_;

    // Publisher cho Node ID 0
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_0_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_0_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iq_setpoint_pub_0_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iq_measured_pub_0_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_setpoint_pub_0_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_actual_pub_0_;

    // Publisher cho Node ID 1
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_1_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_1_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iq_setpoint_pub_1_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iq_measured_pub_1_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_setpoint_pub_1_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_actual_pub_1_;

    float bytes_to_float(const uint8_t* bytes) {
        float value;
        std::memcpy(&value, bytes, sizeof(float));
        return value;
    }

    void read_can_data() {
        struct pollfd pfd;
        pfd.fd = socket_can_;
        pfd.events = POLLIN;

        while (rclcpp::ok()) {
            int ret = poll(&pfd, 1, 1); // Timeout 1ms để tăng tần suất
            if (ret > 0 && (pfd.revents & POLLIN)) {
                struct can_frame frame;
                int nbytes = read(socket_can_, &frame, sizeof(struct can_frame));
                if (nbytes > 0) {
                    process_can_data(frame);
                }
            }
        }
    }

    void process_can_data(const struct can_frame& frame) {
        // Kiểm tra độ dài frame (trừ lệnh gửi torque: 0x0E cho Node 0, 0x2E cho Node 1)
        if (frame.can_dlc != 8 && frame.can_id != ((0 << 5) | 0x0E) && frame.can_id != ((1 << 5) | 0x0E)) {
            RCLCPP_WARN(this->get_logger(), "Invalid CAN frame size or ID: 0x%03x", frame.can_id);
            return;
        }

        int node_id = frame.can_id >> 5; // Lấy Node ID
        int command_id = frame.can_id & 0x1F; // Lấy Command ID

        if (command_id == 0x09) { // Get_Encoder_Estimates
            float position = bytes_to_float(frame.data);
            float velocity = bytes_to_float(frame.data + 4);
            if (node_id == 0) { // Node ID 0 (CAN ID = 0x09)
                publish_float(position_pub_0_, position);
                publish_float(velocity_pub_0_, velocity);
                RCLCPP_INFO(this->get_logger(), "Node 0: Pos = %.3f turns, Vel = %.3f turns/s", position, velocity);
            } else if (node_id == 1) { // Node ID 1 (CAN ID = 0x29)
                publish_float(position_pub_1_, position);
                publish_float(velocity_pub_1_, velocity);
                RCLCPP_INFO(this->get_logger(), "Node 1: Pos = %.3f turns, Vel = %.3f turns/s", position, velocity);
            }
        } 
        else if (command_id == 0x14) { // Get_Iq
            float iq_setpoint = bytes_to_float(frame.data);
            float iq_measured = bytes_to_float(frame.data + 4);
            float torque_setpoint = (iq_setpoint * 8.27) / KV;
            float torque_actual = (iq_measured * 8.27) / KV;

            if (node_id == 0) { // Node ID 0 (CAN ID = 0x14)
                publish_float(iq_setpoint_pub_0_, iq_setpoint);
                publish_float(iq_measured_pub_0_, iq_measured);
                publish_float(torque_setpoint_pub_0_, torque_setpoint);
                publish_float(torque_actual_pub_0_, torque_actual);
                RCLCPP_INFO(this->get_logger(), "Node 0: Iq_set = %.3f A, Iq_meas = %.3f A, Torque_set = %.3f Nm, Torque_act = %.3f Nm", 
                            iq_setpoint, iq_measured, torque_setpoint, torque_actual);
            } else if (node_id == 1) { // Node ID 1 (CAN ID = 0x34)
                publish_float(iq_setpoint_pub_1_, iq_setpoint);
                publish_float(iq_measured_pub_1_, iq_measured);
                publish_float(torque_setpoint_pub_1_, torque_setpoint);
                publish_float(torque_actual_pub_1_, torque_actual);
                RCLCPP_INFO(this->get_logger(), "Node 1: Iq_set = %.3f A, Iq_meas = %.3f A, Torque_set = %.3f Nm, Torque_act = %.3f Nm", 
                            iq_setpoint, iq_measured, torque_setpoint, torque_actual);
            }
        }
    }

    void publish_float(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& pub, float value) {
        auto msg = std_msgs::msg::Float32();
        msg.data = value;
        pub->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataPubNode>());
    rclcpp::shutdown();
    return 0;
}