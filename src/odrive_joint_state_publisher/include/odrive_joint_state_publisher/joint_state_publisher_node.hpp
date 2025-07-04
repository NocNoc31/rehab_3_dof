// #ifndef ODRIVE_JOINT_STATE_PUBLISHER_NODE_HPP
// #define ODRIVE_JOINT_STATE_PUBLISHER_NODE_HPP

// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/float32.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <can_odrive_interface/odrive_interface.hpp>
// #include <map>
// #include <vector>
// #include <string>
// #include <chrono>

// class JointStatePublisherNode : public rclcpp::Node {
// public:
//     JointStatePublisherNode();

// private:
//     void pos_callback_0(const std_msgs::msg::Float32::SharedPtr msg);
//     void vel_callback_0(const std_msgs::msg::Float32::SharedPtr msg);
//     void pos_callback_1(const std_msgs::msg::Float32::SharedPtr msg);
//     void vel_callback_1(const std_msgs::msg::Float32::SharedPtr msg);
//     void pos_callback_2(const std_msgs::msg::Float32::SharedPtr msg);
//     void vel_callback_2(const std_msgs::msg::Float32::SharedPtr msg);
//     void pos_callback_3(const std_msgs::msg::Float32::SharedPtr msg);
//     void vel_callback_3(const std_msgs::msg::Float32::SharedPtr msg);
//     void publish_joint_states();
//     void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
//     void execute_trajectory_callback();
//     void send_command_to_odrive(const std::string& joint_name, double position);

//     std::map<int, std::string> odrive_node_to_joint_name_map_;
//     std::map<std::string, double> current_joint_positions_;
//     std::map<std::string, double> current_joint_velocities_;
//     std::vector<std::string> joint_names_;
//     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_sub_0_;
//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_0_;
//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_sub_1_;
//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_1_;
//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_sub_2_;
//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_2_;
//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_sub_3_;
//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_3_;
//     rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
//     rclcpp::TimerBase::SharedPtr joint_state_timer_;
//     rclcpp::TimerBase::SharedPtr trajectory_execution_timer_;
//     std::shared_ptr<can_odrive_interface::ODriveInterface> odrive_interface_;
//     trajectory_msgs::msg::JointTrajectory::SharedPtr current_trajectory_;
//     rclcpp::Time trajectory_start_time_;
// };

// #endif // ODRIVE_JOINT_STATE_PUBLISHER_NODE_HPP















#ifndef ODRIVE_JOINT_STATE_PUBLISHER_NODE_HPP
#define ODRIVE_JOINT_STATE_PUBLISHER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp> // Thêm thư viện action
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp> // Không còn cần thiết cho subscription trực tiếp
#include <control_msgs/action/follow_joint_trajectory.hpp> // Thêm action message type
#include <can_odrive_interface/odrive_interface.hpp>
#include <map>
#include <vector>
#include <string>
#include <chrono>
#include <thread> // Cần thiết cho việc chạy action trong một thread riêng

class JointStatePublisherNode : public rclcpp::Node {
public:
    JointStatePublisherNode();

private:
    // Định nghĩa kiểu cho FollowJointTrajectory Action
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    // Callbacks cho encoder
    void pos_callback_0(const std_msgs::msg::Float32::SharedPtr msg);
    void vel_callback_0(const std_msgs::msg::Float32::SharedPtr msg);
    void pos_callback_1(const std_msgs::msg::Float32::SharedPtr msg);
    void vel_callback_1(const std_msgs::msg::Float32::SharedPtr msg);
    void pos_callback_2(const std_msgs::msg::Float32::SharedPtr msg);
    void vel_callback_2(const std_msgs::msg::Float32::SharedPtr msg);
    void pos_callback_3(const std_msgs::msg::Float32::SharedPtr msg);
    void vel_callback_3(const std_msgs::msg::Float32::SharedPtr msg);

    // Xuất bản JointState
    void publish_joint_states();

    // Gửi lệnh tới ODrive
    void send_command_to_odrive(const std::string& joint_name, double position);

    // Các hàm callback của Action Server
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    // Hàm thực thi quỹ đạo cho Action Server
    void execute_trajectory_action(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);


    // Các biến thành viên
    std::map<int, std::string> odrive_node_to_joint_name_map_;
    std::map<std::string, double> current_joint_positions_;
    std::map<std::string, double> current_joint_velocities_;
    std::vector<std::string> joint_names_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_sub_0_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_0_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_sub_1_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_1_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_sub_2_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_2_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pos_sub_3_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_3_;

    // Thay thế trajectory_sub_ bằng action_server_
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    // trajectory_execution_timer_ sẽ được dùng nội bộ trong action, không cần là thành viên public/protected

    std::shared_ptr<can_odrive_interface::ODriveInterface> odrive_interface_;

    // Các biến cho việc thực thi quỹ đạo (giờ được quản lý bởi action)
    // trajectory_msgs::msg::JointTrajectory::SharedPtr current_trajectory_; // Không cần nữa vì goal handle sẽ giữ quỹ đạo
    // rclcpp::Time trajectory_start_time_; // Không cần nữa
};

#endif // ODRIVE_JOINT_STATE_PUBLISHER_NODE_HPP