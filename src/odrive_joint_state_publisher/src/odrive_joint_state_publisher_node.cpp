
// #include "odrive_joint_state_publisher/joint_state_publisher_node.hpp"

// // Các includes cần thiết cho action server
// #include <thread>
// #include <chrono>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <control_msgs/action/follow_joint_trajectory.hpp>


// JointStatePublisherNode::JointStatePublisherNode() : Node("odrive_joint_state_publisher_node") {
//     // --- Ánh xạ ODrive Node ID tới Tên khớp ---
//     odrive_node_to_joint_name_map_[0] = "joint_0";
//     odrive_node_to_joint_name_map_[1] = "joint_1";
//     odrive_node_to_joint_name_map_[2] = "joint_2";
//     odrive_node_to_joint_name_map_[3] = "joint_3";

//     // --- Cấu hình các tên khớp ---
//     joint_names_.push_back("joint_0");
//     joint_names_.push_back("joint_1");
//     joint_names_.push_back("joint_2");
//     joint_names_.push_back("joint_3");

//     // Khởi tạo trạng thái khớp ban đầu
//     for (const auto& name : joint_names_) {
//         current_joint_positions_[name] = 0.0;
//         current_joint_velocities_[name] = 0.0;
//     }

//     // --- Khởi tạo ODriveInterface ---
//     odrive_interface_ = std::make_shared<can_odrive_interface::ODriveInterface>();

//     // Khởi tạo chế độ position cho các node ODrive
//     // Chú ý: vòng lặp ban đầu của bạn chỉ chạy cho node_id = 1. Hãy kiểm tra lại nếu bạn có nhiều ODrive
//     for (int node_id = 0; node_id <= 3; ++node_id) { // Thay đổi để bao gồm tất cả các node ODrive có thể
//         odrive_interface_->InitPositionMode(node_id);
//     }

//     // --- Tạo Subscriptions cho encoder ---
//     pos_sub_0_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/position_0", 10, std::bind(&JointStatePublisherNode::pos_callback_0, this, std::placeholders::_1));
//     vel_sub_0_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/velocity_0", 10, std::bind(&JointStatePublisherNode::vel_callback_0, this, std::placeholders::_1));
//     pos_sub_1_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/position_1", 10, std::bind(&JointStatePublisherNode::pos_callback_1, this, std::placeholders::_1));
//     vel_sub_1_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/velocity_1", 10, std::bind(&JointStatePublisherNode::vel_callback_1, this, std::placeholders::_1));
//     pos_sub_2_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/position_2", 10, std::bind(&JointStatePublisherNode::pos_callback_2, this, std::placeholders::_1));
//     vel_sub_2_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/velocity_2", 10, std::bind(&JointStatePublisherNode::vel_callback_2, this, std::placeholders::_1));
//     pos_sub_3_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/position_3", 10, std::bind(&JointStatePublisherNode::pos_callback_3, this, std::placeholders::_1));
//     vel_sub_3_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/velocity_3", 10, std::bind(&JointStatePublisherNode::vel_callback_3, this, std::placeholders::_1));

//     // --- Tạo Publisher và Timer cho JointState ---
//     joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
//     joint_state_timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(10), // 100 Hz
//         std::bind(&JointStatePublisherNode::publish_joint_states, this)
//     );

//     // --- Khởi tạo Action Server cho FollowJointTrajectory ---
//     action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
//         this,
//         "rehab_arm_moveit_controller/follow_joint_trajectory", // Tên action server phải khớp với action_ns trong moveit_controllers.yaml
//         std::bind(&JointStatePublisherNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
//         std::bind(&JointStatePublisherNode::handle_cancel, this, std::placeholders::_1),
//         std::bind(&JointStatePublisherNode::handle_accepted, this, std::placeholders::_1));

//     // Loại bỏ subscription và timer cũ không dùng cho action server
//     // trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(...);
//     // trajectory_execution_timer_->cancel();

//     // Bắt đầu đọc encoder
//     odrive_interface_->ReadEncoder();

//     RCLCPP_INFO(this->get_logger(), "JointStatePublisherNode started and ready for 4-DOF robot control via FollowJointTrajectory Action.");
// }

// // --- Callbacks cho encoder ---
// void JointStatePublisherNode::pos_callback_0(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_positions_[odrive_node_to_joint_name_map_[0]] = msg->data;
// }
// void JointStatePublisherNode::vel_callback_0(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_velocities_[odrive_node_to_joint_name_map_[0]] = msg->data;
// }
// void JointStatePublisherNode::pos_callback_1(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_positions_[odrive_node_to_joint_name_map_[1]] = msg->data;
// }
// void JointStatePublisherNode::vel_callback_1(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_velocities_[odrive_node_to_joint_name_map_[1]] = msg->data;
// }
// void JointStatePublisherNode::pos_callback_2(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_positions_[odrive_node_to_joint_name_map_[2]] = msg->data;
// }
// void JointStatePublisherNode::vel_callback_2(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_velocities_[odrive_node_to_joint_name_map_[2]] = msg->data;
// }
// void JointStatePublisherNode::pos_callback_3(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_positions_[odrive_node_to_joint_name_map_[3]] = msg->data;
// }
// void JointStatePublisherNode::vel_callback_3(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_velocities_[odrive_node_to_joint_name_map_[3]] = msg->data;
// }

// // --- Xuất bản JointState ---
// void JointStatePublisherNode::publish_joint_states() {
//     sensor_msgs::msg::JointState joint_state_msg;
//     joint_state_msg.header.stamp = this->now();
//     joint_state_msg.name = joint_names_;

//     for (const auto& name : joint_names_) {
//         joint_state_msg.position.push_back(current_joint_positions_[name]);
//         joint_state_msg.velocity.push_back(current_joint_velocities_[name]);
//     }
//     joint_state_pub_->publish(joint_state_msg);
// }

// // --- Các hàm callback của Action Server ---

// // Xử lý Goal mới
// rclcpp_action::GoalResponse JointStatePublisherNode::handle_goal(
//     const rclcpp_action::GoalUUID & uuid,
//     std::shared_ptr<const FollowJointTrajectory::Goal> goal)
// {
//     RCLCPP_INFO(this->get_logger(), "Received goal request for trajectory with %zu points.", goal->trajectory.points.size());
//     (void)uuid; // Sử dụng uuid để tránh cảnh báo compiler

//     // Kiểm tra xem quỹ đạo có rỗng không
//     if (goal->trajectory.points.empty()) {
//         RCLCPP_WARN(this->get_logger(), "Received empty trajectory goal, rejecting.");
//         return rclcpp_action::GoalResponse::REJECT;
//     }

//     // Kiểm tra số lượng khớp có khớp không
//     if (goal->trajectory.joint_names.size() != joint_names_.size()) {
//         RCLCPP_ERROR(this->get_logger(), "Received trajectory with %zu joints, but expected %zu joints. Rejecting goal.",
//                      goal->trajectory.joint_names.size(), joint_names_.size());
//         return rclcpp_action::GoalResponse::REJECT;
//     }

//     // TODO: Có thể thêm các kiểm tra khác về quỹ đạo (ví dụ: các khớp có khớp với robot không)

//     RCLCPP_INFO(this->get_logger(), "Accepting trajectory goal.");
//     return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER; // Chấp nhận và sẽ xử lý sau trong handle_accepted
// }

// // Xử lý yêu cầu hủy bỏ Goal
// rclcpp_action::CancelResponse JointStatePublisherNode::handle_cancel(
//     const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
// {
//     RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
//     (void)goal_handle; // Sử dụng goal_handle để tránh cảnh báo compiler
//     return rclcpp_action::CancelResponse::ACCEPT; // Luôn chấp nhận yêu cầu hủy bỏ
// }

// // Xử lý khi Goal được chấp nhận
// void JointStatePublisherNode::handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
//     // Bắt đầu thực thi quỹ đạo trong một thread riêng để không chặn luồng chính của node
//     std::thread{std::bind(&JointStatePublisherNode::execute_trajectory_action, this, goal_handle)}.detach();
// }

// // --- Thực thi quỹ đạo Action ---
// void JointStatePublisherNode::execute_trajectory_action(
//     const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
// {
//     RCLCPP_INFO(this->get_logger(), "Executing trajectory goal.");
//     rclcpp::Rate loop_rate(100); // Tốc độ nội suy và gửi lệnh (100 Hz)
//     const auto goal = goal_handle->get_goal();
//     auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
//     auto result = std::make_shared<FollowJointTrajectory::Result>();

//     const auto& trajectory_points = goal->trajectory.points;
//     const auto& joint_names_in_trajectory = goal->trajectory.joint_names;
//     rclcpp::Time trajectory_start_time = this->now();

//     size_t current_point_idx = 0;

//     while (rclcpp::ok() && current_point_idx < trajectory_points.size()) {
//         if (goal_handle->is_canceling()) {
//             goal_handle->canceled(result);
//             RCLCPP_INFO(this->get_logger(), "Trajectory goal canceled.");
//             return;
//         }

//         rclcpp::Duration elapsed_time = this->now() - trajectory_start_time;
//         double current_time_sec = elapsed_time.seconds();

//         size_t prev_point_idx = 0;
//         size_t next_point_idx = 0;

//         // Tìm điểm quỹ đạo hiện tại và tiếp theo để nội suy
//         while (next_point_idx < trajectory_points.size() &&
//                (trajectory_points[next_point_idx].time_from_start.sec +
//                 trajectory_points[next_point_idx].time_from_start.nanosec * 1e-9) < current_time_sec) {
//             prev_point_idx = next_point_idx;
//             next_point_idx++;
//         }

//         // Nếu đã đi qua tất cả các điểm, kết thúc quỹ đạo
//         if (prev_point_idx >= trajectory_points.size() - 1) {
//             const auto& final_point = trajectory_points.back();
//             if (final_point.positions.size() == joint_names_in_trajectory.size()) {
//                 for (size_t i = 0; i < final_point.positions.size(); ++i) {
//                     const std::string& joint_name = joint_names_in_trajectory[i];
//                     double target_pos = final_point.positions[i];
//                     send_command_to_odrive(joint_name, target_pos);
//                 }
//             } else {
//                 RCLCPP_ERROR(this->get_logger(), "Final point position size mismatch with joint names in trajectory. Setting goal as ABORTED.");
//                 result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
//                 goal_handle->abort(result);
//                 return;
//             }
//             RCLCPP_INFO(this->get_logger(), "Trajectory finished. Sent final point commands. Setting goal as SUCCEEDED.");
//             result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
//             goal_handle->succeed(result);
//             return;
//         }

//         // Nội suy vị trí
//         const auto& p_prev = trajectory_points[prev_point_idx];
//         const auto& p_next = trajectory_points[next_point_idx];

//         double t_prev = p_prev.time_from_start.sec + p_prev.time_from_start.nanosec * 1e-9;
//         double t_next = p_next.time_from_start.sec + p_next.time_from_start.nanosec * 1e-9;

//         double fraction = 0.0;
//         if ((t_next - t_prev) > 1e-9) { // Tránh chia cho 0
//             fraction = (current_time_sec - t_prev) / (t_next - t_prev);
//         }

//         if (p_prev.positions.size() == joint_names_in_trajectory.size() &&
//             p_next.positions.size() == joint_names_in_trajectory.size()) {
//             for (size_t i = 0; i < p_prev.positions.size(); ++i) {
//                 const std::string& joint_name = joint_names_in_trajectory[i];
//                 double target_pos = p_prev.positions[i] + fraction * (p_next.positions[i] - p_prev.positions[i]);
//                 send_command_to_odrive(joint_name, target_pos);
//             }
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Trajectory point position size mismatch with joint names in trajectory during interpolation. Setting goal as ABORTED.");
//             result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
//             goal_handle->abort(result);
//             return;
//         }

//         // Cập nhật và xuất bản feedback (có thể bỏ qua nếu không cần thiết quá chi tiết)
//         feedback->header.stamp = this->now();
//         feedback->joint_names = joint_names_in_trajectory;
//         feedback->actual.positions = {}; // Bạn có thể điền vào vị trí thực tế của robot nếu có
//         feedback->desired.positions = {}; // Vị trí mong muốn từ quỹ đạo
//         feedback->error.positions = {}; // Sai số giữa thực tế và mong muốn
//         // Để điền các trường này, bạn cần truy cập current_joint_positions_ và tính toán
//         // Ví dụ: feedback->actual.positions.push_back(current_joint_positions_[joint_name]);
//         goal_handle->publish_feedback(feedback);

//         loop_rate.sleep();
//     }
// }


// // --- Gửi lệnh tới ODrive ---
// void JointStatePublisherNode::send_command_to_odrive(const std::string& joint_name, double position) {
//     int odrive_node_id = -1;
//     for (const auto& [node_id, name] : odrive_node_to_joint_name_map_) {
//         if (name == joint_name) {
//             odrive_node_id = node_id;
//             break;
//         }
//     }

//     if (odrive_node_id != -1) {
//         float pos_float = static_cast<float>(position);
//         odrive_interface_->SendPositionCommand(odrive_node_id, pos_float);
//         // RCLCPP_INFO(this->get_logger(), "Commanding ODrive Node %d (%s): Pos=%.3f rad", // Log này có thể quá nhiều
//         //             odrive_node_id, joint_name.c_str(), position);
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "Unknown joint name: %s. Cannot send command to ODrive.", joint_name.c_str());
//     }
// }

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<JointStatePublisherNode>());
//     rclcpp::shutdown();
//     return 0;
// }





// #include "odrive_joint_state_publisher/joint_state_publisher_node.hpp"

// // Các includes cần thiết cho action server
// #include <thread>
// #include <chrono>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <control_msgs/action/follow_joint_trajectory.hpp>


// JointStatePublisherNode::JointStatePublisherNode() : Node("odrive_joint_state_publisher_node") {
//     // --- Ánh xạ ODrive Node ID tới Tên khớp ---
//     odrive_node_to_joint_name_map_[0] = "joint_0";
//     odrive_node_to_joint_name_map_[1] = "joint_1";
//     odrive_node_to_joint_name_map_[2] = "joint_2";
//     odrive_node_to_joint_name_map_[3] = "joint_3";

//     // --- Cấu hình các tên khớp ---
//     joint_names_.push_back("joint_0");
//     joint_names_.push_back("joint_1");
//     joint_names_.push_back("joint_2");
//     joint_names_.push_back("joint_3");

//     // Khởi tạo trạng thái khớp ban đầu
//     for (const auto& name : joint_names_) {
//         current_joint_positions_[name] = 0.0;
//         current_joint_velocities_[name] = 0.0;
//     }

//     // --- Khởi tạo ODriveInterface ---
//     odrive_interface_ = std::make_shared<can_odrive_interface::ODriveInterface>();

//     // Khởi tạo chế độ position cho các node ODrive
//     // Chú ý: vòng lặp ban đầu của bạn chỉ chạy cho node_id = 1. Hãy kiểm tra lại nếu bạn có nhiều ODrive
//     for (int node_id = 0; node_id <= 3; ++node_id) { // Thay đổi để bao gồm tất cả các node ODrive có thể
//         odrive_interface_->InitPositionMode(node_id);
//     }

//     // --- Tạo Subscriptions cho encoder ---
//     pos_sub_0_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/position_0", 10, std::bind(&JointStatePublisherNode::pos_callback_0, this, std::placeholders::_1));
//     vel_sub_0_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/velocity_0", 10, std::bind(&JointStatePublisherNode::vel_callback_0, this, std::placeholders::_1));
//     pos_sub_1_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/position_1", 10, std::bind(&JointStatePublisherNode::pos_callback_1, this, std::placeholders::_1));
//     vel_sub_1_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/velocity_1", 10, std::bind(&JointStatePublisherNode::vel_callback_1, this, std::placeholders::_1));
//     pos_sub_2_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/position_2", 10, std::bind(&JointStatePublisherNode::pos_callback_2, this, std::placeholders::_1));
//     vel_sub_2_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/velocity_2", 10, std::bind(&JointStatePublisherNode::vel_callback_2, this, std::placeholders::_1));
//     pos_sub_3_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/position_3", 10, std::bind(&JointStatePublisherNode::pos_callback_3, this, std::placeholders::_1));
//     vel_sub_3_ = this->create_subscription<std_msgs::msg::Float32>(
//         "/motor/velocity_3", 10, std::bind(&JointStatePublisherNode::vel_callback_3, this, std::placeholders::_1));

//     // --- Tạo Publisher và Timer cho JointState ---
//     joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
//     joint_state_timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(10), // 100 Hz
//         std::bind(&JointStatePublisherNode::publish_joint_states, this)
//     );

//     // --- Khởi tạo Action Server cho FollowJointTrajectory ---
//     action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
//         this,
//         "rehab_arm_moveit_controller/follow_joint_trajectory", // Tên action server phải khớp với action_ns trong moveit_controllers.yaml
//         std::bind(&JointStatePublisherNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
//         std::bind(&JointStatePublisherNode::handle_cancel, this, std::placeholders::_1),
//         std::bind(&JointStatePublisherNode::handle_accepted, this, std::placeholders::_1));

//     // Loại bỏ subscription và timer cũ không dùng cho action server
//     // trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(...);
//     // trajectory_execution_timer_->cancel();

//     // Bắt đầu đọc encoder
//     odrive_interface_->ReadEncoder();

//     RCLCPP_INFO(this->get_logger(), "JointStatePublisherNode started and ready for 4-DOF robot control via FollowJointTrajectory Action.");
// }

// // --- Callbacks cho encoder ---
// void JointStatePublisherNode::pos_callback_0(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_positions_[odrive_node_to_joint_name_map_[0]] = msg->data;
// }
// void JointStatePublisherNode::vel_callback_0(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_velocities_[odrive_node_to_joint_name_map_[0]] = msg->data;
// }
// void JointStatePublisherNode::pos_callback_1(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_positions_[odrive_node_to_joint_name_map_[1]] = msg->data;
// }
// void JointStatePublisherNode::vel_callback_1(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_velocities_[odrive_node_to_joint_name_map_[1]] = msg->data;
// }
// void JointStatePublisherNode::pos_callback_2(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_positions_[odrive_node_to_joint_name_map_[2]] = msg->data;
// }
// void JointStatePublisherNode::vel_callback_2(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_velocities_[odrive_node_to_joint_name_map_[2]] = msg->data;
// }
// void JointStatePublisherNode::pos_callback_3(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_positions_[odrive_node_to_joint_name_map_[3]] = msg->data;
// }
// void JointStatePublisherNode::vel_callback_3(const std_msgs::msg::Float32::SharedPtr msg) {
//     current_joint_velocities_[odrive_node_to_joint_name_map_[3]] = msg->data;
// }

// // --- Xuất bản JointState ---
// void JointStatePublisherNode::publish_joint_states() {
//     sensor_msgs::msg::JointState joint_state_msg;
//     joint_state_msg.header.stamp = this->now();
//     joint_state_msg.name = joint_names_;

//     for (const auto& name : joint_names_) {
//         joint_state_msg.position.push_back(current_joint_positions_[name]);
//         joint_state_msg.velocity.push_back(current_joint_velocities_[name]);
//     }
//     joint_state_pub_->publish(joint_state_msg);
// }

// // --- Các hàm callback của Action Server ---

// // Xử lý Goal mới
// rclcpp_action::GoalResponse JointStatePublisherNode::handle_goal(
//     const rclcpp_action::GoalUUID & uuid,
//     std::shared_ptr<const FollowJointTrajectory::Goal> goal)
// {
//     RCLCPP_INFO(this->get_logger(), "Received goal request for trajectory with %zu points.", goal->trajectory.points.size());
//     (void)uuid; // Sử dụng uuid để tránh cảnh báo compiler

//     // Kiểm tra xem quỹ đạo có rỗng không
//     if (goal->trajectory.points.empty()) {
//         RCLCPP_WARN(this->get_logger(), "Received empty trajectory goal, rejecting.");
//         return rclcpp_action::GoalResponse::REJECT;
//     }

//     // Kiểm tra số lượng khớp có khớp không
//     if (goal->trajectory.joint_names.size() != joint_names_.size()) {
//         RCLCPP_ERROR(this->get_logger(), "Received trajectory with %zu joints, but expected %zu joints. Rejecting goal.",
//                      goal->trajectory.joint_names.size(), joint_names_.size());
//         return rclcpp_action::GoalResponse::REJECT;
//     }

//     // TODO: Có thể thêm các kiểm tra khác về quỹ đạo (ví dụ: các khớp có khớp với robot không)

//     RCLCPP_INFO(this->get_logger(), "Accepting trajectory goal.");
//     return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER; // Chấp nhận và sẽ xử lý sau trong handle_accepted
// }

// // Xử lý yêu cầu hủy bỏ Goal
// rclcpp_action::CancelResponse JointStatePublisherNode::handle_cancel(
//     const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
// {
//     RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
//     (void)goal_handle; // Sử dụng goal_handle để tránh cảnh báo compiler
//     return rclcpp_action::CancelResponse::ACCEPT; // Luôn chấp nhận yêu cầu hủy bỏ
// }

// // Xử lý khi Goal được chấp nhận
// void JointStatePublisherNode::handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
//     // Bắt đầu thực thi quỹ đạo trong một thread riêng để không chặn luồng chính của node
//     std::thread{std::bind(&JointStatePublisherNode::execute_trajectory_action, this, goal_handle)}.detach();
// }

// // --- Thực thi quỹ đạo Action ---
// void JointStatePublisherNode::execute_trajectory_action(
//     const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
// {
//     RCLCPP_INFO(this->get_logger(), "Executing trajectory goal.");
//     rclcpp::Rate loop_rate(100); // Tốc độ nội suy và gửi lệnh (100 Hz)
//     const auto goal = goal_handle->get_goal();
//     auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
//     auto result = std::make_shared<FollowJointTrajectory::Result>();

//     const auto& trajectory_points = goal->trajectory.points;
//     const auto& joint_names_in_trajectory = goal->trajectory.joint_names;
//     rclcpp::Time trajectory_start_time = this->now();

//     size_t current_point_idx = 0;

//     while (rclcpp::ok() && current_point_idx < trajectory_points.size()) {
//         if (goal_handle->is_canceling()) {
//             goal_handle->canceled(result);
//             std::this_thread::sleep_for(std::chrono::milliseconds(200)); // THÊM DÒNG NÀY
//             RCLCPP_INFO(this->get_logger(), "Trajectory goal canceled.");
//             return;
//         }

//         rclcpp::Duration elapsed_time = this->now() - trajectory_start_time;
//         double current_time_sec = elapsed_time.seconds();

//         size_t prev_point_idx = 0;
//         size_t next_point_idx = 0;

//         // Tìm điểm quỹ đạo hiện tại và tiếp theo để nội suy
//         while (next_point_idx < trajectory_points.size() &&
//                (trajectory_points[next_point_idx].time_from_start.sec +
//                 trajectory_points[next_point_idx].time_from_start.nanosec * 1e-9) < current_time_sec) {
//             prev_point_idx = next_point_idx;
//             next_point_idx++;
//         }

//         // Nếu đã đi qua tất cả các điểm, kết thúc quỹ đạo
//         if (prev_point_idx >= trajectory_points.size() - 1) {
//             const auto& final_point = trajectory_points.back();
//             if (final_point.positions.size() == joint_names_in_trajectory.size()) {
//                 for (size_t i = 0; i < final_point.positions.size(); ++i) {
//                     const std::string& joint_name = joint_names_in_trajectory[i];
//                     double target_pos = final_point.positions[i];
//                     send_command_to_odrive(joint_name, target_pos);
//                 }
//             } else {
//                 RCLCPP_ERROR(this->get_logger(), "Final point position size mismatch with joint names in trajectory. Setting goal as ABORTED.");
//                 result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
//                 goal_handle->abort(result);
//                 std::this_thread::sleep_for(std::chrono::milliseconds(200)); // THÊM DÒNG NÀY
//                 return;
//             }
//             RCLCPP_INFO(this->get_logger(), "Trajectory finished. Sent final point commands. Setting goal as SUCCEEDED.");
//             result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
//             goal_handle->succeed(result);
//             std::this_thread::sleep_for(std::chrono::milliseconds(200)); // THÊM DÒNG NÀY
//             return;
//         }

//         // Nội suy vị trí
//         const auto& p_prev = trajectory_points[prev_point_idx];
//         const auto& p_next = trajectory_points[next_point_idx];

//         double t_prev = p_prev.time_from_start.sec + p_prev.time_from_start.nanosec * 1e-9;
//         double t_next = p_next.time_from_start.sec + p_next.time_from_start.nanosec * 1e-9;

//         double fraction = 0.0;
//         if ((t_next - t_prev) > 1e-9) { // Tránh chia cho 0
//             fraction = (current_time_sec - t_prev) / (t_next - t_prev);
//         }

//         if (p_prev.positions.size() == joint_names_in_trajectory.size() &&
//             p_next.positions.size() == joint_names_in_trajectory.size()) {
//             for (size_t i = 0; i < p_prev.positions.size(); ++i) {
//                 const std::string& joint_name = joint_names_in_trajectory[i];
//                 double target_pos = p_prev.positions[i] + fraction * (p_next.positions[i] - p_prev.positions[i]);
//                 send_command_to_odrive(joint_name, target_pos);
//             }
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Trajectory point position size mismatch with joint names in trajectory during interpolation. Setting goal as ABORTED.");
//             result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
//             goal_handle->abort(result);
//             std::this_thread::sleep_for(std::chrono::milliseconds(200)); // THÊM DÒNG NÀY
//             return;
//         }

//         // Cập nhật và xuất bản feedback (có thể bỏ qua nếu không cần thiết quá chi tiết)
//         feedback->header.stamp = this->now();
//         feedback->joint_names = joint_names_in_trajectory;
//         feedback->actual.positions = {}; // Bạn có thể điền vào vị trí thực tế của robot nếu có
//         feedback->desired.positions = {}; // Vị trí mong muốn từ quỹ đạo
//         feedback->error.positions = {}; // Sai số giữa thực tế và mong muốn
//         // Để điền các trường này, bạn cần truy cập current_joint_positions_ và tính toán
//         // Ví dụ: feedback->actual.positions.push_back(current_joint_positions_[joint_name]);
//         goal_handle->publish_feedback(feedback);

//         loop_rate.sleep();
//     }
// }


// // --- Gửi lệnh tới ODrive ---
// void JointStatePublisherNode::send_command_to_odrive(const std::string& joint_name, double position) {
//     int odrive_node_id = -1;
//     for (const auto& [node_id, name] : odrive_node_to_joint_name_map_) {
//         if (name == joint_name) {
//             odrive_node_id = node_id;
//             break;
//         }
//     }

//     if (odrive_node_id != -1) {
//         float pos_float = static_cast<float>(position);
//         odrive_interface_->SendPositionCommand(odrive_node_id, pos_float);
//         // RCLCPP_INFO(this->get_logger(), "Commanding ODrive Node %d (%s): Pos=%.3f rad", // Log này có thể quá nhiều
//         //             odrive_node_id, joint_name.c_str(), position);
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "Unknown joint name: %s. Cannot send command to ODrive.", joint_name.c_str());
//     }
// }

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<JointStatePublisherNode>());
//     rclcpp::shutdown();
//     return 0;
// }






#include "odrive_joint_state_publisher/joint_state_publisher_node.hpp"

// Các includes cần thiết cho action server
#include <thread>
#include <chrono>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>


JointStatePublisherNode::JointStatePublisherNode() : Node("odrive_joint_state_publisher_node") {
    // --- Ánh xạ ODrive Node ID tới Tên khớp ---
    odrive_node_to_joint_name_map_[0] = "joint_0";
    odrive_node_to_joint_name_map_[1] = "joint_1";
    odrive_node_to_joint_name_map_[2] = "joint_2";
    odrive_node_to_joint_name_map_[3] = "joint_3";

    // --- Cấu hình các tên khớp ---
    joint_names_.push_back("joint_0");
    joint_names_.push_back("joint_1");
    joint_names_.push_back("joint_2");
    joint_names_.push_back("joint_3");

    // Khởi tạo trạng thái khớp ban đầu
    for (const auto& name : joint_names_) {
        current_joint_positions_[name] = 0.0;
        current_joint_velocities_[name] = 0.0;
    }

    // --- Khởi tạo ODriveInterface ---
    odrive_interface_ = std::make_shared<can_odrive_interface::ODriveInterface>();

    // Khởi tạo chế độ position cho các node ODrive
    // Chú ý: vòng lặp ban đầu của bạn chỉ chạy cho node_id = 1. Hãy kiểm tra lại nếu bạn có nhiều ODrive
    for (int node_id = 0; node_id <= 3; ++node_id) { // Thay đổi để bao gồm tất cả các node ODrive có thể
        odrive_interface_->InitPositionMode(node_id);
    }

    // --- Tạo Subscriptions cho encoder ---
    pos_sub_0_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor/position_0", 10, std::bind(&JointStatePublisherNode::pos_callback_0, this, std::placeholders::_1));
    vel_sub_0_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor/velocity_0", 10, std::bind(&JointStatePublisherNode::vel_callback_0, this, std::placeholders::_1));
    pos_sub_1_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor/position_1", 10, std::bind(&JointStatePublisherNode::pos_callback_1, this, std::placeholders::_1));
    vel_sub_1_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor/velocity_1", 10, std::bind(&JointStatePublisherNode::vel_callback_1, this, std::placeholders::_1));
    pos_sub_2_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor/position_2", 10, std::bind(&JointStatePublisherNode::pos_callback_2, this, std::placeholders::_1));
    vel_sub_2_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor/velocity_2", 10, std::bind(&JointStatePublisherNode::vel_callback_2, this, std::placeholders::_1));
    pos_sub_3_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor/position_3", 10, std::bind(&JointStatePublisherNode::pos_callback_3, this, std::placeholders::_1));
    vel_sub_3_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor/velocity_3", 10, std::bind(&JointStatePublisherNode::vel_callback_3, this, std::placeholders::_1));

    // --- Tạo Publisher và Timer cho JointState ---
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    joint_state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // 100 Hz
        std::bind(&JointStatePublisherNode::publish_joint_states, this)
    );

    // --- Khởi tạo Action Server cho FollowJointTrajectory ---
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this,
        "rehab_arm_moveit_controller/follow_joint_trajectory", // Tên action server phải khớp với action_ns trong moveit_controllers.yaml
        std::bind(&JointStatePublisherNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointStatePublisherNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&JointStatePublisherNode::handle_accepted, this, std::placeholders::_1));

    // Loại bỏ subscription và timer cũ không dùng cho action server
    // trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(...);
    // trajectory_execution_timer_->cancel();

    // Bắt đầu đọc encoder
    odrive_interface_->ReadEncoder();

    RCLCPP_INFO(this->get_logger(), "JointStatePublisherNode started and ready for 4-DOF robot control via FollowJointTrajectory Action.");
}

// --- Callbacks cho encoder ---
void JointStatePublisherNode::pos_callback_0(const std_msgs::msg::Float32::SharedPtr msg) {
    current_joint_positions_[odrive_node_to_joint_name_map_[0]] = msg->data;
}
void JointStatePublisherNode::vel_callback_0(const std_msgs::msg::Float32::SharedPtr msg) {
    current_joint_velocities_[odrive_node_to_joint_name_map_[0]] = msg->data;
}
void JointStatePublisherNode::pos_callback_1(const std_msgs::msg::Float32::SharedPtr msg) {
    current_joint_positions_[odrive_node_to_joint_name_map_[1]] = msg->data;
}
void JointStatePublisherNode::vel_callback_1(const std_msgs::msg::Float32::SharedPtr msg) {
    current_joint_velocities_[odrive_node_to_joint_name_map_[1]] = msg->data;
}
void JointStatePublisherNode::pos_callback_2(const std_msgs::msg::Float32::SharedPtr msg) {
    current_joint_positions_[odrive_node_to_joint_name_map_[2]] = msg->data;
}
void JointStatePublisherNode::vel_callback_2(const std_msgs::msg::Float32::SharedPtr msg) {
    current_joint_velocities_[odrive_node_to_joint_name_map_[2]] = msg->data;
}
void JointStatePublisherNode::pos_callback_3(const std_msgs::msg::Float32::SharedPtr msg) {
    current_joint_positions_[odrive_node_to_joint_name_map_[3]] = msg->data;
}
void JointStatePublisherNode::vel_callback_3(const std_msgs::msg::Float32::SharedPtr msg) {
    current_joint_velocities_[odrive_node_to_joint_name_map_[3]] = msg->data;
}

// --- Xuất bản JointState ---
void JointStatePublisherNode::publish_joint_states() {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = joint_names_;

    for (const auto& name : joint_names_) {
        joint_state_msg.position.push_back(current_joint_positions_[name]);
        joint_state_msg.velocity.push_back(current_joint_velocities_[name]);
    }
    joint_state_pub_->publish(joint_state_msg);
}

// --- Các hàm callback của Action Server ---

// Xử lý Goal mới
rclcpp_action::GoalResponse JointStatePublisherNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request for trajectory with %zu points.", goal->trajectory.points.size());
    (void)uuid; // Sử dụng uuid để tránh cảnh báo compiler

    // Kiểm tra xem quỹ đạo có rỗng không
    if (goal->trajectory.points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty trajectory goal, rejecting.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Kiểm tra số lượng khớp có khớp không
    if (goal->trajectory.joint_names.size() != joint_names_.size()) {
        RCLCPP_ERROR(this->get_logger(), "Received trajectory with %zu joints, but expected %zu joints. Rejecting goal.",
                     goal->trajectory.joint_names.size(), joint_names_.size());
        return rclcpp_action::GoalResponse::REJECT;
    }

    // TODO: Có thể thêm các kiểm tra khác về quỹ đạo (ví dụ: các khớp có khớp với robot không)

    RCLCPP_INFO(this->get_logger(), "Accepting trajectory goal.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Thay đổi từ ACCEPT_AND_DEFER
}

// Xử lý yêu cầu hủy bỏ Goal
rclcpp_action::CancelResponse JointStatePublisherNode::handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
    (void)goal_handle; // Sử dụng goal_handle để tránh cảnh báo compiler
    return rclcpp_action::CancelResponse::ACCEPT; // Luôn chấp nhận yêu cầu hủy bỏ
}

// Xử lý khi Goal được chấp nhận
void JointStatePublisherNode::handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
    // Bắt đầu thực thi quỹ đạo trong một thread riêng để không chặn luồng chính của node
    std::thread{std::bind(&JointStatePublisherNode::execute_trajectory_action, this, goal_handle)}.detach();
}

// --- Thực thi quỹ đạo Action ---
void JointStatePublisherNode::execute_trajectory_action(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing trajectory goal.");
    rclcpp::Rate loop_rate(100); // Tốc độ nội suy và gửi lệnh (100 Hz)
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    const auto& trajectory_points = goal->trajectory.points;
    const auto& joint_names_in_trajectory = goal->trajectory.joint_names;
    rclcpp::Time trajectory_start_time = this->now();

    size_t current_point_idx = 0;
    bool goal_completed = false;

    while (rclcpp::ok() && current_point_idx < trajectory_points.size() && !goal_completed) {
        // Kiểm tra trạng thái goal handle trước khi xử lý
        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Trajectory goal canceled.");
            // result->error_code = FollowJointTrajectory::Result::PREEMPTED;
            goal_handle->canceled(result);
            return;
        }

        rclcpp::Duration elapsed_time = this->now() - trajectory_start_time;
        double current_time_sec = elapsed_time.seconds();

        size_t prev_point_idx = 0;
        size_t next_point_idx = 0;

        // Tìm điểm quỹ đạo hiện tại và tiếp theo để nội suy
        while (next_point_idx < trajectory_points.size() &&
               (trajectory_points[next_point_idx].time_from_start.sec +
                trajectory_points[next_point_idx].time_from_start.nanosec * 1e-9) < current_time_sec) {
            prev_point_idx = next_point_idx;
            next_point_idx++;
        }

        // Nếu đã đi qua tất cả các điểm, kết thúc quỹ đạo
        if (prev_point_idx >= trajectory_points.size() - 1) {
            const auto& final_point = trajectory_points.back();
            if (final_point.positions.size() == joint_names_in_trajectory.size()) {
                for (size_t i = 0; i < final_point.positions.size(); ++i) {
                    const std::string& joint_name = joint_names_in_trajectory[i];
                    double target_pos = final_point.positions[i];
                    send_command_to_odrive(joint_name, target_pos);
                }
                
                // Đặt flag để thoát khỏi vòng lặp
                goal_completed = true;
                
                // Kiểm tra lại trạng thái trước khi gọi succeed
                if (!goal_handle->is_canceling()) {
                    RCLCPP_INFO(this->get_logger(), "Trajectory finished. Sent final point commands. Setting goal as SUCCEEDED.");
                    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                    goal_handle->succeed(result);
                }
                return;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Final point position size mismatch with joint names in trajectory. Setting goal as ABORTED.");
                result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
                goal_handle->abort(result);
                return;
            }
        }

        // Nội suy vị trí
        const auto& p_prev = trajectory_points[prev_point_idx];
        const auto& p_next = trajectory_points[next_point_idx];

        double t_prev = p_prev.time_from_start.sec + p_prev.time_from_start.nanosec * 1e-9;
        double t_next = p_next.time_from_start.sec + p_next.time_from_start.nanosec * 1e-9;

        double fraction = 0.0;
        if ((t_next - t_prev) > 1e-9) { // Tránh chia cho 0
            fraction = (current_time_sec - t_prev) / (t_next - t_prev);
        }

        if (p_prev.positions.size() == joint_names_in_trajectory.size() &&
            p_next.positions.size() == joint_names_in_trajectory.size()) {
            for (size_t i = 0; i < p_prev.positions.size(); ++i) {
                const std::string& joint_name = joint_names_in_trajectory[i];
                double target_pos = p_prev.positions[i] + fraction * (p_next.positions[i] - p_prev.positions[i]);
                send_command_to_odrive(joint_name, target_pos);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Trajectory point position size mismatch with joint names in trajectory during interpolation. Setting goal as ABORTED.");
            result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
            goal_handle->abort(result);
            return;
        }

        // Cập nhật và xuất bản feedback
        feedback->header.stamp = this->now();
        feedback->joint_names = joint_names_in_trajectory;
        
        // Điền thông tin feedback với dữ liệu thực tế
        feedback->actual.positions.clear();
        feedback->desired.positions.clear();
        feedback->error.positions.clear();
        
        for (size_t i = 0; i < joint_names_in_trajectory.size(); ++i) {
            const std::string& joint_name = joint_names_in_trajectory[i];
            double desired_pos = p_prev.positions[i] + fraction * (p_next.positions[i] - p_prev.positions[i]);
            double actual_pos = current_joint_positions_[joint_name];
            double error_pos = desired_pos - actual_pos;
            
            feedback->actual.positions.push_back(actual_pos);
            feedback->desired.positions.push_back(desired_pos);
            feedback->error.positions.push_back(error_pos);
        }
        
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }
}

// --- Gửi lệnh tới ODrive ---
void JointStatePublisherNode::send_command_to_odrive(const std::string& joint_name, double position) {
    int odrive_node_id = -1;
    for (const auto& [node_id, name] : odrive_node_to_joint_name_map_) {
        if (name == joint_name) {
            odrive_node_id = node_id;
            break;
        }
    }

    if (odrive_node_id != -1) {
        float pos_float = static_cast<float>(position);
        odrive_interface_->SendPositionCommand(odrive_node_id, pos_float);
        // RCLCPP_INFO(this->get_logger(), "Commanding ODrive Node %d (%s): Pos=%.3f rad", // Log này có thể quá nhiều
        //             odrive_node_id, joint_name.c_str(), position);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown joint name: %s. Cannot send command to ODrive.", joint_name.c_str());
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisherNode>());
    rclcpp::shutdown();
    return 0;
}