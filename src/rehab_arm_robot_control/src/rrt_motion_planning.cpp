#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

// RRT Node structure
struct RRTNode {
    std::vector<double> joint_angles;
    int parent_id;
    double cost;
    
    RRTNode(const std::vector<double>& angles, int parent = -1, double c = 0.0) 
        : joint_angles(angles), parent_id(parent), cost(c) {}
};

// Define constants - Phù hợp với rehab arm 4 DOF
const std::vector<std::string> arm_joints = {
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3"
};

// Joint limits cho 4 khớp (radians)
const std::vector<std::pair<double, double>> joint_limits = {
    {0.0, 1.57},   // joint_0
    {-1.57, 1.57},   // joint_1
    {-3.14, 3.14},   // joint_2
    {-1.57, 1.57}    // joint_3
};

class RehabArmRRTPlanner : public rclcpp::Node
{
public:
    RehabArmRRTPlanner()
        : Node("rehab_arm_rrt_planner"), rng_(std::random_device{}())
    {
        RCLCPP_INFO(this->get_logger(), "🚀 Rehab Arm RRT Planner Started!");
        RCLCPP_INFO(this->get_logger(), "Publishing to /rehab_arm_moveit_controller/joint_trajectory");
        
        // Publisher - Same as original code
        arm_pose_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/rehab_arm_moveit_controller/joint_trajectory", 1);
        
        // Subscriber for current joint states
        joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RehabArmRRTPlanner::jointStateCallback, this, std::placeholders::_1));
        
        // Timer - Same interval as original (5s)
        timer_ = create_wall_timer(5s, 
            std::bind(&RehabArmRRTPlanner::planningTimerCallback, this));
        
        // Frame ID - Same as original
        frame_id_ = "base_link";
        
        // Duration settings - Same as original
        duration_sec_ = 2;
        duration_nanosec_ = 0.5 * 1e9;
        
        // RRT Parameters
        max_iterations_ = 1000;
        step_size_ = 0.2;
        goal_tolerance_ = 0.1;
        goal_bias_ = 0.1;  // 10% chance to sample goal directly
        
        // Initialize current joint state (4 DOF)
        current_joint_state_.resize(4, 0.0);
        joint_state_received_ = false;
        
        // Goal positions - Same as original but with RRT planning
        arm_positions_ = {
            {0.0, 0.0, 0.0, 0.0},        // Home location
            {0.0, 0.0, -1.56, -1.56},    // UP_45
            {0.0, -1.56, 0.0, -1.56},    // Custom position
            {0.0, 0.5, -0.5, -1.0}       // Additional test position
        };
        
        // Parameters - Same as original
        this->declare_parameter<int>("state_cmd", 0);
        this->declare_parameter<int>("max_iterations", max_iterations_);
        this->declare_parameter<double>("step_size", step_size_);
        this->declare_parameter<double>("goal_tolerance", goal_tolerance_);
        this->declare_parameter<bool>("use_rrt_planning", true);
        
        // Statistics
        planning_attempts_ = 0;
        successful_plans_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Available states: 0:Home, 1:UP_45, 2:Custom, 3:Test");
        RCLCPP_INFO(this->get_logger(), "✅ RRT Planner initialized for 4-DOF Rehab Arm");
        RCLCPP_INFO(this->get_logger(), "Waiting for joint state data from /joint_states...");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!joint_state_received_) {
            RCLCPP_INFO(this->get_logger(), "✅ Joint state data received!");
            joint_state_received_ = true;
        }
        
        // Parse joint states for 4 DOF arm
        if (msg->name.size() >= 4 && msg->position.size() >= 4) {
            // Find correct joint indices
            for (size_t i = 0; i < arm_joints.size(); ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), arm_joints[i]);
                if (it != msg->name.end()) {
                    size_t index = std::distance(msg->name.begin(), it);
                    if (index < msg->position.size()) {
                        current_joint_state_[i] = msg->position[index];
                    }
                }
            }
        } else if (msg->position.size() >= 4) {
            // Fallback: assume first 4 positions
            for (size_t i = 0; i < 4; ++i) {
                current_joint_state_[i] = msg->position[i];
            }
        }
    }
    
    void planningTimerCallback()
    {
        // Get state command - Same as original
        int state_cmd = this->get_parameter("state_cmd").as_int();
        bool use_rrt = this->get_parameter("use_rrt_planning").as_bool();
        
        if (state_cmd < 0 || state_cmd >= static_cast<int>(arm_positions_.size())) {
            RCLCPP_WARN(this->get_logger(), "state_cmd (%d) out of range, using 0", state_cmd);
            state_cmd = 0;
        }
        
        auto goal_position = arm_positions_[state_cmd];
        
        // Log with 4 joints - Fixed format string
        RCLCPP_INFO(this->get_logger(), "Planning state_cmd: %d -> [%.2f, %.2f, %.2f, %.2f]", 
                   state_cmd, goal_position[0], goal_position[1], goal_position[2], goal_position[3]);
        
        if (!joint_state_received_) {
            RCLCPP_WARN(this->get_logger(), "⚠️  No joint state received, using default start position");
        }
        
        std::vector<std::vector<double>> trajectory;
        
        if (use_rrt) {
            // Use RRT planning
            planning_attempts_++;
            
            // Check if already at goal
            if (distance(current_joint_state_, goal_position) < goal_tolerance_) {
                RCLCPP_INFO(this->get_logger(), "✅ Already at goal position!");
                trajectory = {current_joint_state_, goal_position};
            } else {
                RCLCPP_INFO(this->get_logger(), "🔄 Planning with RRT from [%.2f, %.2f, %.2f, %.2f]", 
                           current_joint_state_[0], current_joint_state_[1], 
                           current_joint_state_[2], current_joint_state_[3]);
                
                auto start_time = std::chrono::high_resolution_clock::now();
                trajectory = planRRT(current_joint_state_, goal_position);
                auto end_time = std::chrono::high_resolution_clock::now();
                
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                
                if (!trajectory.empty()) {
                    successful_plans_++;
                    RCLCPP_INFO(this->get_logger(), "✅ RRT Success in %ld ms! Path: %zu waypoints", 
                               duration.count(), trajectory.size());
                    
                    // Smooth the trajectory
                    trajectory = smoothPath(trajectory);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ RRT Failed after %ld ms", duration.count());
                    // Fallback to direct path
                    trajectory = {current_joint_state_, goal_position};
                }
                
                double success_rate = (double)successful_plans_ / planning_attempts_ * 100.0;
                RCLCPP_INFO(this->get_logger(), "📊 RRT Success rate: %.1f%% (%d/%d)", 
                           success_rate, successful_plans_, planning_attempts_);
            }
        } else {
            // Direct movement - Same as original
            RCLCPP_INFO(this->get_logger(), "➡️  Direct movement (RRT disabled)");
            trajectory = {goal_position};
        }
        
        // Publish trajectory
        publishTrajectory(trajectory);
    }
    
    std::vector<std::vector<double>> planRRT(const std::vector<double>& start, 
                                           const std::vector<double>& goal)
    {
        // Update parameters
        max_iterations_ = this->get_parameter("max_iterations").as_int();
        step_size_ = this->get_parameter("step_size").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        
        std::vector<RRTNode> tree;
        tree.emplace_back(start, -1, 0.0);
        
        for (int iter = 0; iter < max_iterations_; ++iter) {
            std::vector<double> q_rand;
            
            // Goal biasing
            std::uniform_real_distribution<double> bias_dist(0.0, 1.0);
            if (bias_dist(rng_) < goal_bias_) {
                q_rand = goal;
            } else {
                q_rand = sampleRandomConfiguration();
            }
            
            // Find nearest node
            int nearest_id = findNearestNode(tree, q_rand);
            
            // Extend towards random sample
            auto q_new = extend(tree[nearest_id].joint_angles, q_rand);
            
            // Check if valid configuration
            if (isValidConfiguration(q_new)) {
                double cost = tree[nearest_id].cost + distance(tree[nearest_id].joint_angles, q_new);
                tree.emplace_back(q_new, nearest_id, cost);
                
                // Check if goal reached
                if (distance(q_new, goal) < goal_tolerance_) {
                    RCLCPP_DEBUG(this->get_logger(), "Goal reached in %d iterations!", iter + 1);
                    return extractPath(tree, tree.size() - 1);
                }
            }
        }
        
        return {}; // Failed to find path
    }
    
    std::vector<double> sampleRandomConfiguration()
    {
        std::vector<double> config(4);
        for (size_t i = 0; i < 4; ++i) {
            std::uniform_real_distribution<double> dist(joint_limits[i].first, joint_limits[i].second);
            config[i] = dist(rng_);
        }
        return config;
    }
    
    int findNearestNode(const std::vector<RRTNode>& tree, const std::vector<double>& q)
    {
        int nearest = 0;
        double min_dist = distance(tree[0].joint_angles, q);
        
        for (size_t i = 1; i < tree.size(); ++i) {
            double dist = distance(tree[i].joint_angles, q);
            if (dist < min_dist) {
                min_dist = dist;
                nearest = i;
            }
        }
        return nearest;
    }
    
    std::vector<double> extend(const std::vector<double>& q_near, const std::vector<double>& q_rand)
    {
        std::vector<double> direction(4);
        double norm = 0.0;
        
        for (size_t i = 0; i < 4; ++i) {
            direction[i] = q_rand[i] - q_near[i];
            norm += direction[i] * direction[i];
        }
        norm = sqrt(norm);
        
        std::vector<double> q_new(4);
        for (size_t i = 0; i < 4; ++i) {
            if (norm > step_size_) {
                q_new[i] = q_near[i] + (direction[i] / norm) * step_size_;
            } else {
                q_new[i] = q_rand[i];
            }
            
            // Clamp to joint limits
            q_new[i] = std::clamp(q_new[i], joint_limits[i].first, joint_limits[i].second);
        }
        return q_new;
    }
    
    bool isValidConfiguration(const std::vector<double>& config)
    {
        // Check joint limits
        for (size_t i = 0; i < 4; ++i) {
            if (config[i] < joint_limits[i].first || config[i] > joint_limits[i].second) {
                return false;
            }
        }
        
        // Add more sophisticated collision checking here if needed
        // For now, just check joint limits
        
        return true;
    }
    
    double distance(const std::vector<double>& q1, const std::vector<double>& q2)
    {
        double dist = 0.0;
        for (size_t i = 0; i < q1.size(); ++i) {
            double diff = q1[i] - q2[i];
            dist += diff * diff;
        }
        return sqrt(dist);
    }
    
    std::vector<std::vector<double>> extractPath(const std::vector<RRTNode>& tree, int goal_id)
    {
        std::vector<std::vector<double>> path;
        int current = goal_id;
        
        while (current != -1) {
            path.push_back(tree[current].joint_angles);
            current = tree[current].parent_id;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    std::vector<std::vector<double>> smoothPath(const std::vector<std::vector<double>>& path)
    {
        if (path.size() <= 2) {
            return path;
        }
        
        // Simple path smoothing by removing redundant waypoints
        std::vector<std::vector<double>> smoothed;
        smoothed.push_back(path[0]);
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            // Add intermediate waypoints for better motion
            if (distance(smoothed.back(), path[i]) > step_size_) {
                smoothed.push_back(path[i]);
            }
        }
        
        smoothed.push_back(path.back());
        return smoothed;
    }
    
    void publishTrajectory(const std::vector<std::vector<double>>& trajectory)
    {
        // Create message - Same structure as original
        auto msg_arm = trajectory_msgs::msg::JointTrajectory();
        msg_arm.header.frame_id = frame_id_;
        msg_arm.header.stamp = this->now();
        msg_arm.joint_names = arm_joints;
        
        if (trajectory.size() == 1) {
            // Single point trajectory (same as original)
            auto point_arm = trajectory_msgs::msg::JointTrajectoryPoint();
            point_arm.positions = trajectory[0];
            point_arm.time_from_start = rclcpp::Duration(duration_sec_, duration_nanosec_);
            msg_arm.points.push_back(point_arm);
        } else {
            // Multi-point trajectory from RRT
            double time_step = (double)duration_sec_ / std::max(1.0, (double)(trajectory.size() - 1));
            
            for (size_t i = 0; i < trajectory.size(); ++i) {
                auto point_arm = trajectory_msgs::msg::JointTrajectoryPoint();
                point_arm.positions = trajectory[i];
                
                if (i == 0) {
                    point_arm.time_from_start = rclcpp::Duration::from_seconds(0.1);
                } else {
                    point_arm.time_from_start = rclcpp::Duration::from_seconds(i * time_step);
                }
                
                msg_arm.points.push_back(point_arm);
            }
        }
        
        // Publish - Same as original
        arm_pose_publisher_->publish(msg_arm);
        
        RCLCPP_INFO(this->get_logger(), "📤 Published trajectory with %zu waypoints", trajectory.size());
    }
    
    // Member variables
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pose_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Same as original
    std::string frame_id_;
    int duration_sec_;
    int duration_nanosec_;
    std::vector<std::vector<double>> arm_positions_;
    
    // RRT specific
    std::vector<double> current_joint_state_;
    bool joint_state_received_;
    
    // RRT parameters
    int max_iterations_;
    double step_size_;
    double goal_tolerance_;
    double goal_bias_;
    
    // Statistics
    int planning_attempts_;
    int successful_plans_;
    
    std::mt19937 rng_;
};

int main(int argc, char * argv[])
{
    // Same as original
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RehabArmRRTPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


