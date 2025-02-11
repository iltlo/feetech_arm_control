#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

class EEGoalControlNode : public rclcpp::Node
{
public:
  EEGoalControlNode() : Node("ee_goal_control_node")
  {
    // Set a callback for the goal pose topic
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "ee_goal_pose", 10, std::bind(&EEGoalControlNode::goalPoseCallback, this, std::placeholders::_1));
    
    // Create publishers for planning and execution states
    planning_state_pub_ = this->create_publisher<std_msgs::msg::String>("planning_state", 10);
    execution_state_pub_ = this->create_publisher<std_msgs::msg::String>("execution_state", 10);
  }

  void initializeMoveGroup()
  {
    // Use rclcpp::Node's shared_from_this() directly
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    // Explicitly set the end-effector link (change "link_6" to your actual EE link)
    move_group_->setEndEffectorLink("Link_6"); 
  }

private:
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!move_group_)
    {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface is not initialized!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received new goal pose");

    // Set the start state to the current state
    move_group_->setStartStateToCurrentState();

    // Set the target pose for the end-effector
    move_group_->setPoseTarget(*msg);

    // Publish planning state
    auto planning_state_msg = std_msgs::msg::String();
    planning_state_msg.data = "In progress";
    planning_state_pub_->publish(planning_state_msg);

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing the plan");
      planning_state_msg.data = "Successful";
      planning_state_pub_->publish(planning_state_msg);

      auto execution_state_msg = std_msgs::msg::String();
      execution_state_msg.data = "In progress";
      execution_state_pub_->publish(execution_state_msg);

      move_group_->execute(plan);

      execution_state_msg.data = "Completed";
      execution_state_pub_->publish(execution_state_msg);

      // Update the goal state in RViz
      move_group_->setStartStateToCurrentState();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Planning failed");
      planning_state_msg.data = "Failed";
      planning_state_pub_->publish(planning_state_msg);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_state_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EEGoalControlNode>();

  // Initialize MoveGroupInterface after the node is fully constructed
  node->initializeMoveGroup();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}