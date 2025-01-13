#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
 
using std::placeholders::_1;
using std::placeholders::_2;
 
class Nav2Sim : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;
 
  Nav2Sim()
  : Node("navigate_to_pose_server")
  {
    // Subscribe to the /odom topic to get the robot's real-time position
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Nav2Sim::update_current_position, this, _1));
  }
 
  void start_server()
  {
    using namespace std::placeholders;
 
    repeat_sentence_action_server_ = rclcpp_action::create_server<NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose",
      std::bind(&Nav2Sim::handle_goal, this, _1, _2),
      std::bind(&Nav2Sim::handle_cancel, this, _1),
      std::bind(&Nav2Sim::handle_accepted, this, _1));
 
    RCLCPP_INFO(get_logger(), "Ready.");
  }
 
private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr repeat_sentence_action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
 
  double current_x_ = 0.0;
  double current_y_ = 0.0;
 
  void update_current_position(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
 
    //RCLCPP_INFO(this->get_logger(), "Updated Current Position: [%f, %f]", current_x_, current_y_);
  }
 
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
 
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
 
  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();
 
    auto pose_cmd = goal_handle->get_goal()->pose.pose;
    tf2::Quaternion q;
    tf2::fromMsg(pose_cmd.orientation, q);
 
    double target_x = pose_cmd.position.x;
    double target_y = pose_cmd.position.y;
 
    RCLCPP_INFO(
      this->get_logger(), "Starting navigation to [%lf, %lf, %lf]",
      target_x, target_y, q.getAngle());
 
    while (rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Action Canceled");
        return;
      }
 
      // Calculate distance to the target using current position from /odom
      feedback->distance_remaining = std::sqrt(
        std::pow(target_x - current_x_, 2) +
        std::pow(target_y - current_y_, 2)
      );
 
      RCLCPP_INFO(this->get_logger(), "Current distance to goal: %f", feedback->distance_remaining);
 
      // Check if the robot has reached the goal (threshold: 0.1 meters)
      if (feedback->distance_remaining <= 0.299)
      {
        RCLCPP_INFO(this->get_logger(), "Goal Reached");
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Navigation Succeeded");
        return;
      }
 
      loop_rate.sleep();
    }
  }
 
  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&Nav2Sim::execute, this, _1), goal_handle}.detach();
  }
};
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
 
  auto node = std::make_shared<Nav2Sim>();
 
  node->start_server();
 
  rclcpp::spin(node);
 
  rclcpp::shutdown();
 
  return 0;
}
