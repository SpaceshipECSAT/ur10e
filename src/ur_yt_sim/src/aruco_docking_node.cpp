#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
tf2::Transform poseMsgToTf(const geometry_msgs::msg::Pose &pose)
{
  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  q.normalize();
  tf.setRotation(q);
  return tf;
}

geometry_msgs::msg::Pose tfToPoseMsg(const tf2::Transform &tf)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  pose.orientation = tf2::toMsg(tf.getRotation());
  return pose;
}
}  // namespace

class ArucoDockingNode : public rclcpp::Node
{
public:
  ArucoDockingNode()
  : Node("aruco_docking_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    move_group_(std::shared_ptr<rclcpp::Node>(this, [](auto *){}),
                this->declare_parameter<std::string>("move_group", "ur10_manipulator"))
  {
    marker_topic_ = this->declare_parameter<std::string>("marker_pose_topic", "aruco/pose");
    reference_frame_ = this->declare_parameter<std::string>("reference_frame", "base_link");
    end_effector_link_ = this->declare_parameter<std::string>("end_effector_link", "wrist_3_link");
    approach_distance_ = this->declare_parameter<double>("approach_distance", 0.25);
    cartesian_step_ = this->declare_parameter<double>("cartesian_step", 0.01);
    cartesian_fraction_threshold_ = this->declare_parameter<double>("cartesian_fraction_threshold", 0.9);
    move_group_.setPoseReferenceFrame(reference_frame_);
    move_group_.setEndEffectorLink(end_effector_link_);
    move_group_.setMaxVelocityScalingFactor(
      this->declare_parameter<double>("max_velocity_scale", 0.2));
    move_group_.setMaxAccelerationScalingFactor(
      this->declare_parameter<double>("max_acceleration_scale", 0.2));
    move_group_.setPlanningTime(this->declare_parameter<double>("planning_time", 10.0));
    goal_position_tolerance_ =
      this->declare_parameter<double>("goal_position_tolerance", 0.01);
    goal_orientation_tolerance_ =
      this->declare_parameter<double>("goal_orientation_tolerance", 0.05);
    move_group_.setGoalPositionTolerance(goal_position_tolerance_);
    move_group_.setGoalOrientationTolerance(goal_orientation_tolerance_);
    move_group_.allowReplanning(true);
    target_marker_id_ = this->declare_parameter<int>("target_marker_id", 0);
    marker_frame_prefix_ = this->declare_parameter<std::string>("marker_frame_prefix", "aruco_marker_");

    const auto translation_default = std::vector<double>{0.0, 0.0, 0.0};
    const auto rpy_default = std::vector<double>{M_PI, 0.0, 0.0};
    marker_to_tool_translation_ = this->declare_parameter("tool_from_marker_translation", translation_default);
    marker_to_tool_rpy_ = this->declare_parameter("tool_from_marker_rpy", rpy_default);
    if (marker_to_tool_translation_.size() != 3 || marker_to_tool_rpy_.size() != 3)
    {
      throw std::runtime_error("tool_from_marker_* parameters must contain exactly 3 elements");
    }

    marker_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      marker_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ArucoDockingNode::markerCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Aruco docking node ready. Waiting for poses on '%s' to dock in '%s' frame.",
      marker_topic_.c_str(), reference_frame_.c_str());
  }

private:
  void markerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (docking_started_)
    {
      return;
    }

    geometry_msgs::msg::PoseStamped marker_in_reference;
    if (target_marker_id_ >= 0)
    {
      const auto marker_frame = marker_frame_prefix_ + std::to_string(target_marker_id_);
      try
      {
        const auto transform = tf_buffer_.lookupTransform(
          reference_frame_, marker_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
        marker_in_reference.header.stamp = transform.header.stamp;
        marker_in_reference.header.frame_id = reference_frame_;
        marker_in_reference.pose.position.x = transform.transform.translation.x;
        marker_in_reference.pose.position.y = transform.transform.translation.y;
        marker_in_reference.pose.position.z = transform.transform.translation.z;
        marker_in_reference.pose.orientation = transform.transform.rotation;
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Target marker frame %s unavailable yet: %s", marker_frame.c_str(), ex.what());
        return;
      }
    }
    else
    {
      try
      {
        marker_in_reference = tf_buffer_.transform(*msg, reference_frame_, tf2::durationFromSec(0.1));
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Transform to %s unavailable yet: %s", reference_frame_.c_str(), ex.what());
        return;
      }
    }

    const auto dock_pose = computeDockPose(marker_in_reference.pose);
    const auto pre_dock_pose = computePreDockPose(dock_pose);
    docking_started_ = true;

    RCLCPP_INFO(this->get_logger(), "Received marker pose. Starting docking sequence.");

    std::thread(
      [this, pre_dock_pose, dock_pose]()
      {
        try
        {
          runDocking(pre_dock_pose, dock_pose);
          RCLCPP_INFO(this->get_logger(), "Docking sequence completed.");
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Docking failed: %s", e.what());
        }
      })
      .detach();
  }

  geometry_msgs::msg::Pose computeDockPose(const geometry_msgs::msg::Pose &marker_pose) const
  {
    const tf2::Transform base_T_marker = poseMsgToTf(marker_pose);
    tf2::Quaternion q;
    q.setRPY(marker_to_tool_rpy_[0], marker_to_tool_rpy_[1], marker_to_tool_rpy_[2]);
    const tf2::Vector3 translation(marker_to_tool_translation_[0], marker_to_tool_translation_[1],
                                   marker_to_tool_translation_[2]);
    const tf2::Transform marker_T_tool(q, translation);
    const tf2::Transform base_T_tool = base_T_marker * marker_T_tool;
    return tfToPoseMsg(base_T_tool);
  }

  geometry_msgs::msg::Pose computePreDockPose(const geometry_msgs::msg::Pose &dock_pose) const
  {
    tf2::Transform dock_tf = poseMsgToTf(dock_pose);
    const tf2::Vector3 z_axis = dock_tf.getBasis().getColumn(2);  // Tool +Z axis
    dock_tf.setOrigin(dock_tf.getOrigin() - approach_distance_ * z_axis);
    return tfToPoseMsg(dock_tf);
  }

  void runDocking(const geometry_msgs::msg::Pose &pre_dock, const geometry_msgs::msg::Pose &dock_pose)
  {
    move_group_.setStartStateToCurrentState();
    move_group_.setPoseTarget(pre_dock, end_effector_link_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      throw std::runtime_error("Planning to pre-dock pose failed");
    }
    move_group_.execute(plan);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(dock_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_.computeCartesianPath(
      waypoints, cartesian_step_, 0.0, trajectory, true);

    if (fraction < cartesian_fraction_threshold_)
    {
      throw std::runtime_error(
              "Cartesian approach incomplete. Fraction=" + std::to_string(fraction));
    }

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;
    move_group_.execute(cartesian_plan);
  }

  // Parameters / state
  std::string marker_topic_;
  std::string reference_frame_;
  std::string end_effector_link_;
  double approach_distance_;
  double cartesian_step_;
  double cartesian_fraction_threshold_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  std::vector<double> marker_to_tool_translation_;
  std::vector<double> marker_to_tool_rpy_;
  int target_marker_id_;
  std::string marker_frame_prefix_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  std::mutex mutex_;
  bool docking_started_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDockingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
