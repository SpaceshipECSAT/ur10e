#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <gazebo_msgs/srv/get_model_state.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <fstream>

using namespace std::chrono_literals;

class AutoDock : public rclcpp::Node
{
public:
  AutoDock()
  : Node("auto_dock_node"),
    move_group_(std::shared_ptr<rclcpp::Node>(this, [](auto *){}), "ur5_manipulator"),
    planning_scene_interface_(),
    logger_(rclcpp::get_logger("AutoDock"))
  {
    RCLCPP_INFO(logger_, "=== Autonomous Docking Node Started ===");

    // Service clients
    attach_client_ = this->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    detach_client_ = this->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
    model_state_client_ = this->create_client<gazebo_msgs::srv::GetModelState>("/gazebo/get_model_state");

    move_group_.setPoseReferenceFrame("base_link");
    move_group_.setPlanningTime(10.0);
    move_group_.allowReplanning(true);
    move_group_.setGoalTolerance(0.02);
    move_group_.setMaxVelocityScalingFactor(0.2);
    move_group_.setMaxAccelerationScalingFactor(0.2);

    // Wait for Gazebo service
    while (!model_state_client_->wait_for_service(2s))
      RCLCPP_INFO(logger_, "Waiting for /gazebo/get_model_state ...");

    // Run the docking sequence once after 2 seconds
    timer_ = this->create_wall_timer(2s, std::bind(&AutoDock::runDocking, this));
  }

private:
  // === Helpers ===
  geometry_msgs::msg::Pose getTargetPose(const std::string &model_name)
  {
    auto request = std::make_shared<gazebo_msgs::srv::GetModelState::Request>();
    request->model_name = model_name;

    auto future = model_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(logger_, "Received target pose from Gazebo.");
      return future.get()->pose;
    }
    throw std::runtime_error("Failed to get target pose from Gazebo!");
  }

  geometry_msgs::msg::Pose offsetAlongTargetZ(const geometry_msgs::msg::Pose &target_pose, double offset)
  {
    tf2::Quaternion q_target;
    tf2::fromMsg(target_pose.orientation, q_target);

    tf2::Vector3 z_axis(0, 0, 1);
    tf2::Vector3 z_world = tf2::quatRotate(q_target, z_axis);

    geometry_msgs::msg::Pose result = target_pose;
    result.position.x -= offset * z_world.x();
    result.position.y -= offset * z_world.y();
    result.position.z -= offset * z_world.z();
    return result;
  }

  void attachObject()
  {
    auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    request->model1_name = "cobot";
    request->link1_name  = "wrist_3_link";
    request->model2_name = "docked_spacecraft";
    request->link2_name  = "link";

    while (!attach_client_->wait_for_service(1s))
      RCLCPP_WARN(logger_, "Waiting for AttachLink service...");

    auto future = attach_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future)
        == rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(logger_, "Hard-capture: links attached.");
    else
      RCLCPP_ERROR(logger_, "AttachLink call failed.");
  }

  void detachObject()
  {
    auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    request->model1_name = "cobot";
    request->link1_name  = "wrist_3_link";
    request->model2_name = "docked_spacecraft";
    request->link2_name  = "link";

    while (!detach_client_->wait_for_service(1s))
      RCLCPP_WARN(logger_, "Waiting for DetachLink service...");

    auto future = detach_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future)
        == rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(logger_, "Detached successfully.");
    else
      RCLCPP_ERROR(logger_, "DetachLink call failed.");
  }

  void moveToPose(const geometry_msgs::msg::Pose &pose)
  {
    move_group_.setPoseTarget(pose, "wrist_3_link");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
      RCLCPP_INFO(logger_, "Executing motion to pre-dock pose...");
      move_group_.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger_, "Planning to pre-dock failed!");
      throw std::runtime_error("Plan failed");
    }
  }

  void cartesianApproach(const geometry_msgs::msg::Pose &dock_pose)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(dock_pose);

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj);
    if (fraction > 0.9)
    {
      RCLCPP_INFO(logger_, "Executing Cartesian path (%.1f%% achieved)", fraction * 100.0);
      moveit::planning_interface::MoveGroupInterface::Plan cplan;
      cplan.trajectory_ = traj;
      move_group_.execute(cplan);
    }
    else
      RCLCPP_WARN(logger_, "Cartesian path incomplete (%.1f%% achieved)", fraction * 100.0);
  }

  void runDocking()
  {
    timer_->cancel();
    try
    {
      // 1. Get target spacecraft pose
      auto target_pose = getTargetPose("docked_spacecraft");

      // 2. Compute pre-dock pose (25 cm offset)
      auto pre_dock_pose = offsetAlongTargetZ(target_pose, 0.25);

      // 3. Move to pre-dock
      moveToPose(pre_dock_pose);

      // 4. Cartesian straight-in approach
      cartesianApproach(target_pose);

      // 5. Simulate hard capture
      attachObject();

      // 6. Log result
      std::ofstream log("/tmp/docking_results.csv", std::ios::app);
      log << target_pose.position.x << "," << target_pose.position.y << ","
          << target_pose.position.z << "," << "1" << std::endl;
      log.close();

      RCLCPP_INFO(logger_, "Docking sequence completed and logged.");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(logger_, "Docking sequence failed: %s", e.what());
    }

    rclcpp::shutdown();
  }

  // === Members ===
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Logger logger_;
  rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
  rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;
  rclcpp::Client<gazebo_msgs::srv::GetModelState>::SharedPtr model_state_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// === main ===
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoDock>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
