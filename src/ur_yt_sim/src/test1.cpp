#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// test: move to the home pose and then advance to a docking pose.

class PickAndPlace
{
public:
    PickAndPlace(rclcpp::Node::SharedPtr node)
        : move_group(node, "ur10_manipulator"),
          logger(rclcpp::get_logger("PickAndPlace"))
    {
        move_group.setPoseReferenceFrame("base_link");
    }

    void moveHome()
    {
        configureMotion();
        move_group.setGoalJointTolerance(0.01);
        move_group.setNamedTarget("home");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Moving to home: %s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Home motion execution completed.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for home failed!");
        }
    }

    void moveForward()
    {
        configureMotion();
        move_group.setGoalTolerance(0.03);
        move_group.setStartStateToCurrentState();

        geometry_msgs::msg::Pose forward_pose = dockingPose();

        move_group.setPoseTarget(forward_pose, "wrist_3_link");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Moving forward to docking pose: %s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            auto result = move_group.move();
            if (result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(logger, "Forward motion execution completed.");
            }
            else
            {
                RCLCPP_ERROR(logger, "Forward motion execution failed: %d", result.val);
            }
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for forward pose failed!");
        }
    }

    void moveForwardInZ()
    {
        configureMotion();
        move_group.setGoalTolerance(0.03);
        move_group.setStartStateToCurrentState();

        geometry_msgs::msg::Pose elevated_pose = forwardPose();
        elevated_pose.position.x -= 0.6; // shift 0.6m along -X

        move_group.setPoseTarget(elevated_pose, "wrist_3_link");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Moving ahead from docking pose: %s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            auto result = move_group.move();
            if (result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(logger, "Additional forward motion execution completed.");
            }
            else
            {
                RCLCPP_ERROR(logger, "Additional forward motion execution failed: %d", result.val);
            }
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for additional forward move failed!");
        }
    }

private:
    void configureMotion(double velocity_scale = 0.1, double acceleration_scale = 0.1)
    {
        move_group.setMaxVelocityScalingFactor(velocity_scale);
        move_group.setMaxAccelerationScalingFactor(acceleration_scale);
        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(10);
        move_group.allowReplanning(true);
    }

    geometry_msgs::msg::Pose dockingPose() const
    {
        geometry_msgs::msg::Pose pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-1.518, 0.015, 1.604);
        pose.orientation = tf2::toMsg(orientation);
        pose.position.x = 0.401;
        pose.position.y = -0.162;
        pose.position.z = 0.811;
        return pose;
    }

    geometry_msgs::msg::Pose forwardPose() const
    {
        geometry_msgs::msg::Pose pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-1.579, 0.015, 1.604);
        pose.orientation = tf2::toMsg(orientation);
        pose.position.x = -0.318;
        pose.position.y = -0.162;
        pose.position.z = 0.811;
        return pose;
    }

    moveit::planning_interface::MoveGroupInterface move_group;
    rclcpp::Logger logger;
};

int main(int argc, char **argv)
{
    // ROS2 Initialization
    rclcpp::init(argc, argv);
    //  auto node = std::make_shared<rclcpp::Node>("pick_and_place_node");
    // auto node = std::make_shared<rclcpp::Node>(
    //"pick_and_place_node",
    //rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));
    auto node = std::make_shared<rclcpp::Node>(
        "pick_and_place_node",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));


    PickAndPlace pick_and_place(node);

    pick_and_place.moveHome();
    rclcpp::sleep_for(std::chrono::seconds(5));

    pick_and_place.moveForward();
    rclcpp::sleep_for(std::chrono::seconds(5));

    pick_and_place.moveForwardInZ();
    rclcpp::sleep_for(std::chrono::seconds(5));


    rclcpp::shutdown();
    return 0;
}
