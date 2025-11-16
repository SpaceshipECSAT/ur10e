#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

// added for mesh loading
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

const double tau = 2 * M_PI;

class PickAndPlace
{
public:
    PickAndPlace(rclcpp::Node::SharedPtr node)
        : move_group(node, "ur10_manipulator"), //
          gripper(node, "robotic_gripper"), //gripper
          planning_scene_interface(),
          logger(rclcpp::get_logger("PickAndPlace")),

          // link attacher
          node_(node)
    {
        move_group.setPoseReferenceFrame("base_link");
        attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
    }

    void close_gripper()
    {
        gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.2);
        gripper.move();
    }

    void open_gripper()
    {
        gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
        gripper.move();
    }

        void up()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);
        move_group.allowReplanning(true);
        move_group.setGoalTolerance(0.03);

        geometry_msgs::msg::Pose up;
        tf2::Quaternion orientation;
        orientation.setRPY(-1.571, -0.15, -0.002);
        up.orientation = tf2::toMsg(orientation);
        up.position.x = 0.00;
        up.position.y = 0.191;
        up.position.z = 1.00;
        move_group.setPoseTarget(up, "wrist_3_link");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing pick plan: %s", success ? "SUCCESS" : "FAILED");

        // // Execution
        // if (success)
        // {
        //     move_group.move();
        //     RCLCPP_INFO(logger, "Pick motion execution completed.");

        //     close_gripper();
        //     rclcpp::sleep_for(std::chrono::seconds(1));

        //     attachObject();
        //     rclcpp::sleep_for(std::chrono::seconds(1));
        // }
        // else
        // {
        //     RCLCPP_ERROR(logger, "Motion planning for pick failed!");
        // }
    }

    void pick()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);
        move_group.allowReplanning(true);
        move_group.setGoalTolerance(0.01);

        geometry_msgs::msg::Pose pick_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-1.592, 0.022, 0.01);
        pick_pose.orientation = tf2::toMsg(orientation);
        pick_pose.position.x = -0.003;
        pick_pose.position.y = 0.258;
        pick_pose.position.z = 0.988;
        move_group.setPoseTarget(pick_pose, "wrist_3_link");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing pick plan: %s", success ? "SUCCESS" : "FAILED");

        // Execution
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Pick motion execution completed.");

            close_gripper();
            rclcpp::sleep_for(std::chrono::seconds(1));

            attachObject();
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for pick failed!");
        }
    }

    void place()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);
        move_group.allowReplanning(true);
        move_group.setGoalTolerance(0.03);

        //pose target
        geometry_msgs::msg::Pose place_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-1.548, 0.009, 1.575);
        place_pose.orientation = tf2::toMsg(orientation);
        place_pose.position.x = 0.485;
        place_pose.position.y = -0.013;
        place_pose.position.z = 0.777;

        move_group.setPoseTarget(place_pose, "wrist_3_link");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing plan: %s", success ? "SUCCESS" : "FAILED");

        // Execution
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Motion execution completed.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning failed!");
        }
    }

    void attachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "cobot";            // robot in Gazebo
        request->link1_name = "wrist_3_link";      // gripper link
        request->model2_name = "other_side";        // object model
        request->link2_name = "link";            // object link

        while (!attach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
        }

        auto future = attach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object attached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to attach object.");
        }
    }

    void detachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "cobot";
        request->link1_name = "wrist_3_link";
        request->model2_name = "other_side";
        request->link2_name = "link";

        while (!detach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the DetachLink service...");
        }

        auto future = detach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object detached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to detach object.");
        }
    }

    void addCollisionObjects()
    {
        // Clear and add mesh-based objects
        collision_objects.clear();
        collision_objects.reserve(2);

        // === other_side (from your world pose/orientation) ===
        {
            moveit_msgs::msg::CollisionObject obj;
            obj.id = "other_side";
            obj.header.frame_id = "world";

            shapes::ShapeMsg mesh_msg;
            shapes::Mesh* shape = shapes::createMeshFromResource("package://ur_yt_sim/meshes/other_side.dae");
            if (!shape)
            {
                RCLCPP_ERROR(logger, "Failed to load mesh: package://ur_yt_sim/meshes/other_side.dae");
            }
            else
            {
                shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
                shapes::constructMsgFromShape(shape, mesh_msg);
                mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
                obj.meshes.push_back(mesh);

                geometry_msgs::msg::Pose pose;
                pose.position.x = -0.002;   // approx from <model name='other_side'> state
                pose.position.y = 0.455;
                pose.position.z = 1.786;
                tf2::Quaternion q;
                q.setRPY(-1.57055, 0.0, 0.0);
                pose.orientation = tf2::toMsg(q);
                obj.mesh_poses.push_back(pose);

                obj.operation = moveit_msgs::msg::CollisionObject::ADD;
                collision_objects.push_back(obj);
            }
        }

        // === docked_spacecraft (using notch_ring mesh + world pose/orientation) ===
        {
            moveit_msgs::msg::CollisionObject obj;
            obj.id = "docked_spacecraft";
            obj.header.frame_id = "world";

            shapes::ShapeMsg mesh_msg;
            shapes::Mesh* shape = shapes::createMeshFromResource("package://ur_yt_sim/meshes/notch_ring.dae");
            if (!shape)
            {
                RCLCPP_ERROR(logger, "Failed to load mesh: package://ur_yt_sim/meshes/notch_ring.dae");
            }
            else
            {
                shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
                shapes::constructMsgFromShape(shape, mesh_msg);
                mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
                obj.meshes.push_back(mesh);

                geometry_msgs::msg::Pose pose;
                pose.position.x = -1.243;   // approx from <model name='docked_spacecraft'> state
                pose.position.y = -0.004;
                pose.position.z = 1.573;
                tf2::Quaternion q;
                q.setRPY(0.0, 1.56736, 0.0);
                pose.orientation = tf2::toMsg(q);
                obj.mesh_poses.push_back(pose);

                obj.operation = moveit_msgs::msg::CollisionObject::ADD;
                collision_objects.push_back(obj);
            }
        }

        // Add objects to the scene
        if (!collision_objects.empty())
        {
            planning_scene_interface.applyCollisionObjects(collision_objects);
            RCLCPP_INFO(logger, "Collision meshes added to the planning scene.");
        }
        else
        {
            RCLCPP_WARN(logger, "No collision objects were added.");
        }
    }

    // void attachCollisionObject() { ... }
    // void detachCollisionObject() { ... }

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    rclcpp::Logger logger;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;
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

    // Add Collision object
    pick_and_place.addCollisionObjects();
    rclcpp::sleep_for(std::chrono::seconds(1));

    //
    pick_and_place.up();
    rclcpp::sleep_for(std::chrono::seconds(10));

    // Pick Execution
    pick_and_place.pick();
    rclcpp::sleep_for(std::chrono::seconds(10));

    // Place Execution
    pick_and_place.place();
    rclcpp::sleep_for(std::chrono::seconds(10));

    // Open gripper + detach (Gazebo link attacher)
    pick_and_place.open_gripper();
    rclcpp::sleep_for(std::chrono::seconds(1));
    pick_and_place.detachObject();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Spegni ROS2
    rclcpp::shutdown();
    return 0;
}
