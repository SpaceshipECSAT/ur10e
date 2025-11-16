from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    ld = LaunchDescription()

    # --- share dirs ---
    uryt_share     = get_package_share_directory("ur_yt_sim")
    robotiq_share  = get_package_share_directory("robotiq_description")
    ur_share       = get_package_share_directory("ur_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    world_file = os.path.join(get_package_share_directory('ur_yt_sim'), 'worlds', 'world4.world')

    # --- ENV Gazebo ---
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=":".join(["/usr/share/gazebo-11", uryt_share, robotiq_share, ur_share])
    ))
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=":".join([
            os.path.join(uryt_share,"models"),
            os.path.join(robotiq_share,"models"),
            os.path.expanduser("~/.gazebo/models")
        ])
    ))
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=":".join([
            "/opt/ros/humble/lib",
            os.path.normpath(os.path.join(uryt_share, "..", "..", "lib")),
        ])
    ))

    # --- args ---
    with_rviz     = DeclareLaunchArgument("with_rviz", default_value="true")
    with_octomap  = DeclareLaunchArgument("with_octomap", default_value="true")  # << NEW
    with_aruco    = DeclareLaunchArgument("with_aruco", default_value="true")
    with_aruco_docking = DeclareLaunchArgument("with_aruco_docking", default_value="false")
    aruco_marker_id = DeclareLaunchArgument("aruco_marker_id", default_value="-1")
    aruco_marker_size = DeclareLaunchArgument("aruco_marker_size", default_value="0.05")
    aruco_docking_marker_id = DeclareLaunchArgument("aruco_docking_marker_id", default_value="0")
    aruco_dictionary = DeclareLaunchArgument("aruco_dictionary", default_value="DICT_5X5_100")
    aruco_image_topic = DeclareLaunchArgument("aruco_image_topic", default_value="/camera/image_raw")
    aruco_camera_info_topic = DeclareLaunchArgument("aruco_camera_info_topic", default_value="/camera/camera_info")
    docking_translation = DeclareLaunchArgument(
        "tool_from_marker_translation", default_value="[0.0, 0.0, 0.0]"
    )
    docking_rpy = DeclareLaunchArgument(
        "tool_from_marker_rpy", default_value="[3.14159, 0.0, 0.0]"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0")
    y_arg = DeclareLaunchArgument("y", default_value="0")
    z_arg = DeclareLaunchArgument("z", default_value="0")
    ld.add_action(with_rviz); ld.add_action(with_octomap)
    ld.add_action(x_arg); ld.add_action(y_arg); ld.add_action(z_arg)

    ld.add_action(with_aruco)
    ld.add_action(with_aruco_docking)

    ld.add_action(aruco_marker_id); ld.add_action(aruco_marker_size); ld.add_action(aruco_docking_marker_id)
    ld.add_action(aruco_dictionary)
    ld.add_action(aruco_image_topic)
    ld.add_action(aruco_camera_info_topic)
    ld.add_action(docking_translation)
    ld.add_action(docking_rpy)

    # --- MoveIt config ---
    joint_controllers_file = os.path.join(uryt_share, "config", "ur10_controllers_gripper.yaml")
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur10_camera_gripper_moveit_config")
        .robot_description(
            file_path="config/ur.urdf.xacro",
            mappings={
                "ur_type": "ur10e",
                "sim_gazebo": "true",
                "sim_ignition": "false",
                "use_fake_hardware": "false",
                "simulation_controllers": joint_controllers_file,
                "initial_positions_file": os.path.join(uryt_share, "config", "initial_positions.yaml"),
            },
        )
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .to_moveit_configs()
    )

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")),
        launch_arguments={"use_sim_time":"true", "gui":"true", "paused":"true", "world": world_file}.items()
    )
    ld.add_action(gazebo)

    # --- RSP ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
    )
    ld.add_action(robot_state_publisher)

    # --- Spawn ---
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity","cobot",
            "-topic","robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )
    ld.add_action(TimerAction(period=3.0, actions=[spawn]))

    jsb  = Node(package="controller_manager", executable="spawner",
                arguments=["joint_state_broadcaster","--controller-manager","/controller_manager"], output="screen")
    arm  = Node(package="controller_manager", executable="spawner",
                arguments=["joint_trajectory_controller","--controller-manager","/controller_manager"], output="screen")
    grip = Node(package="controller_manager", executable="spawner",
                arguments=["gripper_position_controller","--controller-manager","/controller_manager"], output="screen")

    ld.add_action(RegisterEventHandler(
        OnProcessStart(target_action=spawn, on_start=[
            TimerAction(period=2.0, actions=[jsb]),
            TimerAction(period=3.0, actions=[arm, grip]),
        ])
    ))

    rviz_cfg = os.path.join(get_package_share_directory("ur10_camera_gripper_moveit_config"),
                            "config", "moveit.rviz")
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("with_rviz"))
    )
    ld.add_action(rviz)

    mg_params = moveit_config.to_dict()
    mg_params.update({"use_sim_time": True})

    move_group_with_octomap = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[mg_params],
        arguments=["--ros-args","--log-level","info"],
        condition=IfCondition(LaunchConfiguration("with_octomap")),   # << NEW
    )

    mg_params_no_sensors = dict(mg_params)
    mg_params_no_sensors.pop("sensors", None)

    move_group_no_octomap = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[mg_params_no_sensors],
        arguments=["--ros-args","--log-level","info"],
        condition=UnlessCondition(LaunchConfiguration("with_octomap")),
    )

    

    ld.add_action(move_group_with_octomap)
    ld.add_action(move_group_no_octomap)

    aruco_detector = Node(
        package="ur_yt_sim",
        executable="aruco_detector.py",
        name="aruco_detector",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("aruco_image_topic"),
                "camera_info_topic": LaunchConfiguration("aruco_camera_info_topic"),
                "marker_size": LaunchConfiguration("aruco_marker_size"),
                "marker_id": LaunchConfiguration("aruco_marker_id"),
                "aruco_dictionary": LaunchConfiguration("aruco_dictionary"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("with_aruco")),
    )
    
    ld.add_action(aruco_detector)

    aruco_docking = Node(
        package="ur_yt_sim",
        executable="aruco_docking_node",
        name="aruco_docking_node",
        output="screen",
        parameters=[{
            "marker_pose_topic": "aruco/pose",
            "reference_frame": "base_link",
            "end_effector_link": "wrist_3_link",
            "approach_distance": 0.25,
            "tool_from_marker_translation": ParameterValue(
                LaunchConfiguration("tool_from_marker_translation"), value_type=list
            ),
            "tool_from_marker_rpy": ParameterValue(
                LaunchConfiguration("tool_from_marker_rpy"), value_type=list
            ),
            "target_marker_id": LaunchConfiguration("aruco_docking_marker_id"),
        }],
        condition=IfCondition(LaunchConfiguration("with_aruco_docking")),
    )
    ld.add_action(aruco_docking)

    return ld
