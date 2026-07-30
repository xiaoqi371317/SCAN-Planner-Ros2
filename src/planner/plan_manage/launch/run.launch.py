"""Main ROS 2 launch entry point for simulation and real-robot remapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    reference_path_file = LaunchConfiguration("reference_path_file").perform(context)
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )
    if reference_path_file and navi_mode != 3:
        raise RuntimeError("reference_path_file is only valid when navi_mode=3")
    if reference_path_file and not os.path.isfile(reference_path_file):
        raise RuntimeError(
            "reference_path_file must reference an existing ROS 2 parameter YAML"
        )

    mode_default_init = (-19.0, 1.0, 0.25) if navi_mode == 1 else (-5.5, 5.5, 0.5)
    initial_position = []
    for name, default in zip(("init_x", "init_y", "init_z"), mode_default_init):
        value = LaunchConfiguration(name).perform(context)
        initial_position.append(default if value == "" else float(value))
    init_x, init_y, init_z = initial_position

    if is_real:
        body_pose = "/LIO/odom_vehicle"
        sensor_pose = "/LIO/odom_imu"
        cloud = "/LIO/clouds_lidar"
        depth = "/camera/aligned_depth_to_color/image_raw"
        cloud_is_world = False
        need_extrinsic = True
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
    else:
        body_pose = "/quad_0/body_pose"
        sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = True
        need_extrinsic = False
        intrinsics = {}

    common = {"use_sim_time": use_sim_time}
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
    }
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml] + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
        )
    ]
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="go2_robot_state_publisher",
            output="screen",
            parameters=[
                common,
                {
                    "robot_description": Command(
                        ["xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                         " use_gazebo:=false"]
                    )
                },
            ],
        )
    )

    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[
                    controllers_yaml,
                    common,
                    {"init_x": init_x, "init_y": init_y, "init_z": init_z},
                ],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("cmd_vel", "/cmd_vel" if is_real else "/quad_0/cmd_vel"),
                ],
            )
        )
        if not is_real:
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        common,
                        {
                            "init_x": init_x,
                            "init_y": init_y,
                            "init_z": init_z,
                            "publish_tf": False,
                        },
                    ],
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/quad_0/cmd_vel"),
                    ],
                )
            )

    if reference_path_file:
        actions.append(
            Node(
                package="scan_planner",
                executable="reference_path_publisher.py",
                name="reference_path_publisher",
                output="screen",
                parameters=[reference_path_file, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("initial_path", "/initial_path"),
                ],
            )
        )

    if not is_real:
        actions.extend(
            [
                Node(
                    package="scan_planner",
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose)],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "sensor_type",
                            "use_gpu",
                            "use_pcd_map",
                            "pcd_map_file",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "use_sim_time",
                        )
                    }.items(),
                ),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("reference_path_file", default_value=""),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("init_x", default_value=""),
            DeclareLaunchArgument("init_y", default_value=""),
            DeclareLaunchArgument("init_z", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
