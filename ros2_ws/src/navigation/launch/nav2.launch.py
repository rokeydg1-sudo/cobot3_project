from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from nav2_common.launch import RewrittenYaml

from launch_ros.actions import Node


def generate_launch_description():

    # =====================================================
    # Launch arguments
    # =====================================================

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
        },
        convert_types=True,
    )

    return LaunchDescription([

        # =================================================
        # Arguments
        # =================================================

        DeclareLaunchArgument(
            "map",
            description="Full path to factory map yaml",
        ),

        DeclareLaunchArgument(
            "params_file",
            description="Full path to Nav2 parameter yaml",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        # =================================================
        # Map Server
        # =================================================

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[configured_params],
        ),

        # =================================================
        # AMCL
        # =================================================

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[configured_params],
        ),

        # =================================================
        # Planner Server
        # =================================================

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params],
        ),

        # =================================================
        # Controller Server
        # =================================================

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[configured_params],
            remappings=[
                ("cmd_vel", "/cmd_vel"),
            ],
        ),

        # =================================================
        # Behavior Server
        # =================================================

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[configured_params],
        ),

        # =================================================
        # BT Navigator
        #
        # ★ 이것이 NavigateToPose Action Server
        # =================================================

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_params],
        ),

        # =================================================
        # Lifecycle Manager
        # =================================================

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,

                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }],
        ),
    ])