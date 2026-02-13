import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # -------------------------
    # Launch Arguments
    # -------------------------
    use_slam = LaunchConfiguration("use_slam")
    map_name = LaunchConfiguration("map_name")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="my_map.yaml"   # change if needed
    )

    # ------------------------------------------------
    # Hardware Interface
    # ------------------------------------------------
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    # ------------------------------------------------
    # RPLidar
    # ------------------------------------------------
    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[os.path.join(
            get_package_share_directory("bumperbot_bringup"),
            "config",
            "rplidar_a1.yaml"
        )],
        output="screen",
        respawn=True,
        respawn_delay=2.0
    )

    # ------------------------------------------------
    # Controller
    # ------------------------------------------------
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    # ------------------------------------------------
    # Joystick
    # ------------------------------------------------
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    # ------------------------------------------------
    # IMU
    # ------------------------------------------------
    imu_driver_node = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py",
        output="screen"
    )

    # ------------------------------------------------
    # Localization (Map + AMCL)
    # ------------------------------------------------
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "map": map_name
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    # ------------------------------------------------
    # Lifecycle Manager for Localization
    # ------------------------------------------------
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
        condition=UnlessCondition(use_slam)
    )

    # ------------------------------------------------
    # SLAM
    # ------------------------------------------------
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    slam_delayed = TimerAction(
        period=6.0,
        actions=[slam]
    )

    # ------------------------------------------------
    # Navigation
    # ------------------------------------------------
    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )

    navigation_delayed = TimerAction(
        period=15.0,
        actions=[navigation]
    )

    # ------------------------------------------------
    # Launch Description
    # ------------------------------------------------
    return LaunchDescription([
        use_slam_arg,
        map_name_arg,

        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_driver_node,

        localization,
        lifecycle_manager_localization,

        slam_delayed,
        navigation_delayed,
    ])
