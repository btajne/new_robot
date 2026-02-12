import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    # ------------------------------------------------
    # Hardware Interface
    # ------------------------------------------------
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        )
    )

    # ------------------------------------------------
    # RPLidar (RESPAWN ENABLED)
    # ------------------------------------------------
    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[
            os.path.join(
                get_package_share_directory("bumperbot_bringup"),
                "config",
                "rplidar_a1.yaml"
            )
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0
    )

    # ------------------------------------------------
    # Controller
    # ------------------------------------------------
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "controller.launch.py"
            )
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
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
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
    # Localization (NO SLAM)
    # ------------------------------------------------
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_localization"),
                "launch",
                "global_localization.launch.py"
            )
        ),
        condition=UnlessCondition(use_slam)
    )

    # ------------------------------------------------
    # SLAM (DELAYED)
    # ------------------------------------------------
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_mapping"),
                "launch",
                "slam.launch.py"
            )
        ),
        condition=IfCondition(use_slam)
    )

    slam_delayed = TimerAction(
        period=6.0,
        actions=[slam]
    )

    # ------------------------------------------------
    # Navigation (DELAYED MORE)
    # ------------------------------------------------
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_navigation"),
                "launch",
                "navigation.launch.py"
            )
        )
    )

    navigation_delayed = TimerAction(
        period=12.0,
        actions=[navigation]
    )

    # ------------------------------------------------
    # Launch Description
    # ------------------------------------------------
    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_driver_node,
        localization,
        slam_delayed,
        navigation_delayed,
    ])
