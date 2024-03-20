import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_prefix('hobot_vio'),
        "lib/hobot_vio/config/realsenseD435i_color.yaml")
    print("config_file_path is ", config_file_path)


    horizon_vio_node = Node(
        package='hobot_vio',
        executable='hobot_vio',
        output='screen',
        parameters=[
            {"path_config": str(config_file_path)},
            {"image_topic": "/camera/color/image_raw"},
            {"imu_topic": "/camera/imu"},
            {"sample_gap": 2}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    realsense_node = ExecuteProcess(
        cmd=[[
            'ros2 launch realsense2_camera rs_launch.py ',
            ' rgb_camera.profile:=', '640x480x30',
            ' enable_depth:=', 'false',
            ' enable_color:=', 'true',
            ' enable_gyro:=', 'true',
            ' enable_accel:=', 'true',
            ' enable_sync:=', 'true',
            ' gyro_fps:=', '200',
            ' accel_fps:=', '200',
            ' unite_imu_method:=', '2'
        ]],
        shell=True
    )

    boost = ExecuteProcess(
        cmd=[[
            'echo 1 > /sys/devices/system/cpu/cpufreq/boost '
        ]],
        shell=True
    )

    return LaunchDescription([
        horizon_vio_node,
        realsense_node,
        boost
    ])