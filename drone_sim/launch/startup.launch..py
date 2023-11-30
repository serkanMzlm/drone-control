#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

drone_sim_path = get_package_share_directory('drone_sim')

drone_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(drone_sim_path,'launch','startup.py')
    )
)

##################################### NODE #####################################
joy = Node(
        package='joy',
        executable='joy_node',
        respawn=True,
        output='screen'
    )

px4_connect = ExecuteProcess(
    cmd=["MicroXRCEAgent", "udp4", "-p",  "8888"]
)

bridge_camera = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera_vision@sensor_msgs/msg/Image[gz.msgs.Image"
        ]
)

def generate_launch_description():
    return LaunchDescription([
        px4_connect,
        joy,
        bridge_camera
	]
)
