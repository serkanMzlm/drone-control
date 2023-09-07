
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

micro_ros_agent = ExecuteProcess(
        cmd=[[
            'micro-ros-agent udp4 --port 8888'
        ]],
        shell=True
    )

command = Node(
        package='command',
        executable='command_node',
        output='screen',
        shell=True,
    )

def generate_launch_description():
    return LaunchDescription([
        micro_ros_agent,
        command
    ])