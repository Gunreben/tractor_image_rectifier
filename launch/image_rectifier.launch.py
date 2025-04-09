import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tractor_image_rectifier',
            executable='image_rectifier_node.py',
            name='image_rectifier_node',
            parameters=[{
                'reliable_qos': False,  # Set to True for reliable QoS
                'input_topic': 'input/image_raw/compressed',  # Change to 'input/image_raw' for uncompressed
                'output_topic': 'output/image_rect'
            }],
            output='screen'
        )
    ]) 