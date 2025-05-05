from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    reliable_qos_arg = DeclareLaunchArgument(
        'reliable_qos',
        default_value='false',
        description='Use reliable QoS profile if true'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/rear_right/image_raw',
        description='Input image topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/camera/rear_right/image_rect',
        description='Output rectified image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/rear_right/camera_info',
        description='Camera info topic'
    )
    
    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed',
        default_value='true',
        description='Use compressed image topics if true'
    )
    
    balance_arg = DeclareLaunchArgument(
        'balance',
        default_value='0.0',
        description='Balance parameter for fisheye undistortion (0.0 to 1.0)'
    )
    
    # Create node
    node = Node(
        package='tractor_image_rectifier',
        executable='image_rectifier_node.py',
        name='image_rectifier_node',
        parameters=[{
            'reliable_qos': LaunchConfiguration('reliable_qos'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'use_compressed': LaunchConfiguration('use_compressed'),
            'balance': LaunchConfiguration('balance')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        reliable_qos_arg,
        input_topic_arg,
        output_topic_arg,
        camera_info_topic_arg,
        use_compressed_arg,
        balance_arg,
        node
    ])