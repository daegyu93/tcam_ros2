from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tcam_ros2',
            executable='mono_camera_publisher',
            name='front_camera_node',
            arguments=['0'],    # camera device number 
            remappings=[
                ('/image_raw', '/image_front')
            ],
        ),
        Node(
            package='tcam_ros2',
            executable='mono_camera_publisher',
            name='front_right_camera_node',
            arguments=['1'],   
            remappings=[
                ('/image_raw', '/image_front_right')
            ],
        ),
        Node(
            package='tcam_ros2',
            executable='mono_camera_publisher',
            name='front_left_camera_node',
            arguments=['2'],   
            remappings=[
                ('/image_raw', '/image_front_left')
            ],
        ),
        Node(
            package='tcam_ros2',
            executable='mono_camera_publisher',
            name='rear_right_camera_node',
            arguments=['3'],   
            remappings=[
                ('/image_raw', '/image_rear_right')
            ],
        ),
        Node(
            package='tcam_ros2',
            executable='mono_camera_publisher',
            name='rear_left_camera_node',
            arguments=['4'],    
            remappings=[
                ('/image_raw', '/image_rear_left')
            ],
        ),
        Node(
            package='tcam_ros2',
            executable='mono_camera_publisher',
            name='front_down_camera_node',
            arguments=['5'],
            remappings=[
                ('/image_raw', '/image_front_down')
            ],
        ),
    ])
