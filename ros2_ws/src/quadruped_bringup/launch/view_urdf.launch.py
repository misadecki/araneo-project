import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc = get_package_share_directory('quadruped_description')
    urdf_path = os.path.join(desc, 'urdf', 'araneo.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0',
                        '--pitch', '0', '--yaw', '0', '--frame-id', 'map',
                        '--child-frame-id', 'base_link']),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(desc, 'rviz', 'araneo.rviz')]),
    ])
