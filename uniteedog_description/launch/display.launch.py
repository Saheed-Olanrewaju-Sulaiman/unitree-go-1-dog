from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('uniteedog_description')

    urdf_path = os.path.join(pkg_path, 'urdf', 'unitreeDog.urdf')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'unitreeDog.rviz')

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(urdf_path).read()
            }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path]
        ),

    ])
