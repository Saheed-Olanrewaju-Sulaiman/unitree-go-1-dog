
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('unitreedog_description')
    
    # Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'unitreeDog.urdf.xacro')
    
    # Process URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 1. Gazebo Launch (Ignition/Gazebo Sim)
    # This includes the standard gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(), # -r runs it immediately
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. Spawn the robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'UnitreeDog',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.5' # Start slightly above ground
        ],
        output='screen',
    )

    # 4. Bridge (Necessary to see the robot in Gazebo and ROS)
    # This maps ROS 2 topics to Gazebo topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/UnitreeDog/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/UnitreeDog/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge
    ])

