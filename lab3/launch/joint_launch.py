import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file_name = 'bogson_move.urdf.xacro.xml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        # Node(
        #     package='lab2',
        #     executable='move_joints',
        #     name='move_joints',
        #     output='screen'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro', ' ', os.path.join(get_package_share_directory('lab3'), xacro_file_name)])}]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro', ' ', os.path.join(get_package_share_directory('lab3'), xacro_file_name)])}])
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='bogson_rviz2',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time,}],
        #     arguments=['-d', os.path.join(get_package_share_directory('lab2'), rviz2_file_name)],)
    ])