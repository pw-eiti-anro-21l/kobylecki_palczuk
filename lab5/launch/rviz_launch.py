import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # urdf_file_name = 'bogson.urdf.xml'
    rviz2_file_name = 'bogson.rviz'
    xacro_file_name = 'bogson_fixed.urdf.xacro.xml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='bogson_rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,}],
            arguments=['-d', os.path.join(get_package_share_directory('lab5'), rviz2_file_name)])#,
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro', ' ', os.path.join(get_package_share_directory('lab4'), xacro_file_name)])}])
    ])