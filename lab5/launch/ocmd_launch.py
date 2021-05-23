import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file_name = 'bogson_move.urdf.xacro.xml'
    p1_1 = 0.
    p2_1 = 0.
    p3_1 = 0.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='lab5',
            executable='ocmd',
            name='ocmd',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'p1_1': p1_1, 'p2_1': p2_1, 'p3_1': p3_1}])#,
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro', ' ', os.path.join(get_package_share_directory('lab4'), xacro_file_name)])}])
    ])