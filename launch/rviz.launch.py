import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


import xacro

def generate_launch_description():
    
    pkg_path = os.path.join(get_package_share_directory('spatial_manipulator'))
    xacro_file = os.path.join(pkg_path,"description","robot.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    
    params = {
        'robot_description':robot_description_config.toxml(),
        'use_sim_time':True,
    }
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(pkg_path,"config","default.rviz")
        ]
    )
    
    joint_state_publisher_gui= Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )
    
    return LaunchDescription([
        node_robot_state_publisher,
        rviz2,
        joint_state_publisher_gui
    ])