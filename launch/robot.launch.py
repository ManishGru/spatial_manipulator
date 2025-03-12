import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


import xacro

def generate_launch_description():
    
    use_ros2_control= LaunchConfiguration("use_ros2_control")
    
    pkg_path = os.path.join(get_package_share_directory('spatial_manipulator'))
    xacro_file = os.path.join(pkg_path,"description","robot.urdf.xacro")
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file,' use_ros2_control:=',use_ros2_control])
    params = {
        'robot_description':robot_description_config,
        'use_sim_time':True,
    }
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    gazebo_params_file = os.path.join(pkg_path,"config","gazebo_params.yaml")
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("gazebo_ros"),"launch","gazebo.launch.py")]),
        launch_arguments={"world":os.path.join(pkg_path,"world","office_small.world"),
                          'extra_gazebo_args':'--ros-args --params-file' + gazebo_params_file}.items()
    ) 
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "spatial_manipulator_bot",
            "-timeout",
            "120"
        ],
        output='screen',
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )
    
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
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
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="true",
            description="Use ros2 control if true"),
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broadcaster_spawner,
        rviz2,
    ])