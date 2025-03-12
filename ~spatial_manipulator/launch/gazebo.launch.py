import os
import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare(package='my_arm_description').find('my_arm_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'arm_v2.urdf')
    pkg_path=get_package_share_directory('my_arm_description')
    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("gazzebo_ros"),"launch","gazebo.launch.py")
        ]),
        launch_arguments={"world":os.path.join(pkg_path, "world","office_small.world")}.items()
    )
    return launch.LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Use simulation clock'),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'arm_v2', '-file', urdf_file],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='gzserver',
            output='screen',
            parameters=[{'use_sim_time': 'true'}],
        ),


        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        )
    ])
