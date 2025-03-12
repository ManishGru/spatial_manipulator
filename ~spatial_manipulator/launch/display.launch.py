import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package share directory for your robot
    pkg_share = FindPackageShare(package='my_arm_description').find('my_arm_description')
    
    # Set default paths for your URDF and RViz configuration files
    default_model_path = os.path.join(pkg_share, 'urdf/arm_v2.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf.rviz')

    # Declare launch arguments
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    # Declare model path and rviz config launch arguments
    decl_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
    )

    decl_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
    )

    # Declare GUI and robot state publisher launch arguments
    decl_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
    )

    decl_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
    )

    decl_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
    )

    decl_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
    )

    # Start the joint state publisher (GUI or terminal-based)
    start_joint_state_publisher_cmd = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Start the robot state publisher with the URDF
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', model])
        }],
        arguments=[default_model_path]
    )

    # Start RViz with the provided configuration
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Create the launch description and add actions
    ld = LaunchDescription()

    ld.add_action(decl_model_path_cmd)
    ld.add_action(decl_rviz_config_file_cmd)
    ld.add_action(decl_use_joint_state_publisher_cmd)
    ld.add_action(decl_use_robot_state_pub_cmd)
    ld.add_action(decl_use_rviz_cmd)
    ld.add_action(decl_use_sim_time_cmd)

    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
