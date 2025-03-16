import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

import xacro

def generate_launch_description():
    
    use_ros2_control= LaunchConfiguration("use_ros2_control")
    
    use_ros2_control_arg= DeclareLaunchArgument(
        "use_ros2_control",
        default_value="true",
        description="Use ros2 control if true"
    )
    
    pkg_path = os.path.join(get_package_share_directory('spatial_manipulator'))
    xacro_file = os.path.join(pkg_path,"description","robot.urdf.xacro")
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file,' use_ros2_control:=',use_ros2_control])
    params = {
        'robot_description':robot_description_config,
        'use_sim_time':True,
    }
    
    moveit_config =(
        MoveItConfigsBuilder("spatial_manipulator")
        .robot_description(file_path=xacro_file)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    run_moveit_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),{"use_sim_time":True}]
        
    )
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
                          'extra_gazebo_args':'--ros-args --params-file ' + gazebo_params_file}.items()
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
            "120",
            "-z",
            "0.1"
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
        arguments=["joint_state_broadcaster"]
    )
    
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"]
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"]
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
    twist_mux_param = os.path.join(pkg_path,"config","twist_mux.yaml")
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            twist_mux_param,
            {"use_sim_time":True}
        ],
        remappings=[("/cmd_vel_out","/diff_cont/cmd_vel_unstamped")]
    )
    
    return LaunchDescription([
        use_ros2_control_arg,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        gripper_controller_spawner,
        rviz2,
        twist_mux_node,
        run_moveit_group_node
    ])