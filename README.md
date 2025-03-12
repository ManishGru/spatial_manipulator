# Spatial Manipulator

## Setting up package
Change the directory to your workspace

    cd <PATH_TO_WS>/src

Clone this repo in the location

    git clone https://github.com/ManishGru/spatial_manipulator

## Setting up Gazebo world

Clone below repo with models and worlds. 

    git clone https://github.com/leonhartyao/gazebo_models_worlds_collection

Add the following line in .bashrc for loading model and worlds in gazebo

    source /usr/share/gazebo/setup.bash

    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<PATH>/gazebo_models_worlds_collection/models
    
    export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:<PATH>/gazebo_models_worlds_collection/worlds

## Building 

Change the directory to your workspace

    cd <PATH_TO_WS>

Build the package using colcon build command

    colcon build --symlink-install --packages-select spatial_manipulator

## Launching Gazebo Sim and Rviz

Differential Drive control can be setup in two different ways either using ros2_control or gazebo_plugin. For launching Gazebo simulation and rviz with bot using ros2_control:

    ros2 launch spatial_manipulator robot.launch.py use_ros2_control:=true

Using Gazebo Plugin:

     ros2 launch spatial_manipulator robot.launch.py use_ros2_control:=false



## Controlling the Bot

Bot can be controlled using teleop_twist_keyboard node.
For ros2_control: 

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstammped


For Gazebo plugin: 

    ros2 run teleop_twist_keyboard teleop_twist_keyboard