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

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key


For Gazebo plugin: 

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

## SLAM Map generation

For Mapping the environment, parameter `mode` needs to be set mapping instead of localization in mapper_params_online_async.yaml file. Then run the following

    ros2 launch spatial_manipulator online_sync_launch.py use_sim_time:=true

After mapping the environment, in RViz use the slam_toolbox plugin to save the map.

# Navigation
Navigation requires localization on map. This can be done by two packages.

1. **slam_toolbox**

    While navigating `mode` in parameter file needs to be set to localization. **Also the parameter `map_file_name` needs to be changed to the path of your map**. Then run the following:

       ros2 launch spatial_manipulator online_sync_launch.py use_sim_time:=true

2. **nav2_amcl**

        ros2 launch spatial_manipulator localize_launch.py map:=<PATH>/to/map.yaml use_sim_time:=true

    One additional thing to do in nav2_amcl is give approximate pose estimate of robot in Rviz to initialize the position of the robot.


After Map has been published and localization is started. We can navigate in map using nav2. This can be done by:

    ros2 launch spatial_manipulator navigation_launch.py use_sim_time:=true

We can tweak additional parameters in nav2_param file for additional features.

## Common problems

- gzserver process died or robot doesn't spawn in gazebo environment

      killall gzserver