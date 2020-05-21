# rbkairos_sim

This package contains launch files to start the robot(s) in simulation.

## Dependencies

This simulation depends on some Robotnik Automation packages:

- [rbkairos_common](https://github.com/RobotnikAutomation/rbkairos_common)

- [roboticsgroup_gazebo_plugins](https://github.com/RobotnikAutomation/roboticsgroup_gazebo_plugins)

Other dependencies can be installed running the following command from the workspace root:

```
rosdep install --from-paths src --ignore-src -r -y
```


## Bringup


```
roslaunch rbkairos_sim_bringup rbkairos_complete.launch

```

This launch files runs Gazebo and the ROS controllers to work with the RB-KAIROS robot. This launch accepts multiple parameters to work with.

By default it runs a full world with all the standard packages for navigation, localization and manipulation.

If you want to launch Moveit you have to run:


```
roslaunch rbkairos_sim_bringup rbkairos_complete.launch move_group_robot_a:=true
```


The default configuration will load a RB-Kairos + UR5 + EGH gripper. Other configurations are also available.
