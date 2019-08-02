# rbkairos_sim

This package contains launch files to start the robot(s) in simulation.

## Dependencies

Install the dependencies from the workspace root:

```
rosdep install --from-paths src --ignore-src -r -y
```


## Launch files


```
roslaunch rbkairos_sim_bringup rbkairos_complete.launch

```

This launch files runs Gazebo and the ROS controllers to work with the RBKAIROS robot. This launch accepts multiple parameters to work with.

If you want to launch Moveit you have to run:


```
roslaunch rbkairos_sim_bringup rbkairos_complete.launch move_group_robot_a:=true
```
