# rbkairos_sim


Packages for the simulation of the RB-Kairos mobile manipulator.

## Dependencies

This simulation depends on some Robotnik Automation packages:

- [rbkairos_common](https://github.com/RobotnikAutomation/rbkairos_common)


It also depends on the UR ROS driver:

- [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

It is highly recomended to read the UR ROS driver readme file to install it correctly.


Other dependencies can be installed running the following command from the workspace root:

```
rosdep install --from-paths src --ignore-src -r -y
```


## Launch files

The simulation is separeted in three different launch files:

- **simulator**: Launches the simulator and the environment. By default it uses gazebo simulator and a .world file.
  - Run example: 

    ```bash
    roslaunch rbkairos_sim_bringup run_simulator.launch
    ```

- **robot**: Spawns a simulation of the hardware of the robot in the simulated environment.
  - Run example: 

    ```bash
    roslaunch rbkairos_sim_bringup run_robot.launch
    ```

- **rviz**: Starts an RVIZ instance to visualize the sensors data.
  - Run example:

    ```bash
    roslaunch rbkairos_sim_bringup run_rviz.launch
    ```

There is also prepared a complete launch the starts a complete simulation of the environment and the robot with a prepared RVIZ config. Run example:

```bash
    roslaunch rbkairos_sim_bringup rbkairos_complete.launch
```

## Spawning several robots

The simulation is set up to work with several robots. First, you will need to start the simulator:

```bash
    roslaunch rbkairos_sim_bringup run_simulator.launch
```

Then you will need to execute the run_robot launch as many times as the number of robots that you want to spawn. In this case, the **robot_id**, **x_init_pose**, **y_init_pose** arguments need to be set to ensure that the robots use different names and ROS namespaces and that their are spawned  in different places. Run example:

```bash
    roslaunch rbkairos_sim_bringup run_robot.launch robot_id:=robot_a x_init_pose:=1.0 y_init_pose:=2.0
```