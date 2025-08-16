# Potato-Bot
Navigate to the potato_ws directory and open a dev container using the Dockerfile

Source the workspace:
 ``` 
 source /opt/ros/jazzy/setup.bash
 ``` 
 Then do 
 ```
 colcon build
 ```
 OR (better) 
 ```
 colcon build --symlink-install
 ```

Note: ```colcon build --symlink-install``` means you will only need to rebuild if you add any additional files, versus ```colcon build``` which forces you to rebuild after any change is made.

# Visualize in Rviz
Run the following, which uses the robot description launch file
```
ros2 launch robot_description rsp.launch.py
```

If you want to run with simulation time...
```
ros2 launch robot_description rsp.launch.py use_sim_time:=true
```

# Gazebo
Prerequisite: Gazebo is installed ( ```sudo apt-get install ros-${ROS_DISTRO}-ros-gz``` ). We're using Gazebo Harmonic

Open new terminal and run
```
ros2 launch ros_gz_sim gz_sim.launch.py
``` 
If there is a pop-up, just click the X

To add an entity to your existing session, use the following as a guide but change it as needed
```
ros2 run ros_gz_sim create -name <robot_name> -topic /robot_description
```

# Save and Reload Rviz Session
If you want to save your current session in rviz, go to file --> save config as --> save it to this directory with some name (e.g "rviz_view_1")

Then to reload this session later on, you can run something like the following

```
rviz2 -d src/robot_description/config/rviz_view_1.rviz
```
but change the path to match yours.


# Other Notes
If you want to push to GitHub, you will need to exit the dev container. Don't push to the main branch if you make changes.