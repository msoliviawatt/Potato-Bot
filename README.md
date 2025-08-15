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
