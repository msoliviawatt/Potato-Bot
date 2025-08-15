# Save and Reload Rviz Session

If you want to save your current session in rviz, go to file --> save config as --> save it to this directory with some name (e.g "rviz_view_1")

Then to reload this session later on, you can run something like the following

```
rviz2 -d src/robot_description/config/rviz_view_1.rviz
```
but change the path to match whatever you named yours