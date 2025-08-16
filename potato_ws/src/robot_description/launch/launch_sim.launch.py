import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'robot_description'
    
    # this basically runs...
    # ros2 launch robot_description launch_sim.launch.py
    rsp = IncludeLaunchDescription (
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments = {'use_sim_time': 'true'}.items()
    )
    
    # include gazebo launch file given by default from gazebo install
    # this basically runs the command...
    # ros2 launch ros_gz_sim gz_sim.launch.py
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
    )
    
    # run spawner node from gazebo package
    # this basically runs the command...
    # ros2 run ros_gz_sim create -name potato_bot -topic /robot_description
    spawn_entity = Node(package = 'ros_gz_sim', executable = 'create',
                        arguments = ['-name', 'potato_bot',
                                     '-topic', '/robot_description'],
                        output = 'screen')
    
    # launch
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity
    ])