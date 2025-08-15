# Procedure

## Dependencies
- ros-jazzy-ros-gz
- ros-jazzy-xacro
- ros-jazzy-joint-state-publisher-gui
- ros-jazzy-image-transport-plugins
- ros-jazzy-rqt-image-view
- ros-jazzy-ros2-control
- ros-jazzy-ros2-controllers
- ros-jazzy-gz-ros2-control

## Initial Setup

Go into an empty directory

Run the command
```
mkdir src
```

Run the command
```
colcon build
```

Source the workspace by running
```
source /opt/ros/jazzy/setup.bash
```

Then do
```
source install/setup.bash
```
This order ensures that locally built packages take precedence over identically named packages from the ROS2 installation.

## Robot Description
1. Navigate to `src` and create a new package with the command 
```
ros2 pkg create --build-type ament_cmake <name>_description
```
2. Navigate to the folder of the newly created package and create a new directory called `description`
3. Inside the description folder create a file with a `.xacro` extension (`robot.urdf.xacro` is a typical name). The file contents should be:
```
<?xml vesrsion="1.0"?>
<robot xmlns:xacro="https://www.ros.org/wiki/xacro" name="<your_robot_name>">
    <link name="base_link"/>
</robot>
```
4. Copy the `inertial_macros.xacro` file into the `description` folder
5. Add your own robot geometry to the urdf file
6. Now create a `launch` directory and create a python file with the following contents:
```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('<name>_description'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    
    params = {'robot_description': robot_description_config}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        robot_state_publisher
    ])
```
7. Add the following to the bottom of the package's `CMakeLists.txt`:
```
install(
  DIRECTORY description launch
  DESTINATION share/${PROJECT_NAME}
)
```
8. Build the workspace and source it again as we did before. You should now be able to run the new launch file with the command:
```
ros2 launch <name>_description <your_launch_file_name>.py
```
9. Now open rviz2, click add at the bottom and click robot model. Set the topic to `/robot_description`, and make sure that under global options, the fixed frame is set to `/base_link`

You can run ```ros2 run joint_state_publisher_gui joint_state_publisher_gui``` to get a gui of the transforms

## Gazebo
1. Create a new package as we did in part 2 but called `<name>_gazebo`. This package only needs a `launch` folder for now.
2. Open the `.xacro` file we created before and add gazebo tags as needed.
3. Create another python launch file inside the launch folder with the following contents:
```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch.actions import RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('<name>_description'), 'launch', 'rsp.launch.py'
        )])
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': ['-r -v4'], 'on_exit_shutdown': 'true' }.items()       
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', '<name>',
                                   '-z', '0.1'
                                  ],
                        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])
```
4. Add the following to the bottom of the package's `CMakeLists.txt`:
```
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```
5. Build and source as we did before and you can run the simulated robot by running the command:
```
ros2 launch <name>_gazebo <gazebo_launch_file>.py
```
6. Once gazebo opens, you can simply choose the empty world for the purposes of this demo

## Gazebo Control
1. In the urdf we created before, add the following to the bottom of the file (but still inside the `robot` tag):
```
    <gazebo>
        <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
            <frame_id>odom</frame_id>
            <child_frame_id>base_link</child_frame_id>
            <tf_topic>tf</tf_topic>
            <odom_topic>odom</odom_topic>
            <topic>cmd_vel</topic>

            <odometry_publish_frequency>50</odometry_publish_frequency>

            <left_joint>...your wheel joint name here...</left_joint>
            <right_joint>...your wheel joint name here...</right_joint>

            <wheel_separation>0.5</wheel_separation> <!-- Distance between left and right wheels -->
            <wheel_radius>0.1</wheel_radius> <!-- Radius of the wheels -->
        </plugin>
    </gazebo>
```
2. In the gazebo package, create a `config` folder and create a yaml file with a name of your choice with the following contents:

```
- ros_topic_name: "clock"
  gz_topic_name: "clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

# gz topic published by DiffDrive plugin
- ros_topic_name: "odom"
  gz_topic_name: "odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

# gz topic published by DiffDrive plugin
- ros_topic_name: "tf"
  gz_topic_name: "tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

# gz topic subscribed to by DiffDrive plugin
- ros_topic_name: "cmd_vel"
  gz_topic_name: "cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

3. Add the following to the gazebo launch file created earlier:
```
bridge_params = os.path.join(get_package_share_directory('<name>_gazebo'), 'config', 'gz_bridge.yaml') 
ros_gz_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        '--ros-args',
        '-p',
        f'config_file:={bridge_params}'
    ]
)

```
4. Modify the return statement so itt also contains `ros_gz_bridge` as shown below:
```
return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        ros_gz_bridge
    ])
```
5. Now we need to modify the line we added earlier in `CMakeLists.txt`. Add the config file after `launch`, which should give you something like this:
```
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
```

6. Build and source as we did before and launch the gazebo launch file again. At this point you should be able to run `teleop_twist_keyboard` and drive the robot around in Gazebo.

## ROS2 Control
1. Create a package called `<name>_hardware` as we did before. 
2. In the `include/<name>_hardware` folder, create an hpp file with the following contents:
```
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace demo_hardware
{

class DemoHardware : public hardware_interface::SystemInterface
{

public:

    DemoHardware();

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;   
    
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;
    
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

    double l_wheel_1_state_vel;
    double l_wheel_2_state_vel;
    double r_wheel_1_state_vel;
    double r_wheel_2_state_vel;
    double l_wheel_1_state_pos;
    double l_wheel_2_state_pos;
    double r_wheel_1_state_pos;
    double r_wheel_2_state_pos;

    double l_wheel_1_cmd_vel;
    double l_wheel_2_cmd_vel;
    double r_wheel_1_cmd_vel;
    double r_wheel_2_cmd_vel;

    rclcpp::Logger logger_;

    std::chrono::time_point<std::chrono::system_clock> time_;
};

}
```
2. In the `src` folder, create a `.cpp` file that includes the `.hpp` file we created earlier. The contents should be:
```
#include "demo_hardware/demo_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace demo_hardware{

DemoHardware::DemoHardware()
  : logger_(rclcpp::get_logger("DemoHardware"))
{}


hardware_interface::CallbackReturn DemoHardware::on_init(const hardware_interface::HardwareInfo & info) {

    // In this function you should validate the passed in hardware information and set any variables as required

    RCLCPP_INFO(logger_, "Configuring...");

    time_ = std::chrono::system_clock::now();

    RCLCPP_INFO(logger_, "Finished Configuration");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DemoHardware::on_configure(
    const rclcpp_lifecycle::State & previous_state) {

    // Here you should try to connect to the comms system

    return hardware_interface::CallbackReturn::SUCCESS;
}   

hardware_interface::CallbackReturn DemoHardware::on_cleanup(
    const rclcpp_lifecycle::State & previous_state) {

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DemoHardware::on_activate(
    const rclcpp_lifecycle::State & previous_state) {

    // Here you should do any pre run-time actions

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DemoHardware::on_deactivate(
    const rclcpp_lifecycle::State & previous_state) {

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DemoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface("front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &l_wheel_1_state_vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface("back_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &l_wheel_2_state_vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &r_wheel_1_state_vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface("back_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &r_wheel_2_state_vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface("front_left_wheel_joint", hardware_interface::HW_IF_POSITION, &l_wheel_1_state_pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface("back_left_wheel_joint", hardware_interface::HW_IF_POSITION, &l_wheel_2_state_pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface("front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &r_wheel_1_state_pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface("back_right_wheel_joint", hardware_interface::HW_IF_POSITION, &r_wheel_2_state_pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DemoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface("front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &l_wheel_1_cmd_vel));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("back_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &l_wheel_2_cmd_vel));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &r_wheel_1_cmd_vel));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("back_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &r_wheel_2_cmd_vel));

  return command_interfaces;
}


hardware_interface::return_type DemoHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{

  return hardware_interface::return_type::OK;
 
}

hardware_interface::return_type DemoHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{

  // Set the wheel velocities to directly match what is commanded

  l_wheel_1_state_vel = l_wheel_1_state_vel;
  l_wheel_2_state_vel = l_wheel_2_state_vel;
  r_wheel_1_state_vel = r_wheel_1_state_vel;
  r_wheel_2_state_vel = r_wheel_2_state_vel;


  return hardware_interface::return_type::OK;
}
}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  demo_hardware::DemoHardware,
  hardware_interface::SystemInterface
)
```
3. Overwrite the package's `CMakeLists.txt` to match the following:
```
cmake_minimum_required(VERSION 3.8)
project(demo_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(demo_hardware SHARED src/demo_hardware_interface.cpp)

target_include_directories(
  demo_hardware
  PRIVATE
  include
)
ament_target_dependencies(
  demo_hardware
  ${INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface demo_hardware.xml)

install(
  TARGETS demo_hardware
  DESTINATION lib
)

ament_export_libraries(
  demo_hardware
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

ament_package()
```
4. Finally, in this package, create `<name>_hardware.xml` with the following contents:
```
<library path="demo_hardware">
    <class name="demo_hardware/DemoHardware"
    type="demo_hardware::DemoHardware"
    base_class_type="hardware_interface::SystemInterface">
        <description>
            An optional plugin description
        </description>
    </class>
</library>
```
5. Now in the urdf we created in part 1, add the following to the bottom (but still inside the `robot` tag):
```
<ros2_control name="DemoBot" type="system">
        <hardware>
            <plugin>demo_hardware/DemoHardware</plugin>

            <!-- You can put other hardware parameters here -->
        </hardware>

        <joint name="front_left_wheel_joint">
            <command_interface name="velocity"/>
            <state_interface name="velocity"/>
        </joint>

        <joint name="back_left_wheel_joint">
            <command_interface name="velocity"/>
            <state_interface name="velocity"/>
        </joint>

        <joint name="front_right_wheel_joint">
            <command_interface name="velocity"/>
            <state_interface name="velocity"/>
        </joint>

        <joint name="back_right_wheel_joint">
            <command_interface name="velocity"/>
            <state_interface name="velocity"/>
        </joint>
    </ros2_control>
```
6. Create a package called `<name>_bringup` as we did before. This package should have a launch and config file. At this point you should also add those folders to the `CMakeLists.txt` as we did before.
7. In the config folder create a `.yaml` file with the following contents:
```
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    diff_controller:
      type: diff_drive_controller/DiffDriveController

    joint_broad:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_controller:
  ros__parameters:
    publish_rate: 50.0
    left_wheel_names: ["front_left_wheel_joint", "back_left_wheel_joint"]
    right_wheel_names: ["front_right_wheel_joint", "back_right_wheel_joint"]
    wheel_separation: 0.3
    wheel_radius: 0.05
    base_frame_id: base_link
    use_stamped_vel: false
```
8. Now in the `launch` folder create a launch file with the following contents:
```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF via xacro
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('demo_description'), 'launch', 'rsp.launch.py'
        )])
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("demo_bringup"),
            "config",
            "ros2_control.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        rsp,
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
```
9. Now after building and sourcing you should be able to see the hardware interface and controllers successfully activating. If you get an error saying that the hardware interface doesn't exist, try deleteing the build and install folders and rebuilding.