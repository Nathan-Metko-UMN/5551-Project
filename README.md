Using file structure, and dockerfile layout from https://github.com/ros-controls/ros2_control_demos/blob/master/example_7/CMakeLists.txt

https://classic.gazebosim.org/tutorials/?tut=ros_urdf

https://wiki.ros.org/urdf/Tutorials

https://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo




## Running

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
ros2 launch robot gazebo.launch.py


ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=shapes.sdf

ros2 launch urdf_tutorial display.launch.py model:=/workspaces/5551-Project/robot/discription/udrf/robot.urdf.xacro



## moving the robot
only works with old controlers

move +x at 0.2 m/s while turning at 0.5 rad/s
ros2 topic pub --once /base_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.2, 0.0, 0.5]}"

stop
ros2 topic pub --once /base_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0]}"



ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, 0.3, -0.5, 0.0, 0.2, 0.0]}"

back to zero
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"


# wire

Claude generated wire helpers.
Define points in wire.yaml and a spline will be fitted to those points and shown in gazebo and used for trogetory generation.
