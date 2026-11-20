import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import AppendEnvironmentVariable



def generate_launch_description():
    world_arg = DeclareLaunchArgument('world', default_value='empty.sdf',
                                      description='Gazebo world file to load')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('robot'), 'discription', 'udrf', 'robot.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('robot')))

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r -v 3 ', LaunchConfiguration('world')]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robot', '-z', '0.0'],
        output='screen',
    )

    bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                #    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                #    '/model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                #    '/model/robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
                  ],
        output='screen',
    )

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('robot'), 'config', 'controllers.yaml'])

    wire_yaml = PathJoinSubstitution([FindPackageShare('robot'), 'config', 'wire.yaml'])


    def spawner(name, *extra):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--param-file', controllers_yaml, '--param-file', wire_yaml,  '--switch-timeout', '30', *extra],
            output='screen',
        )

    joint_state_broadcaster = spawner('joint_state_broadcaster')

    # Start the remaining controllers once the broadcaster is up
    other_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                spawner('robot_controller'),
                spawner('base_controller', '--inactive'),
                spawner('arm_controller', '--inactive'),
            ],
        )
    )

    wire_sdf_file = Command(
        ['ros2 run robot wire_generator --ros-args --params-file ', wire_yaml],
        on_stderr='warn')
    spawn_wire = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', wire_sdf_file, '-name', 'wire'],
        output='screen',
    )



    return LaunchDescription([
        world_arg,
        resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridges,
        joint_state_broadcaster,
        other_controllers,
        spawn_wire,
    ])
