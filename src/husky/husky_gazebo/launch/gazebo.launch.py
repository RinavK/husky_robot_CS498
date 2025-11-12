from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, SetParameter
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='empty.sdf',
                          description='Ignition World file'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          description='Use sim time if true'),
]

def generate_launch_description():
    # Launch args
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Set use_sim_time globally
    set_use_sim_time = SetParameter(name='use_sim_time', value=True)

    pkg_husky_description = get_package_share_directory('husky_description')
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.dirname(pkg_husky_description),
            ':',
            pkg_husky_description,
            ':',
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ]
    )

    # Paths
    pkg_husky_gazebo = get_package_share_directory('husky_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "control.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky_ur3.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_husky_velocity_controller,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    # Ignition Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                pkg_husky_gazebo,
                'worlds',
                world
            ])
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'husky',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Bridge for clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Joint state broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Velocity controller
    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # UR joint trajectory controller
    spawn_ur_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_joint_trajectory_controller', '-c', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '-c', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )

    # Make sure UR controller starts after velocity controller
    ur_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_husky_velocity_controller,
            on_exit=[spawn_ur_joint_trajectory_controller],
        )
    )

    gripper_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_husky_velocity_controller,
            on_exit=[spawn_gripper_controller],
        )
    )

    # Launch husky_control/control.launch.py
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Launch husky_control/teleop_base.launch.py
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(clock_bridge)
    ld.add_action(ign_resource_path)
    ld.add_action(set_use_sim_time)
    ld.add_action(gz_sim)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(diffdrive_controller_spawn_callback)
    ld.add_action(spawn_gripper_controller)
    ld.add_action(ur_controller_spawn_callback)
    ld.add_action(launch_husky_control)
    ld.add_action(launch_husky_teleop_base)

    return ld