from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='clearpath_playpen.sdf',
                          description='Ignition World file'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          description='Use sim time if true'),
]

def generate_launch_description():
    # Launch args
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Set resource paths for meshes
    pkg_husky_description = get_package_share_directory('husky_description')
    pkg_ur_description = get_package_share_directory('ur_description')
    
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.dirname(pkg_husky_description),
            ':',
            pkg_husky_description,
            ':',
            os.path.dirname(pkg_ur_description),
            ':',
            pkg_ur_description,
            ':',
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ]
    )

    # Paths
    pkg_husky_gazebo = get_package_share_directory('husky_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    config_husky_ur3_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "husky_ur3_control.yaml"]
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
            "ur_type:=ur3",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "laser_enabled:=true",
            " ",
            "gazebo_controllers:=",
            config_husky_ur3_controller,
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
            'gz_args': [PathJoinSubstitution([
                pkg_husky_gazebo,
                'worlds',
                world
            ]), ' -r']
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'husky_ur3',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen',
    )

    # Bridge for clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Bridge for IMU - corrected syntax
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU'
        ],
        output='screen'
    )

    # Bridge for GPS - corrected syntax
    gps_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/gps/data@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat'
        ],
        output='screen'
    )

    # Bridge for Laser Scan
    laser_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    # Joint state broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Husky velocity controller
    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
        remappings=[('/husky_velocity_controller/odom', '/odom')],
    )

    # UR3 arm controller
    spawn_ur_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_joint_trajectory_controller', '-c', '/controller_manager'],
        output='screen',
    )

    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # Controller spawning sequence
    joint_state_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )

    husky_velocity_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_husky_velocity_controller,
            on_exit=[spawn_ur_controller],
        )
    )

    ur_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_ur_controller,
            on_exit=[spawn_gripper_controller],
        )
    )

    # Launch husky_control/control.launch.py
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("husky_control"), 'launch', 'control.launch.py']))
    )

    # Launch husky_control/teleop_base.launch.py
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py']))
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(gz_sim)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(clock_bridge)
    ld.add_action(imu_bridge)
    ld.add_action(gps_bridge)
    ld.add_action(laser_bridge)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(joint_state_callback)
    ld.add_action(husky_velocity_callback)
    ld.add_action(ur_controller_callback)
    ld.add_action(launch_husky_control)
    ld.add_action(launch_husky_teleop_base)

    return ld