from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import IfElseSubstitution, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # Load parameters
    gripper_description_file = LaunchConfiguration('gripper_description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    world_file = LaunchConfiguration('world_file')
    gazebo_gui = LaunchConfiguration('gazebo_gui')

    # Robot Description from xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        gripper_description_file,
        ' ',
        'use_fake_hardware:=true'
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{"use_sim_time": True}, robot_description],
    )

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments={
            'gz_args': IfElseSubstitution(
                gazebo_gui,
                if_value=[' -r -v 4 ', world_file],
                else_value=[' -s -r -v 4 ', world_file],
            )
        }.items()
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robotiq_gripper',
            '-allow_renaming', 'true'
        ],
    )

    gz_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Only spawn controllers after robot is spawned!
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'robotiq_gripper_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', PathJoinSubstitution([
                FindPackageShare('robotiq_gripper_gazebo'), 'config', 'robotiq_gripper_controller.yaml'
            ])
        ],
        output='screen',
    )

    return [
        gz_launch_description,
        robot_state_publisher_node,
        gz_spawn_entity,
        gz_clock_bridge,
        TimerAction(
            period=5.0,  # Small wait to make sure robot is fully inserted into Gazebo before spawning controllers
            actions=[
                joint_state_broadcaster_spawner,
                gripper_controller_spawner,
            ]
        )
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'gripper_description_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robotiq_gripper_gazebo'), 'urdf', 'robotiq_2f_85_gripper.urdf.xacro'
            ]),
            description='URDF/Xacro file for the Robotiq gripper',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robotiq_gripper_gazebo'), 'config', 'gripper_controller.yaml'
            ]),
            description='YAML file with controllers configuration',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Launch Gazebo with GUI or not',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robotiq_gripper_gazebo"), "worlds", "bullet_world.sdf"]
            ),
            description="Gazebo world file containing a custom world.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


#---------------------------------------------------------------------------------------------------------------------------------------------------------------

# Command to Control the Gripper:

# To Open the Gripper:

"""ros2 topic pub --once /robotiq_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "joint_names:
- 'robotiq_85_left_knuckle_joint'
points:
- positions: [0.0]
  time_from_start: {sec: 1, nanosec: 0}"
"""

# To Close the Gripper

"""ros2 topic pub --once /robotiq_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "joint_names:
- 'robotiq_85_left_knuckle_joint'
points:
- positions: [0.8]
  time_from_start: {sec: 1, nanosec: 0}"
"""