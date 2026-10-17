import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def launch_setup(context, *args, **kwargs):
    desc_pkg_share = get_package_share_directory('torq_r1_description')
    gazebo_pkg_share = get_package_share_directory('torq_r1_gazebo')
    navigation_pkg_share = get_package_share_directory('torq_r1_navigation')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_name = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    use_rviz = LaunchConfiguration('rviz').perform(context)
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    z_pose = LaunchConfiguration('z_pose').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    # Resolve world path
    if not world_name.endswith('.sdf'):
        world_name += '.sdf'
    world_path = os.path.join(gazebo_pkg_share, 'worlds', world_name)

    # URDF via xacro (TorQ-R1 Differential Drive)
    xacro_file = os.path.join(desc_pkg_share, 'urdf', 'torq_r1.urdf.xacro')
    robot_description_cmd = Command(['xacro ', xacro_file])
    robot_description_content = robot_description_cmd.perform(context)

    # Resource paths for Gazebo to find meshes
    install_share = os.path.dirname(desc_pkg_share)
    gz_resource_path = f"{desc_pkg_share}:{install_share}:{gazebo_pkg_share}"
    current_gz_res = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if current_gz_res:
        gz_resource_path = f"{gz_resource_path}:{current_gz_res}"

    set_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Gazebo Sim Launch
    gz_args = f"-r {world_path}" if gui == 'true' else f"-r -s {world_path}"
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # Spawn robot entity in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'TorQ-R1',
            '-string', robot_description_content,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw
        ]
    )

    # ROS-Gazebo Parameter Bridge
    bridge_config = os.path.join(gazebo_pkg_share, 'config', 'ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True
        }]
    )

    # SLAM Toolbox Node (Asynchronous 2D Online Mapping)
    slam_params_file = os.path.join(navigation_pkg_share, 'config', 'slam_toolbox_params.yaml')
    slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
            {
                'use_lifecycle_manager': False,
                'use_sim_time': True
            }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    # SLAM Auto Configure & Activate Events
    slam_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    slam_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[SLAM AutoStart] SLAM Toolbox activating automatically...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    # RViz2 Node with SLAM layout
    rviz_config = os.path.join(gazebo_pkg_share, 'rviz', 'torq_r1_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    # Path Publisher Node (draws clean trajectory line without arrow clutter)
    path_publisher_node = Node(
        package='torq_r1_navigation',
        executable='path_publisher_node',
        name='path_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return [
        set_env,
        robot_state_publisher_node,
        gz_sim,
        spawn_robot,
        ros_gz_bridge,
        slam_toolbox_node,
        slam_configure_event,
        slam_activate_event,
        path_publisher_node,
        rviz_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='maze_world', description='World name (maze_world, obstacle_world)'),
        DeclareLaunchArgument('gui', default_value='true', description='Set to false for headless simulation'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz2 for visualization'),
        DeclareLaunchArgument('x_pose', default_value='-5.5', description='Initial spawn X coordinate'),
        DeclareLaunchArgument('y_pose', default_value='-5.5', description='Initial spawn Y coordinate'),
        DeclareLaunchArgument('z_pose', default_value='0.05', description='Initial spawn Z coordinate'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial spawn Yaw rotation'),
        OpaqueFunction(function=launch_setup)
    ])
