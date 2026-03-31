import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_sirius_description = get_package_share_directory('sirius_description')
    pkg_sirius_gazebo = get_package_share_directory('sirius_gazebo')

    xacro_file = os.path.join(pkg_sirius_description, 'robots', 'sirius.urdf.xacro')
    controllers_file = os.path.join(pkg_sirius_description, 'config', 'controllers.yaml')
    bridge_config_file = os.path.join(pkg_sirius_gazebo, 'config', 'bridge_config.yaml')

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    description_package_parent = os.path.dirname(pkg_sirius_description)
    gazebo_models_path = os.path.join(pkg_sirius_gazebo, 'models')   

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + description_package_parent
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + gazebo_models_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = description_package_parent
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + gazebo_models_path

    manipulator_arg = LaunchConfiguration('manipulator', default='none')

    world_arg = LaunchConfiguration('world')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='world.sdf',
        description='World file name (from sirius_gazebo/worlds)'
    )

    world_path = PathJoinSubstitution([
        pkg_sirius_gazebo,
        'worlds',
        world_arg
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items(),
    )
    
    robot_description_raw = Command([
        'xacro ', xacro_file, 
        ' manipulator:=', manipulator_arg,
        # Jeśli potrzebujesz przekazać więcej argumentów do URDF:
        # ' datum:="[50.8, 20.6, 144.4]"'
    ])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
        }],
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'sirius',
                   '-allow_renaming', 'true',
                   '-z', '0.5'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', 
            '--controller-manager', '/controller_manager'
        ]
    )

    sirius_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'sirius_base_controller', 
            '--controller-manager', '/controller_manager'
        ],
        remappings=[
            ('/sirius_base_controller/cmd_vel', '/cmd_vel'),
            ('/sirius_base_controller/odom', '/odom'),
        ]
    )

    return LaunchDescription([      
        declare_world_arg,  
        node_robot_state_publisher,
        gz_sim,
        gz_spawn_entity,
        bridge,
        TimerAction(
            period=5.0,
            actions=[joint_state_broadcaster_spawner, sirius_base_controller_spawner]
        )
    ])