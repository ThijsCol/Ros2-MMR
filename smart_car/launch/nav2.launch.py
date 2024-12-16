from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()
    
    # package paths
    smart_car_path = FindPackageShare('smart_car')
    nav2_bringup_path = FindPackageShare('nav2_bringup')
    
    # default paths
    default_rviz_config_path = PathJoinSubstitution([nav2_bringup_path, 'rviz', 'nav2_default_view.rviz'])
    default_model_path = PathJoinSubstitution([smart_car_path, 'urdf', 'smartcar.urdf'])
    default_world_path = PathJoinSubstitution([FindPackageShare('gazebo_world'), 'smalltown.world'])
    default_ekf_config_path = PathJoinSubstitution([smart_car_path, 'config', 'ekf.yaml'])
    default_map_path = PathJoinSubstitution([smart_car_path, 'map', 'smalltown_world.yaml'])
    
    
    ld.add_action(DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Path to robot urdf file'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        name='world', 
        default_value=default_world_path,
        description='Path to world file'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        name='ekf_config', 
        default_value=default_ekf_config_path,
        description='Path to ekf config file'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        name='map_config', 
        default_value=default_map_path,
        description='Path to map yaml file'
    ))
    
    # static transform publisher map->odom
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    ))
    
    # robot state publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_smartcar',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[LaunchConfiguration('model')]
    ))
    
    # joint state publisher
    ld.add_action(Node(
        package='smart_car',
        executable='joint_state_publisher.py',
        parameters=[{'use_sim_time': True}]
    ))
    
    # wheel odometry node
    ld.add_action(Node(
        package='smart_car',
        executable='wheel_odometry_node.py',
        parameters=[{'use_sim_time': True}]
    ))
    
    # EKF node
    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('ekf_config'),
            {'use_sim_time': True}
        ]
    ))
    
    # Nav2 
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_bringup_path, 'launch', 'bringup_launch.py'])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map_config'),
            'use_sim_time': 'true'
        }.items()
    ))
    
    # Start RViz2
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}]
    ))
    
    # Gazebo
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    ))
    
    # Spawn robot 
    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-entity', 'smartcar', '-file', LaunchConfiguration('model')],
        output='screen'
    ))
    
    return ld
