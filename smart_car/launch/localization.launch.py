from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # paths
    gazebo_world_path = os.path.join(get_package_share_directory('gazebo_world'), 'smalltown.world')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    smart_car_path = get_package_share_directory('smart_car')
    urdf_file = os.path.join(smart_car_path, 'urdf', 'smartcar.urdf')  

    gazebo_launch_file = os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
    
    ld = LaunchDescription()
    
    # default paths 
    default_rviz_config_path = PathJoinSubstitution([FindPackageShare('smart_car'), 'rviz', 'urdf_config.rviz'])
    default_ekf_config_path = PathJoinSubstitution([FindPackageShare('smart_car'), 'config', 'ekf.yaml'])
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time_param = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }
    
    # launch arguments
    ld.add_action(use_sim_time)
    
    ld.add_action(DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        name="ekf_config", 
        default_value=default_ekf_config_path,
        description='Path to extended kalman filter config'
    ))
    
    # Include Gazebo with world
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'world': gazebo_world_path,
            'extra_gazebo_args': '--ros-args -p publish_rate:=0'  # Disable Gazebo joint state publisher
        }.items()
    ))
    
    # Spawn robot 
    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'smartcar',
            '-file', urdf_file, 
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        parameters=[use_sim_time_param],
        output='screen'
    ))
    
    # robot state publisher 
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_smartcar',
        parameters=[{
            'robot_description': Command(['cat ', urdf_file]), 
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    ))
    
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    ))
    
    # wheel odometry node
    ld.add_action(Node(
        package='smart_car',
        executable='wheel_odometry_node.py',
        parameters=[use_sim_time_param]
    ))
    
    # joint state publisher
    ld.add_action(Node(
        package='smart_car',
        executable='joint_state_publisher.py',
        parameters=[use_sim_time_param]
    ))
    
    # EKF node
    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config'), use_sim_time_param]
    ))
    
    return ld
