from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # paths
    gazebo_world_path = os.path.join(get_package_share_directory('gazebo_world'), 'smalltown.world')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    smartcar_path = get_package_share_directory('smart_car')
    urdf_file = os.path.join(smartcar_path, 'urdf', 'smartcar.urdf')

    gazebo_launch_file = os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Common param for nodes
    use_sim_time_param = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'world': gazebo_world_path,
            'extra_gazebo_args': '--ros-args -p publish_rate:=0'  # Disable the Gazebo joint state publisher
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_smartcar',
        arguments=[
            '-entity', 'smartcar',
            '-file', urdf_file,
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        parameters=[use_sim_time_param],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    joint_state_publisher = Node(
        package='smart_car',
        executable='joint_state_publisher.py',
        name='joint_state_publisher',
        parameters=[use_sim_time_param],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        gazebo_launch,
        spawn_robot,
        robot_state_publisher,
        joint_state_publisher
    ])
