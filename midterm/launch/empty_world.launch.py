from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to Gazebo launch
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    # Path to your custom world
    world_file = os.path.join(
        get_package_share_directory('midterm'),
        'worlds',
        'empty_world.world'
    )

    # Path to TurtleBot3 model SDF (not URDF!)
    turtlebot3_model = os.environ['TURTLEBOT3_MODEL']
    turtlebot3_sdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        f'turtlebot3_{turtlebot3_model}',
        'model.sdf'
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', turtlebot3_sdf,
                   '-entity', 'turtlebot3'],
        output='screen'
    )

    return LaunchDescription([
        # Launch Gazebo with your world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn the TurtleBot3
        spawn_entity
    ])

