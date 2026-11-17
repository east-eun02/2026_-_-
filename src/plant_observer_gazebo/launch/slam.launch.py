"""slam_toolbox로 지도를 만든다. Gazebo에서는 use_sim_time:=true(기본), 실물에서는 false.

    ros2 launch plant_observer_gazebo slam.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': PathJoinSubstitution(
                [FindPackageShare('plant_observer_gazebo'), 'config', 'slam_sim.yaml']),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam,
    ])
