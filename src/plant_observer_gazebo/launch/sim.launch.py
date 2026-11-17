"""가상 온실에 B안 로봇을 띄운다: Gazebo + robot_state_publisher + 스폰 + 토픽 브리지 + RViz.

    ros2 launch plant_observer_gazebo sim.launch.py                # 4m x 3m 온실, 화분 6개
    ros2 launch plant_observer_gazebo sim.launch.py map:=demo      # 2.7m x 1.8m 시연 구역, 화분 2개
    ros2 launch plant_observer_gazebo sim.launch.py headless:=true rviz:=false   # 화면 없이
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('plant_observer_gazebo')
    world = LaunchConfiguration('world')
    map_name = LaunchConfiguration('map')

    # 맵마다 서쪽 벽에서 약 0.4m 떨어진 곳에 동쪽(+x)을 보고 스폰
    spawn_x = {'greenhouse': '-1.6', 'demo': '-0.6'}
    headless = LaunchConfiguration('headless')

    # GUI는 -r(바로 시작), headless는 서버만 + 화면 없는 렌더링(센서용)
    gz_args = PythonExpression([
        "('-s -r -v 2 --headless-rendering ' if '", headless, "' == 'true' else '-r -v 2 ')",
        " + '", world, "'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('plant_observer_description'), 'launch', 'description.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'plant_observer',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.01', '-Y', LaunchConfiguration('yaw')],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([pkg, 'config', 'bridge.yaml']),
                     'use_sim_time': True}],
        output='screen',
    )

    # 라이다가 보는 차체(카메라 기둥)를 지운 /scan_filtered를 만든다
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[PathJoinSubstitution([pkg, 'config', 'scan_filter.yaml']),
                    {'use_sim_time': True}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'sim.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='greenhouse', choices=list(spawn_x)),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([pkg, 'worlds', [map_name, '.sdf']])),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value=PythonExpression([str(spawn_x), "['", map_name, "']"])),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # Ubuntu 기본 Wayland에서 Gazebo·RViz 창이 깨지지 않도록 X11(XWayland)로 띄운다
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        gazebo,
        description,
        spawn,
        bridge,
        scan_filter,
        rviz,
    ])
