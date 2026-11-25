"""노트북을 차체에 올려 운용할 때 한 번에 띄운다: 로봇(robot.launch.py) + Nav2(nav2_session).

관찰 기록 화면은 plant-review 서비스로 따로 항상 떠 있고, 거기서 이 런치(plant-onboard)를 켜고 끈다.
서비스 없이 터미널로 띄우면서 화면도 보려면 review:=true.

    ros2 launch plant_observer_bringup onboard.launch.py
    ros2 launch plant_observer_bringup onboard.launch.py review:=true          # 관찰 기록 화면도 같이
    ros2 launch plant_observer_bringup onboard.launch.py map:=$HOME/plant_ws/maps/demo_real2.yaml
    ros2 launch plant_observer_bringup onboard.launch.py rviz:=true               # 선 연결해서 화면 볼 때
    ros2 launch plant_observer_bringup onboard.launch.py nav2:=false              # 로봇만 (= robot.launch.py)

순찰은 시작하지 않고 대기한다. 순찰은 따로
    ros2 run plant_observer_bringup patrol_manager --ros-args -p route:=demo_real3
터미널을 나눠 하나씩 띄우던 방식도 그대로 쓸 수 있다. 같은 노드를 두 방식으로 동시에 띄우지는 않는다.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('plant_observer_bringup'), 'launch', 'robot.launch.py'])),
        launch_arguments={name: LaunchConfiguration(name)
                          for name in ('lidar', 'camera', 'upload', 'rviz', 'respawn')}.items(),
    )

    # nav2_session은 argparse만 받아서 Node(--ros-args가 붙음) 대신 실행 파일을 직접 띄운다.
    # Ctrl-C/SIGTERM을 받으면 이동 취소 → navigation → localization 순으로 끄는 데 수십 초가 걸릴 수 있어
    # launch가 기다려 주는 시간을 늘린다 (기본 5초면 중간에 강제 종료돼 Nav2가 남는다).
    # 구동 노드가 늦게 붙어 90초 안에 Nav2가 활성화되지 못하면 nav2_session이 끝나므로 다시 띄운다
    nav2 = ExecuteProcess(
        name='nav2_session',
        cmd=[PathJoinSubstitution([FindPackagePrefix('plant_observer_bringup'),
                                   'lib', 'plant_observer_bringup', 'nav2_session']),
             '--map', LaunchConfiguration('map')],
        sigterm_timeout='60',
        sigkill_timeout='10',
        respawn=LaunchConfiguration('respawn'),
        respawn_delay=5.0,
        output='screen',
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    review = Node(
        package='plant_observer_bringup',
        executable='review_server',
        arguments=['--port', LaunchConfiguration('review_port')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('review')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/plant_ws/maps/demo_real3.yaml')),
        DeclareLaunchArgument('nav2', default_value='true'),
        DeclareLaunchArgument('review', default_value='false',
                              description='관찰 기록 화면도 띄움 (plant-review 서비스가 떠 있으면 포트가 겹침)'),
        DeclareLaunchArgument('review_port', default_value='8080'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('upload', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('respawn', default_value='true',
                              description='구동 노드·nav2_session이 죽으면 5초 뒤 다시 띄움'),
        robot,
        nav2,
        review,
    ])
