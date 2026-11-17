"""실물 B안 로봇을 띄운다: robot_state_publisher + 구동 노드 + RPLIDAR C1 + USB 카메라 (+ RViz).

    ros2 launch plant_observer_bringup robot.launch.py
    ros2 launch plant_observer_bringup robot.launch.py camera:=false             # 라이다·구동만
    ros2 launch plant_observer_bringup robot.launch.py lidar:=false camera:=false # 바퀴만 시험
    ros2 launch plant_observer_bringup robot.launch.py upload:=false             # GitHub 자동 업로드 끄기

토픽 이름은 Gazebo와 같다(/cmd_vel, /odom, /scan, /camera/image_raw). 지도는
    ros2 launch plant_observer_gazebo slam.launch.py use_sim_time:=false
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('plant_observer_bringup')

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('plant_observer_description'), 'launch', 'description.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    base_driver = Node(
        package='plant_observer_bringup',
        executable='base_driver',
        name='base_driver',
        parameters=[PathJoinSubstitution([pkg, 'config', 'base_driver.yaml']),
                    {'port': LaunchConfiguration('openrb_port')}],
        # 배터리를 늦게 켜거나 전압 확인에 실패해 죽어도 다시 띄워 붙게 한다 (onboard.launch.py에서 켬)
        respawn=LaunchConfiguration('respawn'),
        respawn_delay=5.0,
        output='screen',
    )

    # sllidar_ros2의 C1 런치를 그대로 쓰고 포트·프레임만 맞춘다 (끌 때는 Ctrl-C로 모터 정지 확인)
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py'])),
        launch_arguments={'serial_port': LaunchConfiguration('lidar_port'), 'frame_id': 'laser'}.items(),
        condition=IfCondition(LaunchConfiguration('lidar')),
    )
    
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[PathJoinSubstitution(
            [FindPackageShare('plant_observer_gazebo'), 'config', 'scan_filter.yaml'])],
        condition=IfCondition(LaunchConfiguration('lidar')),
        output='screen',
    )

    # usb_cam 0.8은 /dev/v4l/by-id/ 링크를 '/dev/' + 상대 링크(../../video32)로 잘못 풀어 열지 못한다.
    # 그래서 실제 장치 경로로 바꿔서 넘긴다
    camera = OpaqueFunction(function=lambda context: [Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',
        parameters=[{
            'video_device': os.path.realpath(LaunchConfiguration('video_device').perform(context)),
            'frame_id': 'camera_optical_frame',
            'camera_name': 'u20cam',
            'pixel_format': 'mjpeg2rgb',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            # usb_cam 기본값 50은 U20CAM(-64~64, 기본 0)에서 너무 밝아 허옇게 나온다
            'brightness': int(LaunchConfiguration('camera_brightness').perform(context)),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('camera')),
    )])

    # 끝난 순찰 회차를 30초마다 팀 GitHub 저장소(~/plant_ws/upload_repo)에 올린다. ROS 노드가 아니라
    # 그냥 프로그램이고, 저장소가 없거나 push가 실패해도 다른 노드에는 영향이 없다
    uploader = Node(
        package='plant_observer_bringup',
        executable='capture_uploader',
        output='screen',
        condition=IfCondition(LaunchConfiguration('upload')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('plant_observer_gazebo'), 'rviz', 'sim.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('openrb_port', default_value='/dev/openrb'),
        DeclareLaunchArgument('respawn', default_value='false',
                              description='구동 노드가 죽으면 5초 뒤 다시 띄움'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('camera', default_value='true'),
        # /dev/video0~31은 노트북 내장 카메라(Intel IPU)라 U20CAM은 번호가 뒤로 밀린다. 고정 경로로 연다
        DeclareLaunchArgument('video_device',
                              default_value='/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0',
                              description='다른 카메라는 ls /dev/v4l/by-id/ 로 확인'),
        DeclareLaunchArgument('camera_brightness', default_value='25',
                              description='U20CAM 밝기 -64~64 (카메라 기본 0)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('upload', default_value='true',
                              description='끝난 순찰 회차를 GitHub에 자동으로 올림'),
        description,
        base_driver,
        lidar,
        scan_filter,
        camera,
        uploader,
        rviz,
    ])
