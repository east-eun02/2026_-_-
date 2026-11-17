# plant_observer — 식물 관찰 순찰 로봇

ROS 2 Jazzy(Ubuntu 24.04) 기반 식물 관찰 로봇입니다. RPLIDAR C1 + Nav2로 지정한 지점을 순찰하고, USB 카메라로 화분을 찍어 이전 순찰과 비교해 변화를 찾습니다. Gazebo 시뮬레이션과 실물 로봇 모두 지원합니다.

하드웨어: 노트북(차체 탑재 PC), RPLIDAR C1, OpenRB-150 + Dynamixel XL430, USB UVC 카메라

## 폴더 구성

| 경로 | 내용 |
|---|---|
| `src/plant_observer_bringup` | 실물 로봇 실행: 모터 드라이버, 순찰 관리, 촬영·변화 비교, 결과 업로드, 리뷰 서버, 자동 시작 서비스 |
| `src/plant_observer_description` | 로봇 URDF(xacro) |
| `src/plant_observer_gazebo` | Gazebo 월드, Nav2·SLAM 설정, 순찰 경로(`patrol_*.yaml`) |
| `maps/` | 저장된 지도 (실물 최신: `demo_real3`) |
| `setup_robot_linux.sh` | 개발 환경 한 번에 설치 (ROS 패키지, udev 규칙, 드라이버 빌드) |

## 설치

ROS 2 Jazzy가 설치된 Ubuntu 24.04에서:

```bash
git clone <이 저장소 주소> ~/plant_ws
cd ~/plant_ws
bash setup_robot_linux.sh --with-gazebo   # 라이다 드라이버(sllidar_ros2)도 자동으로 받아 빌드합니다
```

설치가 끝나면 재부팅합니다. 이후 새 터미널에서 `plant` 명령으로 작업공간을 활성화합니다.

YOLO 가중치는 저장소에 포함하지 않습니다. `~/plant_ws/models/yolo11n-seg.pt`에 Ultralytics 공식 가중치를 넣어 주세요.

## 실행

```bash
# 시뮬레이션 (map:=greenhouse | demo ...)
ros2 launch plant_observer_gazebo sim.launch.py map:=demo

# 실물 로봇: 로봇 + Nav2 + 관찰 기록 화면 (기본 지도 maps/demo_real3.yaml)
ros2 launch plant_observer_bringup onboard.launch.py

# 순찰 시작 (다른 터미널)
ros2 run plant_observer_bringup patrol_manager --ros-args -p route:=demo_real3
```

순찰 결과 업로드(`capture_uploader`)는 별도의 결과 저장소를 `~/plant_ws/upload_repo`에 clone해 두었을 때만 동작합니다.

OpenRB-150 펌웨어는 `src/plant_observer_bringup/firmware/flash_openrb.sh`로 올립니다(`--with-arduino` 옵션으로 도구를 먼저 설치).

## 테스트

```bash
cd ~/plant_ws/src/plant_observer_bringup && python3 -m pytest test
```
