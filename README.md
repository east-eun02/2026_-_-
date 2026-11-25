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
git clone https://github.com/east-eun02/2026_-_-.git ~/plant_ws
cd ~/plant_ws
bash setup_robot_linux.sh --with-gazebo   # 라이다 드라이버(sllidar_ros2)도 자동으로 받아 빌드합니다
```

설치가 끝나면 재부팅합니다. 이후 새 터미널에서 `plant` 명령으로 작업공간을 활성화합니다.

YOLO 가중치는 저장소에 포함하지 않습니다. `~/plant_ws/models/`에 다음 두 파일을 넣어 주세요.

- `yolo11n-seg.pt`: Ultralytics 공식 가중치 (정면 비교 실패 시 식물 영역 보조)
- `apple_yolo26n_aug_v1.pt`: 팀에서 학습한 사과 검출 모델 (`normal_apple`/`abnormal_apple`, 열매 생성·손상·탈락 판정). 없으면 열매는 색 규칙으로 판정합니다.

## 실행

```bash
# 시뮬레이션 (map:=greenhouse | demo ...)
ros2 launch plant_observer_gazebo sim.launch.py map:=demo

# 실물 로봇: 로봇 + Nav2 (기본 지도 maps/demo_real3.yaml). review:=true면 관찰 기록 화면도 같이
ros2 launch plant_observer_bringup onboard.launch.py

# 순찰 시작 (다른 터미널)
ros2 run plant_observer_bringup patrol_manager --ros-args -p route:=demo_real3
```

### 휴대폰으로 운용 (노트북을 차체에 올렸을 때)

관찰 기록 화면(`http://<노트북 IP>:8080`) 맨 위의 **로봇 제어**에서 로봇 켜기·끄기, 순찰 시작·정지, 출발점 복귀, 노트북 끄기를 합니다. 세 가지 systemd 사용자 서비스로 돌아갑니다.

| 서비스 | 내용 |
|---|---|
| `plant-review` | 관찰 기록 화면 + 로봇 제어 버튼. 부팅하면 항상 켜짐 |
| `plant-onboard` | 로봇 + Nav2 (`onboard.launch.py`). 화면의 '로봇 켜기'로 켬 |
| `plant-patrol` | 순찰 한 번. 로봇이 켜져 있을 때만 시작되고, 로봇을 끄면 먼저 멈춤 |

```bash
src/plant_observer_bringup/service/install_onboard_service.sh --enable   # 부팅 시 화면 자동 시작 (로그인 불필요)
src/plant_observer_bringup/service/install_onboard_service.sh --disable  # 선 연결해서 터미널로 개발할 때
journalctl --user -u plant-onboard -f                                    # 로봇 로그
```

`~/.config/plant_observer/panel_pin` 파일에 숫자를 적어 두면 제어 버튼에 PIN을 묻습니다. 휴대폰 핫스팟처럼 믿을 수 있는 네트워크에서만 쓰세요.

순찰 결과 업로드(`capture_uploader`)는 별도의 결과 저장소를 `~/plant_ws/upload_repo`에 clone해 두었을 때만 동작합니다.

OpenRB-150 펌웨어는 `src/plant_observer_bringup/firmware/flash_openrb.sh`로 올립니다(`--with-arduino` 옵션으로 도구를 먼저 설치).

## 테스트

```bash
cd ~/plant_ws/src/plant_observer_bringup && python3 -m pytest test
```
