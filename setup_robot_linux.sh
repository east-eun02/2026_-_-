#!/usr/bin/env bash
# 식물관찰 로봇 B안 — Ubuntu 24.04 + ROS 2 Jazzy 개발 환경 설정
# 대상: Galaxy Book5 Pro 14 (차체 탑재 PC) · RPLIDAR C1 · OpenRB-150 + XL430 · USB UVC 카메라
#
# 사용법 (데스크톱 터미널에서, sudo 없이):
#   bash setup_robot_linux.sh                 # 필수 설정
#   bash setup_robot_linux.sh --with-arduino  # + OpenRB 펌웨어 도구
#   bash setup_robot_linux.sh --with-sim      # + MuJoCo 시뮬레이션 venv
#   bash setup_robot_linux.sh --with-gazebo   # + ros_gz 메타패키지·데모 (Gazebo 본체는 nav2_bringup과 함께 설치됨)
# 여러 번 실행해도 됩니다. 끝나면 재부팅하세요.
set -euo pipefail

WS=${WS:-$HOME/plant_ws}
WITH_GAZEBO=0
WITH_ARDUINO=0
WITH_SIM=0
for arg in "$@"; do
  case $arg in
    --with-gazebo) WITH_GAZEBO=1 ;;
    --with-arduino) WITH_ARDUINO=1 ;;
    --with-sim) WITH_SIM=1 ;;
    *) echo "알 수 없는 옵션: $arg" >&2; exit 1 ;;
  esac
done

step() { printf '\n\033[1;32m==> %s\033[0m\n' "$*"; }

[ "$(id -u)" -ne 0 ] || { echo "일반 사용자로 실행하세요. 필요할 때 sudo 비밀번호를 묻습니다." >&2; exit 1; }
. /etc/os-release
[ "$VERSION_CODENAME" = noble ] || { echo "Ubuntu 24.04(noble) 전용입니다." >&2; exit 1; }
[ -d /opt/ros/jazzy ] || { echo "/opt/ros/jazzy 가 없습니다. ROS 2 Jazzy를 먼저 설치하세요." >&2; exit 1; }

step "1. 시스템·ROS 패키지 최신화"
sudo apt update
sudo apt upgrade -y

step "2. 로봇 개발 패키지 설치"
PKGS=(
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
  ros-jazzy-dynamixel-sdk
  ros-jazzy-usb-cam ros-jazzy-image-transport-plugins
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui
  v4l-utils openssh-server python3-venv
)
if [ "$WITH_GAZEBO" = 1 ]; then
  PKGS+=(ros-jazzy-ros-gz)
fi
sudo apt install -y "${PKGS[@]}"

step "3. rosdep 초기화"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update

step "4. 시리얼 포트 권한 (dialout 그룹)"
if getent group dialout | grep -qw "$USER"; then
  echo "이미 dialout 그룹입니다."
else
  sudo usermod -aG dialout "$USER"
  echo "dialout 그룹에 추가했습니다. 재로그인 후 적용됩니다."
fi

step "5. 장치 고정 이름 (/dev/rplidar, /dev/openrb)"
sudo tee /etc/udev/rules.d/99-plant-robot.rules >/dev/null <<'EOF'
# RPLIDAR C1 USB 어댑터 (CP2102N 10c4:ea60) -> /dev/rplidar
# 같은 칩을 쓰는 다른 USB-시리얼 장치도 이 이름을 받으니 함께 꽂지 않습니다.
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0660", SYMLINK+="rplidar"
# OpenRB-150 (2f5d:2202) -> /dev/openrb, ModemManager가 포트를 건드리지 않게
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{idProduct}=="2202", GROUP="dialout", MODE="0660", SYMLINK+="openrb", ENV{ID_MM_DEVICE_IGNORE}="1"
# 주행 중 USB 절전으로 끊기지 않게
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="10c4", ATTR{idProduct}=="ea60", TEST=="power/control", ATTR{power/control}="on"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2f5d", TEST=="power/control", ATTR{power/control}="on"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb

step "6. 덮개를 닫고 배터리로 달려도 잠들지 않게"
sudo mkdir -p /etc/systemd/logind.conf.d
sudo tee /etc/systemd/logind.conf.d/10-plant-robot-lid.conf >/dev/null <<'EOF'
# 차체 탑재 운용: 덮개 닫힘으로 절전하지 않음. 가방에 넣기 전에는 직접 끄거나 절전합니다.
[Login]
HandleLidSwitch=ignore
HandleLidSwitchExternalPower=ignore
HandleLidSwitchDocked=ignore
EOF
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'

step "7. Wi-Fi 절전 끄기 (원격 RViz·teleop 지연 감소)"
sudo tee /etc/NetworkManager/conf.d/zz-plant-robot-wifi-powersave.conf >/dev/null <<'EOF'
[connection]
wifi.powersave = 2
EOF

step "8. 작업공간 $WS + RPLIDAR C1 드라이버(sllidar_ros2)"
mkdir -p "$WS/src"
if [ ! -d "$WS/src/sllidar_ros2" ]; then
  git clone https://github.com/Slamtec/sllidar_ros2.git "$WS/src/sllidar_ros2"
fi
set +u
. /opt/ros/jazzy/setup.bash
set -u
(cd "$WS" && rosdep install --from-paths src --ignore-src -y -r && colcon build --symlink-install)
if ! grep -q 'alias plant=' ~/.bashrc; then
  cat >> ~/.bashrc <<EOF

# 식물관찰 로봇 작업공간
alias plant="jazzy; source $WS/install/local_setup.bash; echo \"plant_ws is activated!\""
EOF
fi

if [ "$WITH_ARDUINO" = 1 ]; then
  step "9. OpenRB-150 펌웨어 도구 (arduino-cli)"
  OPENRB_URL=https://raw.githubusercontent.com/ROBOTIS-GIT/OpenRB-150/master/package_openrb_index.json
  mkdir -p "$HOME/.local/bin"
  export PATH="$HOME/.local/bin:$PATH"
  if ! command -v arduino-cli >/dev/null; then
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$HOME/.local/bin" sh
  fi
  arduino-cli core update-index --additional-urls "$OPENRB_URL"
  arduino-cli core install arduino:samd OpenRB-150:samd --additional-urls "$OPENRB_URL"
  arduino-cli lib install Dynamixel2Arduino
  echo "보드 FQBN: OpenRB-150:samd:OpenRB-150"
fi

if [ "$WITH_SIM" = 1 ]; then
  step "10. MuJoCo 시뮬레이션 venv (~/venvs/sim)"
  # ROS의 시스템 numpy(1.x)와 섞이지 않게 별도 venv에 설치합니다.
  [ -d "$HOME/venvs/sim" ] || python3 -m venv "$HOME/venvs/sim"
  "$HOME/venvs/sim/bin/pip" install mujoco==3.3.7 numpy pillow
fi

step "완료 — 재부팅하세요 (dialout 그룹·덮개·Wi-Fi 설정 적용)"
cat <<EOF
재부팅 후 확인:
  groups | grep dialout
  ls -l /dev/rplidar /dev/openrb      # 장치를 꽂은 상태
  ls /dev/v4l/by-id/                   # USB 카메라 고정 경로
  plant && ros2 launch sllidar_ros2 view_sllidar_c1_launch.py serial_port:=/dev/rplidar
EOF
