#!/usr/bin/env bash
# plant-onboard.service가 부르는 시작 스크립트: ROS 환경을 잡고 onboard.launch.py를 띄운다.
# 뒤에 붙인 인자는 launch 인자로 넘어간다 (예: onboard_start.sh camera:=false)

WS="${PLANT_WS:-$HOME/plant_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"

# 부팅 직후 USB 장치가 늦게 잡힐 수 있어 최대 60초 기다린다.
# 그래도 없으면 실패로 끝내고 systemd가 10초 뒤 다시 시도한다. 장치 없이 시험할 때는 PLANT_DEVICES=
for dev in ${PLANT_DEVICES-/dev/openrb /dev/rplidar}; do
  for _ in $(seq 60); do [ -e "$dev" ] && break; sleep 1; done
  if [ ! -e "$dev" ]; then
    echo "$dev 장치가 없습니다. USB 허브 연결을 확인하세요" >&2
    exit 1
  fi
done

source /opt/ros/jazzy/setup.bash
source "$WS/install/local_setup.bash"

# exec로 바꿔서 systemd가 보내는 SIGINT가 ros2 launch에 바로 가게 한다 (Ctrl-C와 같은 순서로 꺼짐)
exec ros2 launch plant_observer_bringup onboard.launch.py "$@"
