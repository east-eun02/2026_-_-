#!/usr/bin/env bash
# plant-patrol.service가 부르는 시작 스크립트. 관찰 기록 화면이 $XDG_RUNTIME_DIR/plant-patrol.env에
# PATROL_ROUTE·PATROL_LAPS·PATROL_MODE(patrol | home)를 써 두고 서비스를 시작한다.

WS="${PLANT_WS:-$HOME/plant_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"

source /opt/ros/jazzy/setup.bash
source "$WS/install/local_setup.bash"

args=(-p "route:=${PATROL_ROUTE:-demo_real3}" -p "laps:=${PATROL_LAPS:-1}")
[ "${PATROL_MODE:-patrol}" = home ] && args+=(-p home_only:=true)

# ros2 run을 거치지 않고 실행 파일을 직접 exec해야 systemd의 정지 신호가 patrol_manager에 바로 가서
# 진행 중인 이동을 취소한다
exec "$WS/install/plant_observer_bringup/lib/plant_observer_bringup/patrol_manager" --ros-args "${args[@]}"
