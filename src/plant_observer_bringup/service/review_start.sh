#!/usr/bin/env bash
# plant-review.service가 부르는 시작 스크립트: ROS 환경을 잡고 관찰 기록 화면(로봇 제어 포함)을 띄운다.
# 뒤에 붙인 인자는 review_server로 넘어간다 (예: review_start.sh --route demo_real2)

WS="${PLANT_WS:-$HOME/plant_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"

source /opt/ros/jazzy/setup.bash
source "$WS/install/local_setup.bash"

exec "$WS/install/plant_observer_bringup/lib/plant_observer_bringup/review_server" "$@"
