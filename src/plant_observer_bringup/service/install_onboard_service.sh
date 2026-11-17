#!/usr/bin/env bash
# plant-onboard.service(부팅 시 자동 시작) 설치와 켜고 끄기.
#
#   ./install_onboard_service.sh            # 설치만 (자동 시작은 아직 안 함)
#   ./install_onboard_service.sh --enable   # 부팅하면 로그인 없이 자동 시작
#   ./install_onboard_service.sh --disable  # 자동 시작 끔 (선 연결해서 터미널로 개발할 때)
#
# 자동 시작을 켠 채로 터미널에서 robot.launch.py 등을 또 띄우면 모터·라이다를 두 프로그램이 잡는다.
# 그때는 먼저  systemctl --user stop plant-onboard
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
UNIT_DIR="$HOME/.config/systemd/user"

mkdir -p "$UNIT_DIR"
ln -sf "$HERE/plant-onboard.service" "$UNIT_DIR/plant-onboard.service"
chmod +x "$HERE/onboard_start.sh"
systemctl --user daemon-reload
echo "설치: $UNIT_DIR/plant-onboard.service"

case "${1:-}" in
  --enable)
    # 사용자 서비스는 원래 로그인해야 뜬다. linger를 켜야 덮개 닫고 부팅해도 뜬다
    loginctl enable-linger "$USER"
    systemctl --user enable plant-onboard
    echo "자동 시작 켬. 지금 바로 띄우려면: systemctl --user start plant-onboard"
    ;;
  --disable)
    systemctl --user disable --now plant-onboard
    echo "자동 시작 끔 (linger는 그대로 둠)"
    ;;
esac
