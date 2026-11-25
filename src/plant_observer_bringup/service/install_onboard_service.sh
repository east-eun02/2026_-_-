#!/usr/bin/env bash
# 차체 탑재 운용 서비스 설치와 켜고 끄기.
#   plant-review  : 관찰 기록 화면 + 로봇 제어 버튼 (항상 켜 둠, http://<노트북 IP>:8080)
#   plant-onboard : 로봇 + Nav2 (화면의 '로봇 켜기'로 켬)
#   plant-patrol  : 순찰 한 번 (화면의 '순찰 시작'으로 켬)
#
#   ./install_onboard_service.sh                    # 설치만 (부팅 때 아무것도 안 뜸)
#   ./install_onboard_service.sh --enable           # 부팅하면 로그인 없이 관찰 기록 화면이 뜸. 로봇은 화면에서 켬
#   ./install_onboard_service.sh --autostart-robot  # 위 + 부팅하면 로봇까지 바로 켬
#   ./install_onboard_service.sh --disable          # 자동 시작 모두 끔 (선 연결해서 터미널로 개발할 때)
#
# 로봇이 서비스로 켜진 채로 터미널에서 robot.launch.py 등을 또 띄우면 모터·라이다를 두 프로그램이 잡는다.
# 그때는 먼저  systemctl --user stop plant-onboard
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
UNIT_DIR="$HOME/.config/systemd/user"

mkdir -p "$UNIT_DIR"
for unit in plant-onboard plant-review plant-patrol; do
  ln -sf "$HERE/$unit.service" "$UNIT_DIR/$unit.service"
done
chmod +x "$HERE"/*.sh
systemctl --user daemon-reload
echo "설치: $UNIT_DIR/plant-{onboard,review,patrol}.service"

case "${1:-}" in
  --enable|--autostart-robot)
    # 사용자 서비스는 원래 로그인해야 뜬다. linger를 켜야 덮개 닫고 부팅해도 뜬다
    loginctl enable-linger "$USER"
    systemctl --user enable --now plant-review
    if [ "$1" = --autostart-robot ]; then
      systemctl --user enable plant-onboard
      echo "부팅하면 관찰 기록 화면과 로봇이 함께 켜집니다"
    else
      systemctl --user disable plant-onboard 2>/dev/null || true
      echo "부팅하면 관찰 기록 화면이 켜집니다. 로봇은 화면의 '로봇 켜기'로 켜세요"
    fi
    ;;
  --disable)
    systemctl --user disable --now plant-review plant-onboard
    echo "자동 시작 끔 (linger는 그대로 둠)"
    ;;
esac
