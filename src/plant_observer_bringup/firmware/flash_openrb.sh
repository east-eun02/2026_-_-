#!/usr/bin/env bash
# OpenRB-150에 ROBOTIS 공식 usb_to_dynamixel(USB 패스스루) 펌웨어를 올린다.
# 노트북의 Dynamixel SDK가 USB로 보낸 패킷을 그대로 XL430(TTL)에 전달하고,
# 모터 통신 속도는 노트북이 연 USB 포트 속도(57600)를 따른다.
#
#   bash flash_openrb.sh              # /dev/openrb
#   bash flash_openrb.sh /dev/ttyACM0
set -euo pipefail
PORT=${1:-/dev/openrb}
FQBN=OpenRB-150:samd:OpenRB-150
CLI=$(command -v arduino-cli || echo "$HOME/.local/bin/arduino-cli")
SKETCH=$(ls -d "$HOME"/.arduino15/packages/OpenRB-150/hardware/samd/*/libraries/OpenRB-150/examples/usb_to_dynamixel | tail -1)

[ -x "$CLI" ] || { echo "arduino-cli가 없습니다: bash setup_robot_linux.sh --with-arduino" >&2; exit 1; }
[ -d "$SKETCH" ] || { echo "OpenRB-150 보드 패키지 예제를 찾지 못했습니다." >&2; exit 1; }
[ -e "$PORT" ] || { echo "$PORT 가 없습니다. OpenRB를 USB로 연결했는지 확인하세요." >&2; exit 1; }

# /dev/openrb 는 심볼릭 링크라 업로더가 리셋 후 포트를 다시 찾도록 실제 장치 경로를 넘긴다
REAL_PORT=$(readlink -f "$PORT")
"$CLI" compile --fqbn "$FQBN" "$SKETCH"
"$CLI" upload --fqbn "$FQBN" -p "$REAL_PORT" "$SKETCH"
echo "업로드 완료. 이제 ros2 launch plant_observer_bringup robot.launch.py"
