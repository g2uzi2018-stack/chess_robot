#!/usr/bin/env bash
# 同一入口自动准备设备；不会清零参考、回退旧档或重试不确定的设备命令。
set -eo pipefail
cd "$(dirname -- "${BASH_SOURCE[0]}")/../.."
source /opt/ros/humble/setup.bash
source runs/row_lower_1mm_20261105_01/install/setup.bash
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
arm_service=chess-robot-arm-contact-2mm-20261105.service
ensure_arm_service() {
  if [ "$(systemctl --user show "$arm_service" -p LoadState --value)" = "not-found" ]; then
    # 临时 unit 在停止后会消失；使用已核对的同一安装重建，不连接第二个驱动。
    if pgrep -x arm_node >/dev/null; then
      printf '%s\n' '发现其他 arm_node；请先结束其他机械臂控制进程。' >&2
      return 2
    fi
    systemd-run --user --unit="$arm_service" --collect \
      --property=Type=simple --property=Restart=no --working-directory="$PWD" \
      /bin/bash "$PWD/runs/row_lower_1mm_20261105_01/start-arm.sh"
  else
    systemctl --user start "$arm_service"
  fi
}
prepare() {
  ensure_arm_service
  python3 tools/pick_stack/startup.py "$@"
}
case "${1:-}" in
  shutdown)
    # 回 Home 并确认停稳存档后，先停止反馈节点，再由操作者切断机械臂电源。
    # 避免断电抱闸/反馈速度尖峰覆盖已经保存的可信记录。
    prepare before-restart "${@:2}"
    systemctl --user stop "$arm_service"
    python3 - <<'PY'
import hashlib,yaml
from pathlib import Path
content=Path('.runtime/arm-checkpoint.save').read_text()
digest,payload=content.split('\n',1)
if hashlib.sha256(payload.encode()).hexdigest()!=digest or not yaml.safe_load(payload)['restorable']:
    raise SystemExit('停机存档未确认，请勿直接断电；检查机械臂状态')
print('POWER_OFF_READY：停稳记录已保存，机械臂节点已停止，现在可以关闭机械臂电源。')
PY
    ;;
  activate)
    prepare before-restart "${@:2}"
    systemctl --user restart chess-robot-arm-contact-2mm-20261105.service
    # 前一步已确认停稳存档；受控重启期间不允许手动移动设备。
    prepare prepare --no-unobserved-motion
    ;;
  prepare|recover)
    prepare prepare "${@:2}"
    ;;
  row)
    if [ "${2:-}" != "--workspace-ready" ] && [ "${2:-}" != "--workspace-clear" ]; then
      printf '%s\n' '请确认桌面就绪、夹爪无物体、手已移开：row --workspace-ready [--no-unobserved-motion]' >&2
      exit 2
    fi
    prepare prepare "${@:3}"
    exec python3 tools/pick_stack/demo.py       --config tools/pick_stack/demo.row.live.json       --execute "$2"       --authorization 'User launched automatic-preparation row demo with configured occupied-slot policy'       --output "runs/row_speed_$(date +%Y%m%d_%H%M%S_%N)"
    ;;
  scan)
    prepare prepare "${@:2}"
    exec python3 tools/scan_table/scan_controller.py       --config runs/row_lower_1mm_20261105_01/scan.guard2.json       --execute --output "runs/speed_scan_$(date +%Y%m%d_%H%M%S_%N)"
    ;;
  vertical)
    prepare vertical "${@:2}"
    ;;
  *)
    printf '%s\n' 'Usage: run_speed_demo.sh row --workspace-ready [--no-unobserved-motion]'       '       run_speed_demo.sh prepare|recover|activate|scan|vertical|shutdown [--no-unobserved-motion]' >&2
    exit 2
    ;;
esac
