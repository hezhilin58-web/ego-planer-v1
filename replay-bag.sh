#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" = --help || "${1:-}" = -h ]]; then
  echo '回放指定录包：./run-local.sh replay 文件名.bag'
  echo '回放最近的完整录包：./run-local.sh replay'
  echo '自动打开 RViz；空格暂停/继续，Ctrl+C 关闭本次回放。'
  exit 0
fi
if [[ "$#" -gt 1 ]]; then
  echo '用法：./run-local.sh replay [文件名.bag]' >&2
  exit 2
fi
if [[ -z "${ROS_DISTRO:-}" || -z "${ROS_MASTER_URI:-}" ]]; then
  echo '请通过 ./run-local.sh replay 启动。' >&2
  exit 1
fi
if [[ "$#" -eq 0 ]]; then
  bag=$(python3 - "$PROJECT_DIR" <<'PY'
from pathlib import Path
import sys
files = [p for p in (Path(sys.argv[1]) / 'bags').glob('*.bag')
         if not p.name.startswith('diagnostic-')]
if not files:
    sys.exit('bags 目录中没有可回放的录包，请明确指定文件。')
print(max(files, key=lambda p: p.stat().st_mtime))
PY
)
elif [[ -f "$1" ]]; then
  bag=$(realpath -- "$1")
elif [[ -f "$PROJECT_DIR/bags/$1" ]]; then
  bag=$(realpath -- "$PROJECT_DIR/bags/$1")
else
  echo "找不到录包：$1（可以只输入 bags 目录下的文件名）" >&2
  exit 1
fi
if [[ "$bag" != *.bag ]]; then
  echo '请选择正常停止录制后生成的 .bag 文件。' >&2
  exit 1
fi
# Always use an independent loopback master. Never publish a recording to live controls.
port=$(python3 - <<'PY'
import socket
with socket.socket() as s:
    try:
        s.bind(('127.0.0.1', 11322))
    except OSError:
        s.bind(('127.0.0.1', 0))
    print(s.getsockname()[1])
PY
)
export ROS_MASTER_URI="http://127.0.0.1:$port"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export ROS_HOME="$PROJECT_DIR/.ros-replay"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"
printf '%s\n' "$ROS_MASTER_URI" > "$ROS_HOME/last-replay-master.txt"
view_log="$ROS_LOG_DIR/replay-view-$port.log"
echo "回放文件：$bag"
echo "独立回放端口：$port"
echo '正在打开 RViz……'
roslaunch "$PROJECT_DIR/replay-view.launch" > "$view_log" 2>&1 &
view_pid=$!
bag_pid=''
cleanup() {
  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid" 2>/dev/null || true
  fi
  if kill -0 "$view_pid" 2>/dev/null; then
    kill -INT "$view_pid" 2>/dev/null || true
  fi
  wait "$view_pid" 2>/dev/null || true
  if [[ -n "$bag_pid" ]]; then wait "$bag_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if ! python3 - "$view_pid" <<'PY'
import os
import signal
import socket
import sys
import time
import xmlrpc.client
socket.setdefaulttimeout(1.0)
master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
deadline = time.monotonic() + 30
while time.monotonic() < deadline:
    try:
        os.kill(int(sys.argv[1]), 0)
    except ProcessLookupError:
        sys.exit('RViz 启动进程已退出。')
    try:
        code, _, uri = master.lookupNode('/replay_launcher', '/rviz')
        if code == 1 and xmlrpc.client.ServerProxy(uri).getPid('/replay_launcher')[0] == 1:
            break
    except Exception:
        pass
    time.sleep(0.2)
else:
    sys.exit('等待 RViz 就绪超时。')
PY
then
  tail -30 "$view_log" >&2
  exit 1
fi
echo '空格：暂停/继续；暂停后 s：单步；Ctrl+C：停止并关闭本次 RViz。'
echo '播放结束后保留最后画面，按 Ctrl+C 退出。'
rosbag play --clock --keep-alive --delay=2 "$bag" <&0 &
bag_pid=$!
while kill -0 "$bag_pid" 2>/dev/null; do
  if ! kill -0 "$view_pid" 2>/dev/null; then
    echo "RViz 已关闭或异常退出，正在停止回放。日志：$view_log" >&2
    exit 1
  fi
  sleep 0.2
done
wait "$bag_pid"
