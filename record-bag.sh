#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" = --help || "${1:-}" = -h ]]; then
  echo '手动录包：./run-local.sh record'
  echo '请先启动仿真；录制期间保持此终端打开，按 Ctrl+C 停止并保存。'
  echo "输出目录：$PROJECT_DIR/bags"
  echo '普通仿真启动不会录包。本命令不启动或重启巡检任务。'
  exit 0
fi
if [[ "$#" -ne 0 ]]; then
  echo '用法：./run-local.sh record（不需要额外参数）' >&2
  exit 2
fi
if [[ -z "${ROS_HOME:-}" || -z "${ROS_MASTER_URI:-}" ]]; then
  echo '请通过 ./run-local.sh record 启动，以加载本项目的 ROS 环境。' >&2
  exit 1
fi
# Keep simultaneous invocations from replacing the active recorder node.
exec 9>"$ROS_HOME/inspection-record.lock"
if ! flock -n 9; then
  echo '本项目已经在录包，请先在原录包终端按 Ctrl+C 停止。' >&2
  exit 1
fi
python3 - <<'PY'
import os
import socket
import sys
import xmlrpc.client
socket.setdefaulttimeout(2.0)
try:
    result = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/bag_preflight')
    if result[0] != 1:
        raise RuntimeError(result[1])
except Exception:
    sys.exit('无法连接本项目 ROS master，请先在另一个终端启动 ./run-local.sh inspection 或 demo。')
PY
mkdir -p "$PROJECT_DIR/bags"
output="$PROJECT_DIR/bags/inspection-$(date +%Y%m%d-%H%M%S-%N).bag"
topics=(
  /visual_slam/odom
  /map_generator/global_cloud
  /pcl_render_node/cloud
  /grid_map/occupancy
  /grid_map/occupancy_inflate
  /planning/bspline
  /planning/pos_cmd
  /waypoint_generator/waypoints
  /inspection/yaw
  /inspection/planning_metrics
  /inspection/status
  /inspection/events
  /inspection/markers
  /odom_visualization/path
  /odom_visualization/robot
  /ego_planner_node/goal_point
  /ego_planner_node/optimal_list
  /tf
  /tf_static
  /rosout_agg
)
echo "开始手动录包：$output"
echo '按 Ctrl+C 停止录包并等待保存完成；仿真会继续运行。'
exec rosbag record --lz4 --buffsize=256 -O "$output" "${topics[@]}" __name:=inspection_bag_recorder
