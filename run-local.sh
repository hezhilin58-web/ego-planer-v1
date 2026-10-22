#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Keep ROS 2 and other catkin workspaces out of this project's environment.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH ROSLISP_PACKAGE_DIRECTORIES
unset ROS_HOSTNAME
export ROS_MASTER_URI="${EGO_MASTER_URI:-http://127.0.0.1:11321}"
export ROS_IP="${EGO_ROS_IP:-127.0.0.1}"
source /opt/ros/noetic/setup.bash --extend
export ROS_HOME="$PROJECT_DIR/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"
cd "$PROJECT_DIR"
mode="${1:-demo}"
if [ "$#" -gt 0 ]; then shift; fi
if [ "$mode" = build ]; then
  exec catkin_make --force-cmake -j2 -l2 --cmake-args -C "$PROJECT_DIR/compat/noetic-cache.cmake" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
fi
if [ ! -f devel/setup.bash ]; then
  echo '请先执行 ./run-local.sh build' >&2
  exit 1
fi
source devel/setup.bash --extend
case "$mode" in
  panel) exec python3 "$PROJECT_DIR/web/inspection_panel.py" "$@" ;;
  replay) exec "$PROJECT_DIR/replay-bag.sh" "$@" ;;
  record) exec "$PROJECT_DIR/record-bag.sh" "$@" ;;
  inspection) exec roslaunch ego_planner inspection.launch "$@" ;;
  demo) exec roslaunch "$PROJECT_DIR/local-demo.launch" "$@" ;;
  sim) exec roslaunch ego_planner run_in_sim.launch "$@" ;;
  rviz) exec roslaunch ego_planner rviz.launch "$@" ;;
  *) echo '用法: ./run-local.sh [build|demo|sim|rviz|inspection|panel|record|replay]' >&2; exit 2 ;;
esac
