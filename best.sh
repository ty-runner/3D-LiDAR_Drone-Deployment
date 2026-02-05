#!/usr/bin/env bash
set -euo pipefail

# Use sudo docker by default, but you can override via: DOCKER=docker ./lidar_system.sh lidar-stack
DOCKER="${DOCKER:-sudo docker}"

# Directory where this script lives
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Paths and config
LIDAR_DIR="${HOME}/lidar_system"   # your lidar_system repo on the host

usage() {
  cat <<EOF
Usage: $0 <command>

Commands:
  lidar-stack  Start lidar container with MAVROS2 + system_launch in background (ROS2-only)
  rviz         Start RViz container using lidar:latest
  down         Stop LiDAR + RViz containers

Notes:
  - ROS2-only:
      * No roscore
      * No ros1_bridge
      * No ROS1 MAVROS
  - MAVROS2 and LiDAR TF/processing run INSIDE the lidar container.
  - DDS is still used internally by ROS2, but no Fast DDS XML or RMW
    env vars are configured here.
EOF
}

start_lidar_stack() {
  echo "[lidar] Removing any existing lidar container..."
  ${DOCKER} rm -f lidar >/dev/null 2>&1 || true

  echo "[lidar] Starting lidar stack in background (ROS2 MAVROS2 + LiDAR, default DDS)..."
  ${DOCKER} run -d --net=host --privileged \
    --name lidar \
    -e DISPLAY="${DISPLAY:-}" \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${LIDAR_DIR}:/root/lidar_system" \
    -v /dev:/dev \
    lidar:latest \
    bash -lc '
      set -e

      # Ensure logging works for ROS2 (MAVROS2)
      export HOME=/root
      mkdir -p /root/.ros/log

      # Source ROS2 (prefer $ROS_DISTRO, fall back to foxy)
      if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
      elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
      fi

      cd /root/lidar_system

      colcon build || true

      source install/setup.bash

      echo "[lidar] Starting MAVROS (ROS2) inside container..."
      # Assumes mavros is installed in this ROS2 env
      ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyPixhawk:921600 \
        > /root/mavros_ros2.log 2>&1 &

      sleep 3

      echo "[lidar] Starting LiDAR ROS2 system (TF + pointcloud + Octomap)..."
      # IMPORTANT: system_launch.py should start:
      #   - rplidar_ros/rplidar_node (or rplidar_composition)
      #   - servo_control/dynamic_tf_broadcaster (base_link -> laser_frame)
      #   - servo_control/lidar_pointcloud_node
      #   - octomap_server
      ros2 launch rplidar_ros system_launch.py
    '

  echo "[lidar] Container 'lidar' started with MAVROS2 + LiDAR stack."
}

start_rviz() {
  # Allow root in container to use X11
  if command -v xhost >/dev/null 2>&1; then
    echo "[rviz] Running xhost +local:root for X11 permissions..."
    xhost +local:root || true
  else
    echo "[rviz] WARNING: xhost not found. If RViz fails, run: xhost +local:root"
  fi

  echo "[rviz] Removing existing lidar_rviz container if any..."
  ${DOCKER} rm -f lidar_rviz >/dev/null 2>&1 || true

  echo "[rviz] Starting RViz container (foreground)..."
  ${DOCKER} run -it --net=host --privileged \
    --env "DISPLAY=${DISPLAY:-}" \
    --env "QT_X11_NO_MITSHM=1" \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume "${LIDAR_DIR}:/root/lidar_system" \
    -v /dev:/dev \
    --name lidar_rviz \
    lidar:latest \
    bash -lc '
      # Source ROS2
      if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
      elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
      fi

      cd /root/lidar_system
      if [ -f install/setup.bash ]; then
        source install/setup.bash
      fi

      echo "[rviz] Launching rviz2..."
      rviz2
    '
  echo "[rviz] RViz container exited."
}

down_all() {
  echo "[down] Stopping lidar, lidar_rviz..."
  ${DOCKER} rm -f lidar >/dev/null 2>&1 || true
  ${DOCKER} rm -f lidar_rviz >/dev/null 2>&1 || true
  echo "[down] All containers stopped."
}

cmd="${1:-}"

case "$cmd" in
  lidar-stack)
    start_lidar_stack
    ;;
  rviz)
    start_rviz
    ;;
  down)
    down_all
    ;;
  ""|-h|--help|help)
    usage
    ;;
  *)
    echo "Unknown command: $cmd"
    usage
    exit 1
    ;;
esac




