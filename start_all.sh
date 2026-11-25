#!/bin/bash
killall -9 gzserver gzclient step_detector_node 2>/dev/null || true
pkill -9 -f wqr_mission_control.py 2>/dev/null || true
pkill -9 -f gazebo.launch.py 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
sleep 1
rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/ros2_* 2>/dev/null || true

source /opt/ros/humble/setup.bash
source ~/wheeled_quadruped_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export GAZEBO_MODEL_DATABASE_URI=""

echo "DISPLAY=$DISPLAY"
ros2 launch wheeled_quadruped_description gazebo.launch.py &
sleep 8
ros2 run wheeled_quadruped_vision step_detector_node &
sleep 1
python3 ~/wheeled_quadruped_ws/src/wheeled_quadruped_description/scripts/wqr_mission_control.py
