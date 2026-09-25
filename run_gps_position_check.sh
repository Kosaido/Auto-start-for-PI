#!/bin/bash
# Sources ROS 2 + your workspace, then runs the GPS position check
# for THIS drone. EDIT the paths and --own/--topic flags below to
# match your actual setup.
source /opt/ros/humble/setup.bash
source /home/pi/ros2_ws/install/setup.bash
exec python3 /home/pi/gps_position_check.py --own 0 --topic /fmu/out/vehicle_global_position
