#!/bin/bash
# Load ROS environment
source /opt/ros/melodic/setup.bash
source /home/agilex/agilex_ws/devel/setup.bash

# Start limo startup in a new terminal
gnome-terminal -- bash -c "roslaunch limo_bringup limo_startup.launch; exec bash"

# Start status monitor in another new terminal
gnome-terminal -- bash -c "python3 /home/agilex/Desktop/limo_status_monitor.py; exec bash"

