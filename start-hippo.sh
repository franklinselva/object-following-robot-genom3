#!/bin/bash
mkdir -p log
genomixd -v -v  > log/genomixd-`date +"%Y%m%d-%H%M%S"`.log &
tmux \
    new-session "roslaunch leo_gazebo leo_gazebo_brick.launch" \; \
    split-window -p 80 "script -f -c 'CT_robot-hippo-ros -F 1 /CT_robot/CmdPort:=/cmd_vel; bash' log/CT_robot-ros-`date +"%Y%m%d-%H%M%S"`.log" \; \
    split-window -p 75 "script -f -c \"eltclsh -tf; ./end.sh\" log/eltclsh-`date +"%Y%m%d-%H%M%S"`.log" \; \
    split-window -p 66 "rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/brick_cmd_vel" \; \
    selectp -t 2

