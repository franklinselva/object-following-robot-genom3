#!/bin/bash
mkdir -p log
genomixd -v -v  > log/genomixd-`date +"%Y%m%d-%H%M%S"`.log &
tmux \
    new-session "roslaunch leo_gazebo leo_gazebo_brick.launch" \; \
    split-window -p 75 "script -f -c 'cnam-ros /cnam/CmdPort:=/cmd_vel; bash' log/cnam-ros-`date +"%Y%m%d-%H%M%S"`.log" \; \
    split-window -p 66 "rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/brick_cmd_vel" \; \
    split-window -p 50 "script -f -c \"eltclsh ; ./end.sh\" log/eltclsh-`date +"%Y%m%d-%H%M%S"`.log" \; \
    selectp -t 3

