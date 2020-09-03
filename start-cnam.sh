#!/bin/bash
mkdir -p log
genomixd -v -v  > log/genomixd-`date +"%Y%m%d-%H%M%S"`.log &
tmux \
    new-session "morse run isae simu_ttrk_cibleVisuelle_ros.py" \; \
    split-window -p 66 "script -f -c 'cnam-ros /cnam/CmdPort:=/TTRK/Motion_Controller; bash' log/cnam-ros-`date +"%Y%m%d-%H%M%S"`.log" \; \
    split-window -p 50 "script -f -c \"eltclsh ; ./end.sh\" log/eltclsh-`date +"%Y%m%d-%H%M%S"`.log" \; \
    selectp -t 2

