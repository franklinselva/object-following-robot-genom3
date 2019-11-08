#!/bin/bash
mkdir -p log
genomixd -v -v  > log/genomixd-`date +"%Y%m%d-%H%M%S"`.log &
tmux \
    new-session "cd ~/MORSE_Simulations ; morse run isae simu_ttrk_cibleVisuelle_ros.py" \; \
    split-window -p 66 "script -c 'cnam-ros -b /cnam/Cmd:=/TTRK/Motion_Controller; bash' log/cnam-ros-`date +"%Y%m%d-%H%M%S"`.log" \; \
    split-window -p 50 "script -c \"eltclsh -tf; ./end.sh\" log/eltclsh-`date +"%Y%m%d-%H%M%S"`.log" \; \
    selectp -t 2

