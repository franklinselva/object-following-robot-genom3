#!/bin/bash
pkill genomixd
pkill -f 'CT_robot-ros'
pkill gzserver
pkill gzclient
pkill -f teleop_twist_keyboard.py 
pkill -9 genomixd
