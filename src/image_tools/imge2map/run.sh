#!/bin/bash
roscore &
rosrun rviz rviz -d cfg.rviz &
python canny.py

