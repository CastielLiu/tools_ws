#!/bin/bash
roscore &
rosrun rviz rviz -d cfg.rviz &
python image2map.py

