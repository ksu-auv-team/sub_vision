#!/bin/bash
rosrun yolo compressed_repeater /left/image_rect_color/
rosbag record -O zed_left /left/image_rect_color/compressed
