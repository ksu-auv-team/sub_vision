#!/bin/bash
rosrun yolo compressed_repeater /usb_cam/image_raw/
rosbag record -O bottom_cam /usb_cam/image_raw/compressed
