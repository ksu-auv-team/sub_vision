#!/bin/bash
rosrun yolo compressed_repeater /front_cam/image_raw/
rosbag record -O bottom_cam /front_cam/image_raw/jpegbuffer
