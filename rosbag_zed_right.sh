#!/bin/bash
rosrun yolo compressed_repeater /right/image_rect_color/
rosbag record -O zed_right /right/image_rect_color/jpegbuffer
