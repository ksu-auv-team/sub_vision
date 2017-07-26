#1/bin/bash
roslaunch sub_vision sub_vision.launch
rosrun yolo compressor_repeater /left/image_rect_color
rosrun yolo compressor_viewer /left/image_rect_color/jpegbuffer