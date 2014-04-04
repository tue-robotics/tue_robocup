#! /bin/bash
rosrun rgbd_transport server rgb_image:=/amigo/top_kinect/rgb/image_rect_color depth_image:=/amigo/top_kinect/depth_registered/image_rect cam_info:=/amigo/top_kinect/rgb/camera_info __max_fps:=30 output:=/amigo/top_kinect/rgbd
