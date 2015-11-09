#! /usr/bin/env bash

#Find a topic thats a laser scan and publish a fake laser scan from laser_door_open.yaml on it. Keep latching this too
rostopic pub $(rostopic find sensor_msgs/LaserScan) sensor_msgs/LaserScan -f laser_door_open.yaml -l
rostopic pub "/mockbot/base_laser/scan" sensor_msgs/LaserScan -f laser_door_open.yaml -l
