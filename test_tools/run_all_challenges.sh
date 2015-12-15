#! /usr/bin/env bash

set -e #Enable stopping on a single error

rosrun test_tools open_door.sh  >> /dev/null &
rosrun challenge_test test.py mockbot

rosrun challenge_manipulation manipulation.py mockbot

rosrun test_tools open_door.sh  >> /dev/null &
rosrun challenge_navigation challenge_navigation.py mockbot

timeout 1m rosrun challenge_person_recognition person_recognition.py mockbot

rosrun test_tools open_door.sh  > open_door_log.txt &
rosrun challenge_rips rips.py mockbot

rosrun challenge_robonurse challenge_robonurse.py mockbot

rosrun challenge_speech_recognition speech_recognition.py mockbot

rosrun test_tools open_door.sh  > open_door_log.txt &

timeout 5m rosrun challenge_gpsr gpsr.py mockbot

echo "All challenges passed successfully"
