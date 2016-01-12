# Challenge Navigation 2016
Responsible: Rein Appeldoorn

## Test Log
- 2016-01-12: Performed fine, same state as rwc 2015. No person detected implemented yet.
- 2016-01-12: Performed test with object and human recognition, works fine except for the segmentation, we need a z-min z-max option in segment function.

## Setup
amigo1/sergio1:

    astart/sstart # (if not already running)

amigo2/sergio1:

    ahardware/shardware: base, spindle, arms (left-arm is used) head

amigo1/sergio1:

    amiddle/smiddle

amigo3:

    rosrun challenge_navigation challenge_navigation.py [robot_name]

### During a real competition
On the dev PC:

    sshamigo3 #This will not work because amigo is unplugged

## During the challenge

- The robot enters the arena
- The robot will (try to) go to the first waypoint (dinner_table) and finally say whether it reached its goal or not. The idea of this challenge is to demonstrate flexibility of the navigation: if one path appears to be blocked, the robot should be able to either replan or clear the way to its goal. So don't be easy on the navigation! Block a (few) paths to test this!
- The robot will (try to) go to the second waypoint (black_cabinet2) and finally say whether it reached its goal or not. Again, you may want to block a path to test the navigation.
- After this message the robot will go to the exit

### During a real competition

- Plug the network cable back in
- The sshamigo3 will connect
- type 'hear continue' in the sshamigo3 console.

