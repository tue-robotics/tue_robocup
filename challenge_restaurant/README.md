# Challenge Restaurant 2017

Responsible: Lars

## Startup

* place the robot next to the kitchen where the chef is. (robot detects a large ed entity as the side of the kitchen)
* amigo-start should already be running
* make sure the Jetson is running (check if <robot-name>/pose_detector/openpose is running)
```
rosnode list | grep pose_detector
```
* start the middleware
```
amigo-challenge-restaurant (starts the middleware, without state_machine)
```
* start the challenge
```
rosrun challenge_restaurant challenge_restaurant <robot-name>
```
* navigate around for a bit to get a proper map. (But still leave some unknown space)
* have a customer wave to call <robot-name> for placing an order
* to get the robot to take the order, answer "<robot-name> take the order"
* when ordering, you can choose a beverage or a combo (food & food). Check the RoboCup knowledge to see what foods and drinks are.
