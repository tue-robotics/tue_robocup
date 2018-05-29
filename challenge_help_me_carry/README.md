# Challenge Help Me Carry 2018
Responsible: Sam Aleksandrov

## Setup
amigo2/sergio2:

    amigo-challenge-help-me-carry/sergio-challenge-help-me-carry

### Testing in the lab/simulation

starting location: next to the dinner table, looking to the robocup field

for the simulation it's neccessary to simulate a human entity in RVIZ using:

    rosrun fast_simulator interactive.py human human 0 0 0

## During the challenge

- The robot starts following the operator to a point outside the arena. 
- When arrived at the destination, 'the car', a bag is handed over to the robot.
- The robot will deliver the bag to a location provided by the operator.
- At the location, it will guide a person back to the car.

! The rubber on the gripper could prevent the bag from dropping.
