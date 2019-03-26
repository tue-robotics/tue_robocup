[![Build Status](https://travis-ci.com/tue-robotics/tue_robocup.svg?branch=master)](https://travis-ci.com/tue-robotics/tue_robocup)

# tue_robocup
RoboCup challenge implementations
This is to high-level code that TU Eindhoven's AMIGO robot uses to perform RoboCup@Home challenges.

Most, if not all, challenges are implemented as (large) hierarchical state machines, using the robot_smach_states library of states/state machines.
These states use the robot_skills robot abstraction layer.
