[![Build Status](https://travis-ci.org/tue-robototics/robot_smach_states.svg?branch=master)](https://travis-ci.org/tue-robototics/robot_smach_states)

robot_smach_states
==================

Robot_smach_states is a library of, indeed, smach states and state machine, built using the SMACH state machine library from ROS.

There are states conccerning navigation, perception, world modeling/reasoning, complex and simple arm movements, speech recognition, and some more.
Each state is passed a robot-object from the robot_skills-package.
The actions called via the smach_states are implemented and executed via such a robot-object. 
