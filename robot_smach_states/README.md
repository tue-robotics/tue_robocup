[![Build Status](https://travis-ci.org/tue-robotics/robot_smach_states.svg?branch=master)](https://travis-ci.org/tue-robotics/robot_smach_states) [![codecov.io](https://codecov.io/github/tue-robotics/robot_smach_states/coverage.svg?branch=master)](https://codecov.io/github/tue-robotics/robot_smach_states?branch=master) [![Code Climate](https://codeclimate.com/github/tue-robotics/robot_smach_states/badges/gpa.svg)](https://codeclimate.com/github/tue-robotics/robot_smach_states)

robot_smach_states
==================

Robot_smach_states is a library of, indeed, smach states and state machine, built using the SMACH state machine library from ROS.

There are states concerning navigation, perception, world modeling/reasoning, complex and simple arm movements, speech recognition, and some more.
Each state is passed an instance of the Robot-class from the tue-robotics/robot_skills-package.
The actions called via the robot_smach_states are implemented and executed via such a Robot-object. 

![Architecture](https://raw.githubusercontent.com/tue-robotics/robot_smach_states/master/doc/layers.png)
