[![Stories in Ready](https://badge.waffle.io/tue-robotics/tue_robocup.png?label=ready&title=Ready)](https://waffle.io/tue-robotics/tue_robocup)
[![Build Status](https://travis-ci.org/tue-robotics/tue_robocup.svg?branch=master)](https://travis-ci.org/tue-robotics/tue_robocup)

# tue_robocup
RoboCup challenge implementations
This is to high-level code that TU Eindhoven's AMIGO robot uses to perform RoboCup@Home challenges.

Most, if not all, challenges are implemented as (large) hierarchical state machines, using the https://www.github.com/tue-robotics/robot_smach_states library of states.
These states use the https://www.github.com/tue-robotics/robot_skills robot abstraction layer.
