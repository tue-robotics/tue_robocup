# tue_robocup
[![Build Status](https://dev.azure.com/tue-robotics/tue-robotics/_apis/build/status/tue-robotics.tue_robocup?branchName=master)](https://dev.azure.com/tue-robotics/tue-robotics/_build/latest?definitionId=1&branchName=master)\
RoboCup@Home challenge implementations
This repository contains the high-level code that TU Eindhoven's AMIGO, HERO and SERGIO robots use to perform RoboCup@Home challenges.

Most, if not all, challenges are implemented as (large) hierarchical state machines, using the `robot_smach_states` library of states/state machines.
These states use the `robot_skills` robot abstraction layer.
