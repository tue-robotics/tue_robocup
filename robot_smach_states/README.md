# robot_smach_states

Robot_smach_states is a library of, indeed, smach states and state machine, built using the SMACH state machine library from ROS.

There are states concerning navigation, perception, world modeling/reasoning, complex and simple arm movements, speech recognition, and some more.
Each state is passed an instance of the Robot-class from the tue-robotics/robot_skills-package.
The actions called via the robot_smach_states are implemented and executed via such a Robot-object. 

![Architecture](./doc/layers.png)
