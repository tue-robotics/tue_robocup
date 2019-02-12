#!/usr/bin/python

"""
Clean UP [Housekeeper] challenge
This challenge is described in the 2019 RoboCup@Home Rulebook / Draft version

Main goal
Upon entrance, the robot requests the operator which room shall be cleaned. All misplaced known objects
found in this room must be taken to their predefined locations and unknown objects thrown in the trash bin.

Number of objects: 5
Objects can be anywhere, including the floor, seats, and on furniture. All objects are visible from
at least 1.0 m distance (no occlusions) and have the following distributions:
    Known objects: Any two regular and two alike objects
    Unknown objects: One unknown object at grasping distance (i.e. no decorations)

Reward: 500 pts (100 pts per object)
Bonus: max 500 pts



"""
# ROS
import smach

# Robot smach states
import robot_smach_states as states

# Knowledge has not been made yet.
challenge_knowledge = load_knowledge('challenge_cleanup')

# Make statemachine here

# Request the operator which room has to be cleaned
#       Need some conversation engine here ( see gpsr.py?)
#       Say "Which room do you want me to clean"
#       Wait for a correct answer, but not too long....

# Go to the entrance of the requested room
#       NavigateTo(StartPoint)

# Iterate until all points are visited
#       Make snapshot with camera and distinguish none/recognized/unrecognized objects
#       Update the respective lists for the objects
#       Move to the next point

# Go to the exit of the room
#       NavigateTo(ExitPoint)

# Recite the list of found known objects
#       Say "I have seen ... and ... and ..."


#
