# WakeMeUp challenge

Responsible: Loy

## Startup

1. ...

amigo1/sergio1:

    astart/sstart # (if not already running)

amigo2/sergio1:

    ahardware/shardware: base, spindle, arms, head

amigo1/sergio1:

    amiddle/smiddle

amigo3:

    Test audio:

    rosrun robot_skills test_audio.py [robot_name]

    rosrun challenge_wakemeup challenge_wakemeup.py [robot_name]

    or

    hear continue #Don't press enter just yet

Unplug the network cable in amigo.
localhost:
    sshamigo3 #This will not work because amigo is unplugged

# Notes

Before the challenge
--------------------

    - Make sure the knowledge is up to date!!!


During the challenge
--------------------

    - At initialization, the robot will ask for the newspaper to be put in its gripper. Do this within the time given in the robocup knowledge!
    - The robot will go to the bed, look for an entity above the bed with a number of constraints (knowledge)
    - The operator "wakes up" and sits up in the bed; the robot will detect this.
    - The robot will hand the operator his newspaper and ask for his breakfast order
    - The robot will navigate to the kitchen to prepare breakfast
    - The robot will prepare the breakfast as far as it knows how to.

After the challenge
-------------------

    - 