# RoboNurse challenge

Responsible: Loy

## Startup

1. ...

amigo1/sergio1:

    astart/sstart # (if not already running)

amigo2/sergio1:

    ahardware/shardware: base, spindle, arms (left-arm is used) head

amigo1/sergio1:

    amiddle/smiddle

amigo3:

    Test audio:

    rosrun robot_skills test_audio.py [robot_name]

    asdf

    #For simulator testing
    rosrun challenge_robonurse mock_challenge_robonurse.py

    rosrun challenge_robonurse challenge_robonurse.py [robot_name]

    or

    hear continue #Don't press enter just yet

Unplug the network cable in amigo.
localhost:
    sshamigo3 #This will not work because amigo is unplugged

# Notes

Before the challenge when setting up the environment
----------------------------------------------------
The shelf/closet from which the bottles should be grasped must be a composed object. 
The bookcase should be an entity, with sub-entities like bookcase/shelfX etc. 

During the challenge
--------------------

    - Tell the robot what pills to get when asked
    - Perform one of the actions (standup and drop blanket, walk away & sit & hand over cane or walk a metre or so and fall) after being handed the pills

After the challenge
-------------------

    - Enjoy your pills

Testing parts of the challenge
-----------------------------
$ roscd challenge_robonurse/src/
$ # Give a navgoal to make amigo face the shelf in rviz-amigo
$ amigo-console
>>> from challenge_robonurse import *
>>> test_get_pills(amigo) #There are other test_... as well. 
