# Manipulation and Object recognition

Responsible: Loy & Ramon

## Startup

1. Place the robot near or in front of the bookcase, cabinet or closet it is supposed to grasp from. This is currently hardcoded into variable BOOKCASE.
2. Place 5 items in the bookcase
3. In this bookcase, there should be 1 empty shelve at height PLACE_HEIGHT.

amigo1/sergio1:

    astart/sstart # (if not already running)

amigo2/sergio1:

    ahardware/shardware: base, spindle, arms (left-arm is used) head

amigo1/sergio1:

    amiddle/smiddle

amigo3:

    Mount usb stick:

    rosrun challenge_manipulation mount_usb # usb dir in /home/amigo should be green

    Test audio:

    rosrun robot_skills test_audio.py [robot_name]

    asdf

    rosrun challenge_manipulation manipulation.py [robot_name]

    or

    hear continue #Don't press enter just yet

Unplug the network cable in amigo.
localhost:
    sshamigo3 #This will not work because amigo is unplugged

# Notes

During the challenge
--------------------

    Nothing to do, just tay out the way :-)

After the challenge
-------------------

    - Get the USB stick and inspect the classes in the file manipulation.pdf
