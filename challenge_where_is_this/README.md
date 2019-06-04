# Storing groceries

Responsible: Janno

## Startup

1. Place the robot near or in front of the bookcase, cabinet or closet it is supposed to grasp from. This is currently hardcoded into variable BOOKCASE.
2. Place 5 items in the bookcase on various shelves
3. Put a number of items on the table

amigo1/sergio1:

    amigo-start

amigo2/sergio2:

    Mount usb stick:

    rosrun challenge_storing_groceries mount_usb # usb dir in /home/amigo should be green
    
    amigo-challenge-storing-groceries

# Notes

Before the challenge
--------------------
To run this in simulation, you first need to add the simulated objects to the simulator. In order to do so, run:
rosrun challenge_storing_groceries add_entities.py

During the challenge
--------------------

    Nothing to do, just tay out the way :-)

After the challenge
-------------------

    - Get the USB stick and inspect the classes in the file manipulation.pdf --> ToDo

Installation
------------

If you encounter an error when importing Pisa, install the broken dependency
sudo pip install html5lib==1.0b8
