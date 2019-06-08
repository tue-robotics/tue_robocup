# Where is this?

Responsible: Matthijs

The robot has to explain and show people where they can find places in the arena (e.g. Where is the TV?).
The robot has to tell the operator how to get there (de.

## Startup Guiding state
A new guiding state has been developed as preparation for updates of this challenge. To test this:
amigo1/sergio1/hero1:

    hero-start

amigo2/sergio2/hero2:

    hero-free-mode
    
amigo2/sergio2/hero2:

    roscd robot_smach_states/test
    python test_guidance.py <robot name> <entity id>
    
N.B.: this has not yet been tested on the robot.

## Startup

1. Place the robot near the appointed position (determined by TC on the spot)
2. Ask who ever stands in front of the robot where they woulf like to be guided to.
3. Explain, in a summarized fashion, how to get there
4. Then physically take the operator there in a type of tour guide

amigo1/sergio1/hero1:

    hero-start

amigo2/sergio2/hero2:

    hero-challenge-where-is-this

# Notes
