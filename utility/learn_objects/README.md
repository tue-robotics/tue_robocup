[![Build Status](https://travis-ci.org/tue-robotics/learn_objects.svg?branch=master)](https://travis-ci.org/tue-robotics/challenge_test) [![codecov.io](https://codecov.io/github/tue-robotics/challenge_test/coverage.svg?branch=master)](https://codecov.io/github/tue-robotics/challenge_test?branch=master)

# Learn Objects

Responsible: Peter


----------------------------------------------------
                    Setup
----------------------------------------------------
Place hero's flightcase approximately 1 meter to the right of hero, with the long side facing hero.
Place the objects to be recognised on top of the flightcase.

----------------------------------------------------
                    Startup
----------------------------------------------------
1. ...

amigo/sergio:

learn_objects currently does not work for amigo

hero:
   
hero1:

    hero-start
    
hero2:

    hero-learn-objects
    

----------------------------------------------------
                    Behaviour
----------------------------------------------------

- Rise and look at his flightcase
- Say "Say cheese"
- Take a picture of the objects on top of the flightcase
- Wait 2 seconds: during this time the operator may move the flightcase to get pictures from a different angle.
- After taking 10 pictures the robot will say "pause". It will then wait 5 seconds, 
during this time the operator can exchange the objects on top of the flight case.
