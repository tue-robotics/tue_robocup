[![Build Status](https://travis-ci.org/tue-robotics/challenge_test.svg?branch=master)](https://travis-ci.org/tue-robotics/challenge_test) [![codecov.io](https://codecov.io/github/tue-robotics/challenge_test/coverage.svg?branch=master)](https://codecov.io/github/tue-robotics/challenge_test?branch=master)

# Challenge Test

Responsible: Luis

----------------------------------------------------
                    Startup
----------------------------------------------------
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

    rosrun challenge_test test.py 


----------------------------------------------------
                    Behaviour
----------------------------------------------------

- drive to living room
- wait for person to step in front
- learn person's name
- learn person's face
- go to cabine
- find drinks
- pick random drink
- return to living room
- search for person learned ebfore
- deliver drink to the person