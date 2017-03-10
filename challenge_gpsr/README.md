Responsible: Rokus

# Start-up

    <robot>-challenge-gpsr

# Scenario

The robot will drive to the meeting point and say "What can I do for you?" and then you can say your sentence.

# Testing

## Generating random command

    rosrun challenge_gpsr generate_command.py

or to get multiple commands at once:

    rosrun challenge_gpsr generate_command.py <number-of-commands>

## Test recognition

    rosrun challenge_gpsr test_recognition.py <robot-name>

# TODO:
 - Implementation of state machine design

# State machine design

The state machine builds upon the action server to perform the given tasks. Actions are assembled into a task. These actions require input, but may also provide input for next actions. If required input is needed before the first action can be performed, the task is not feasible, so the robot will have to ask for the missing information.

The task is performed (if possible) and the outcome is stored in a log. 

The robot goes back to the starting position, where it should meet the operator again (maybe it should look for the operator?). 

Finally, the task log is reported to the user just before the robot returns back to idle state.

![GPSR flow chart](doc/GPSR.jpeg)
