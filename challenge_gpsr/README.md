Responsible: Sjoerd

# Start-up

    amiddle

    rosrun challenge_gpsr gpsr.py <robot-name>

# Scenario

The robot will drive to the meeting point and say "What can I do for you?" and then you can say your sentence.

# Testing

## Generating random command

    rosrun challenge_gpsr generate_command.py

or to get multiple commands at once:

    rosrun challenge_gpsr generate_command.py <number-of-commands>

## Test recognition

    rosrun challenge_gpsr test_recognition.py <robot-name>