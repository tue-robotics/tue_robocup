# Responsible: Sam

# DONE

- main scenario is ready
- riddle_game and bluff_game are ready for standalone testing
- riddle game works in simulation, should be tested on robot

# TODO

detectCrowd:

- create robot.perception.getImage() skill
	- create image listener
	- grab one image
- create robot.perception.detectFaces(image)
	- send the image to /amigo/face_recognition/recognize
	- process the results

Riddle / Bluff games:

- send data for the inspected crowd to the library

- make library with questions:
	- about the arena
	- about the inspected crowd
	- about list of official objects

- repair bluff_game.py functions to work in the following order:
	- challenge_description: 	voice input -> turn to person -> answer
	- current_status: 			voice input -> answer -> turn to person