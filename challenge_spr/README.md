# Responsible: Sam

## Testing

Launch procedure:
```
amigo-start
amigo-free-mode
rosrun challenge_spr challenge_spr.py amigo
```

Testing the detection individually:
```
rosrun challenge_spr detect.py amigo
```

Start at the Riddle Game
```
rosrun challenge_spr riddle_game.py amigo
```

Start at the Bluff Game
```
rosrun challenge_spr bluff_game.py amigo
```

## DONE

- main scenario is ready
- riddle_game and bluff_game are ready for standalone testing
- riddle game works in simulation, should be tested on robot
- bluff game works in simulation, should be tested on robot
- detectCrowd:
	- create robot.perception.getImage() skill
		- create image listener
		- grab one image
	- create robot.perception.detectFaces(image)
		- send the image to /amigo/face_recognition/recognize
		- process the results
- send data for the inspected crowd to the library
- updated library with questions for crowd, arena and objects

## TODO

Riddle game:
	- finish refactoring by creating functions for all long answers
	- test

Bluff game:
	- refactor by simplification of the code and reusing instead of copying the function of riddle game
	- test
