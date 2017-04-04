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

## TODO

Riddle / Bluff games:

- update library with questions:
	- about the arena
	- about the inspected crowd
	- about list of official objects
- detect.py
	- use skybiometry node and service
	- use face_recognition node and service
- head.py
    - Change topic name of get_image so it depending on robot name
