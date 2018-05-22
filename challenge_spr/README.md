# Responsible: Sam

## Procedure

	I Start: 
		The robot starts at a designated starting position and announces it wants to play riddles.

	II Waiting and turn: 
		The robot waits for 10 seconds while a crowd is merged on it’s back, then the robot must turn around (about 180deg) and find the crowd.

	III Requesting an operator: 
		After turning around, the robot must state the size of the crowd (including male and female count) and request for an operator (e.g. "Who want to play riddles with me?").

	IV The riddle game: 
		Standing in front of the robot, the operator will ask 5 questions. The robot must answer the question without asking confirmation. Questions will only be asked only once; no repetitions are allowed.

	V Blind man's bluff game:
		The crowd will reposition, making a circle around the robot. A random person from the crowd surrounding the robot will ask a question. The robot may
			• Turn towards the person who asked the question and answer the question
			• Directly answer the question without turning
			• Turn towards the person and ask them to repeat the question
		This process is repeated with 5 (possibly) different people. The game will end when the 5th question has been made. The robot must answer the question without asking confirmation. Questions may be repeated once.
	
	VI Leave:
		The robot must leave the arena/test area after all questions have been asked or
		when instructed to do so.

## Scoring sheet of 2018, Montreal:

	The maximum time for this test is 5 minutes!

	Crowd:
		State crowd’s size:								5
		State crowd’s male/female count: 				10

	Riddle game
		Understanding question: 						5 × 5
		Correctly answered a question: 					5 × 5
		Answering all 5 riddle game question: 			5

	Blind man’s bluff game:
		Understanding question on the first attempt: 	5 × 10
		Understanding question on the second attempt: 	5 × 5
		Correctly answered a question: 					5 × 5
		Turned towards person asking the question: 		5 × 10
		Answering all 5 blind man’s bluff questions: 	5

	Total score (excluding penalties and bonuses):		200

	Outstanding performance:							20

## Testing

Launch procedure:
	```
	amigo-start
	amigo-free-mode
	amigo-challenge-spr
	```

Testing the detection individually:
	```
	amigo-start
	amigo-free-mode
	rosrun challenge_spr detect.py amigo
	```

Testing the Riddle Game individually:
	```
	amigo-start
	amigo-free-mode
	rosrun challenge_spr riddle_game.py amigo
	```

Testing the Bluff Game individually:
	```
	amigo-start
	amigo-free-mode
	rosrun challenge_spr bluff_game.py amigo
	```

## TODO

	Predefined questions:
		- implement "What was the last (previous) question?"
		- update the 10 predefined questions at Montreal

	Arena questions:
		- implement "How many chairs are in the dining room?"

	Crowd questions:
		- implement Posture and Position detection
		- implement Color of shirt detection
		- implement "How old do you think I am?"
		- implement "Am I a man or a woman?"

	Object questions:
		- implement Count object at location
		- implement "What objects are at location?"