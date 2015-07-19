# Open Challenge

Responsible: Ramon & Sjoerd

## Setup

amigo1/sergio1:

    astart/sstart # (if not already running)

amigo2/sergio1:

    ahardware/shardware: base, spindle, arms (left-arm is used) head

amigo1/sergio1:

    amiddle/smiddle

tablet:

    http://amigo1:8000/challenge_open/dist/
    http://sergio1:8000/challenge_open/dist/

external pc:

    Sjoerd's visualization stuff???

## Start

    rosrun challenge_open challenge_open.py 

## In case you ONLY want to test grasping

	rosrun challenge_open challenge_open.py '<type>'

	where you put a type that has been assigned. Note that the 
	object to be grasped might need to be updated.

## Story

- The robot drives around autonomously and maps objects
- The gui can be used to label those snapshots
- You can *call* the robot for ... ???
- When the challenge ends, send a *exit* command via the gui