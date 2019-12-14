# Challenge hand me that

_Responsible: Rein_

## Scenario (RWC2019)

1. Start:

```
Robot starts at the entrance door en drives to the operator waypoint
```

2. Main loop

```
Robot starts routine for detecting the pointing pose of the operator. Based on the pointing pose, we raytrace ED and
determine the target furniture. This furniture will be expected and we will touch the object that we have detected.

Robot will return to the operator waypoint [loop]
```

## Testing / Running

```
robot-start
robot-challenge-hand-me-that
rosrun challenge_hand_me_that challenge_hand_me_that
```
