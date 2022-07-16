# Final challenge RWC2022

## Scenario (RWC2022)

1. Start:

```
Robot starts in the arena
```

2. Story

```
[Rein] Robot drives arbitrarily and detect humans, says hi --> detect waving person
[Matthijs] Goes to waving victim, interact with it
[Arpit] Robot calls neighbor for help
[Lars, Lotte, Rodrigo] Robot navigates to the door and guides neighbor to victim
[Lotte] Outro
```

3. Outro

```
Robot leaves the arena.
```

## Testing / Running

```
hero-start
hero-challenge-final
```

### Everything can be tested separately

```
python3 $(rospack find challenge_final)/src/challenge_final/navigate_arbitrarily.py
python3 $(rospack find challenge_final)/src/challenge_final/navigate_to_waypoint_and_detect_victim.py
python3 $(rospack find challenge_final)/src/challenge_final/navigate_to_and_interact_with_victim.py
python3 $(rospack find challenge_final)/src/challenge_final/call_neighbor.py
python3 $(rospack find challenge_final)/src/challenge_final/navigate_to_the_door_and_guide_neighbor_to_victim.py
python3 $(rospack find challenge_final)/src/challenge_final/outro.py
```
