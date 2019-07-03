# Challenge Receptionist

The robot must introduce new guests to the home owner (John) and offer them a seat.

Before the challenge, someone has to be taught to the robot under the name 'john',
so that the robot will introduce new guests to John (and other already introduced guests)

1. Learn the name of the new guest (who arrives through the door)
2. Ask new guests for their favorite drink
3. Guide guest to the living room
4. Find John
5. Introduce the new guest to John and the 'old' guest(s), by:
    - pointing at the guest
    - stating their name
    - stating their favorite drink
6. Find an empty seat for the new guest

## TODO:
- [x] Person detection does not work all the time...
- [x] Fallback for failing person detection?
- [x] Fix launch files so that ED does not crash due to missing services (mentioned by Rein)
- [x] Challenge crashes (ED detect people not available)
- [ ] Rise robot before human-robot interaction
