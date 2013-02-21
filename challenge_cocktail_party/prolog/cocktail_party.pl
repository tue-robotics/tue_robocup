:- dynamic module_status/2.
:- dynamic current_state/1.

:- retractall(current_state(_)).
current_state(wait_for_door).

transition(wait_for_door, navigate_to_party_room) :-
    property_expected(door1, status, open).
transition(wait_for_door, wait_for_door).

transition(navigate_to_party_room, call_person) :-
    make_true(position(amigo, party_room)).
transition(navigate_to_party_room, navigate_to_party_room).

transition(call_person, detect_person) :-
    make_true(say('Please stand in front of me')).

transition(detect_person, learn_person(ID)) :-
    property_expected(ID, class_label, person),
    property_expected(ID, position, in_front_of(amigo)).
transition(detect_person, detect_person) :-
    make_true(module_status(person_detection, on)).

transition(learn_person(ID), take_order) :-
    known_person(ID).
transition(learn_person(ID), learn_person(ID)) :-
    make_true(learn_person(ID)).

make_true(position(amigo, pose_2d(X, Y, Phi, FrameID))) :- false. % TODO

make_true(position(amigo, Waypoint)) :-
    waypoint(Waypoint, pose_2d(X, Y, Phi)),
    property_expected(amigo, position, pose_2d(X2, Y2, Phi2)),
    abs(X-X2, DX), abs(Y-Y2, DY), abs(Phi-Phi2, DPhi),
    DX < 0.1, DY < 0.2, (DPhi < 0.1; DPhi > 6.2).
make_true(position(amigo, Waypoint)) :-
    write('Navigating to '), write(Waypoint).
    
make_true(module_status(Module, Status)) :-
    module_status(Module, Status).
make_true(module_status(Module, Status)) :-
    assert(module_status(Module, Status)). % TODO

property_expected(A, B, C) :-
    write(' Queried: property_expected('), write(A), write(', '),
    write(B), write(', '), write(C), write(')'), nl.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

step(NewState) :-
    current_state(State),
    transition(State, NewState), !,
    retractall(current_state(_)),
    assert(current_state(NewState)).

print_state_machine :-
   % find all transitions
   findall(transition(A, B), clause(transition(A, B), _), Transitions),
   print_transitions(Transitions).

print_transitions([]).
print_transitions([transition(A, B)|Transitions]) :-
    write(A), write(' ---> '), write(B), nl,
    print_transitions(Transitions).
