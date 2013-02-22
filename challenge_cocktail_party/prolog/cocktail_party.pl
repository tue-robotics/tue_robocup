:- dynamic module_status/2.
:- dynamic current_state/2.
:- dynamic current_goal/1.
:- dynamic action/1.
:- dynamic warning/1.

:- dynamic heard_words/1.

:- dynamic known_person/1.
:- dynamic drink_ordered/2.

:- retractall(warning(_)).
:- retractall(action(_)).
:- retractall(current_state(_, _)).
:- retractall(current_goal(_)).

current_state(wait_for_timeout, 0).
current_state(wait_for_door, 1).

transition(wait_for_timeout, wait_for_timeout).

transition(wait_for_door, navigate_to_party_room) :-
    state(entrance_door, open).
transition(wait_for_door, wait_for_door).

transition(navigate_to_party_room, call_person) :-
    achieve(position(amigo, party_room)).
transition(navigate_to_party_room, navigate_to_party_room).

transition(call_person, detect_person) :-
    achieve(say('Please stand in front of me')).

transition(detect_person, learn_person(ID)) :-
    property_expected(ID, class_label, person),
    property_expected(ID, position, in_front_of(amigo)).
transition(detect_person, detect_person) :-
    achieve(module_status(person_detection, on)).
transition(detect_person, detect_person).

transition(learn_person(Person), take_order(Person)) :-
    known_person(Person),
    achieve(say('OK, thank you. Now I can find you back later.')).
transition(learn_person(Person), capture_person_images(Person)) :-
    achieve(say('Let me have a look at you')).
transition(capture_person_images(Person), learn_person(Person)) :-
    achieve(module_status(face_recognition, on)),
    assert(known_person(Person)).
transition(capture_person_images(Person), capture_person_images(Person)).

transition(take_order(Person), find_drink(Drink)) :-
    drink_ordered(Person, Drink).
transition(take_order(Person), ask_order(Person)) :-
    achieve(say(['So ', Person, '. What would you like to drink?'])).
transition(ask_order(Person), confirm_order(Person, Drink)) :-
    achieve(listen([coke, fanta], Drink)).
transition(confirm_order(Person, Drink), confirm_order(Person, Drink, Answer)) :-
    achieve(say(['I heard ', Drink, ', is that correct?'])),
    achieve(listen([yes, no], Answer)).
transition(confirm_order(Person, _, no), take_order(Person)).
transition(confirm_order(Person, Drink, yes), take_order(Person)) :-
    assert(drink_ordered(Person, Drink)).
    

%make_true(position(amigo, pose_2d(X, Y, Phi, FrameID))) :- false. % TODO

achieve(Goal) :-
    nl, write('Goal checked: '), write(Goal), nl,
    achieve(Goal, check)
    ->
        (nl, write('Goal achieved: '), write(Goal), nl,
        retractall(goal(Goal))
        )
    ;
        (add_goal(Goal),
        fail)
    .

add_goal(Goal) :-
    current_goal(Goal)
    ->
        true
    ;
        (   achieve(Goal, solution)
            ->
                assert(current_goal(Goal))
            ;
                add_warning(['No valid solution for goal ', Goal])
        )
    .

add_action(Action) :-
    assert(action(Action)).

add_warning(Warning) :-
    terms_to_atoms(Warning, WarningAtoms),
    atomic_list_concat(WarningAtoms, WarningMsg),
    assert(warning(WarningMsg)).

terms_to_atoms([], []).
terms_to_atoms([X|Xs], [Atom|Atoms]) :-
    not(atom(X)),
    term_to_atom(X, Atom),
    terms_to_atoms(Xs, Atoms).
terms_to_atoms([X|Xs], [X|Atoms]) :-
    atom(X),
    terms_to_atoms(Xs, Atoms).

achieve(position(amigo, Waypoint), check) :-
    waypoint(Waypoint, pose_2d(X, Y, Phi)),
    property_expected(amigo, pose, pose_2d(X2, Y2, Phi2)),
    abs(X-X2, DX), abs(Y-Y2, DY), abs(Phi-Phi2, DPhi),
    DX < 0.1, DY < 0.2, (DPhi < 0.1; DPhi > 6.2).
achieve(position(amigo, Waypoint), solution) :-
    waypoint(Waypoint, pose_2d(X, Y, Phi)),
    add_action(navigate_to(X, Y, Phi, '/map')).

achieve(say(TextList), check) :-
    terms_to_atoms(TextList, TextAtoms),
    atomic_list_concat(TextAtoms, Text),
    add_action(say(Text)).
achieve(say(Text), check) :-
    add_action(say(Text)).

achieve(listen(_, Words), check) :-
    heard_words(Words),
    retractall(heard_words(_)).
achieve(listen(Options, _), solution) :-
    add_action(listen(Options)).
    
achieve(module_status(Module, Status), check) :-
    module_status(Module, Status).
achieve(module_status(Module, Status), solution) :-
    assert(module_status(Module, Status)). % TODO

%property_expected(A, B, C) :- false.
%property_expected(door1, status, open).
%waypoint(party_room, pose_2d(0, 1, 2)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

step(Actions, Warnings) :-
    retractall(action(_)),
    retractall(warning(_)),
    list_states(States),
    do_transitions(States),
    findall(Action, action(Action), Actions),
    findall(Warning, warning(Warning), Warnings).

% Return States in a list in order of priority
list_states(States) :-
    list_states(-1, States).
list_states(Priority, [State|States]) :-
    current_state(State, Priority2),
    Priority2 > Priority, !,
    list_states(Priority2, States).
list_states(_, []).

do_transitions([]).
do_transitions([State|States]) :-
    do_transition(State),
    do_transitions(States).

do_transition(State) :-
    transition(State, NewState), !,
    retract(current_state(State, Priority)),
    assert(current_state(NewState, Priority)),
    nl, write('Transition: '), write(State), write(' ---> '), write(NewState), nl.
do_transition(State) :-
    add_warning(['Could not find a transition for state ', State]).

print_state_machine :-
   % find all transitions
   findall(transition(A, B), clause(transition(A, B), _), Transitions),
   print_transitions(Transitions).

print_transitions([]).
print_transitions([transition(A, B)|Transitions]) :-
    write(A), write(' ---> '), write(B), nl,
    print_transitions(Transitions).
