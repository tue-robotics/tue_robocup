:- dynamic module_status/2.
:- dynamic current_state/3.
:- dynamic current_goal/1.
:- dynamic action/1.
:- dynamic warning/1.

:- dynamic heard_words/1.

:- dynamic known_person/1.
:- dynamic drink_ordered/2.

:- dynamic property_expected/3.

% initialization
:- retractall(warning(_)).
:- retractall(action(_)).
:- retractall(current_state(_, _, _)).
:- retractall(current_goal(_)).

% Set current state
current_state(meta, wait_for_timeout, 0).
current_state(cp, wait_for_door, 1).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                            TRANSITIONS                               %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

transition(_, State, State).

transition(meta, wait_for_timeout, wait_for_timeout).

% init
transition(cp, init, wait_for_door) :-
    achieve(module_status(object_recognition, off)),
    achieve(module_status(person_detection, off)),
    achieve(module_status(face_recognition, off)).

% wait_for_door
transition(cp, wait_for_door, find_person_for_order) :-
    property_expected(entrance_door, status, open).

% find_person_for_order
transition(cp, find_person_for_order, learn_person(PersonID)) :-
    achieve(find_person(PersonID, [party_room])).

% learn_person
transition(cp, learn_person(PersonID), take_order(PersonID)) :-
    achieve(learn_person(PersonID)).

% take_order
transition(cp, take_order(Person), find_object(Drink)) :-
    drink_ordered(Person, Drink).
transition(cp, take_order(Person), ask_order(Person)) :-
    achieve(say(['So ', Person, '. What would you like to drink?'])).
transition(cp, ask_order(Person), confirm_order(Person, Drink)) :-
    achieve(listen([coke, fanta], Drink)).
transition(cp, confirm_order(Person, Drink), confirm_order(Person, Drink, Answer)) :-
    achieve(say(['I heard ', Drink, ', is that correct?'])),
    achieve(listen([yes, no], Answer)).
transition(cp, confirm_order(Person, _, no), take_order(Person)).
transition(cp, confirm_order(Person, Drink, yes), take_order(Person)) :-
    assert(drink_ordered(Person, Drink)).

% find_object
transition(cp, find_object(Drink), grab_object(DrinkID)) :-
    achieve(find_object(
                DrinkID,
                property_expected(DrinkID, class_label, Drink),
                [storage_room]
            )).

% grab_object
%transition(cp, grab_object(DrinkID), return_to_person(Person)) :-
%    achieve(grab_object(DrinkID)),
%    property_expected(DrinkID, class_label, Drink),
%    drink_ordered(Person, Drink).

% grab_drink
%transition(grab_drink(DrinkID), find_person(Person)) :-
%    achieve(grab(DrinkID)),
%    property_expected(DrinkID, class_label, Drink),
%    drink_ordered(Person, Drink).


% return_drink
%transition(return_drink(DrinkID, Person), hand_over(DrinkID, Person)).

%transition(find_person(Person), redetect_person(Person)) :-
%    property_expected(Person, position, PersonPos),
%    achieve(position(amigo, PersonPos)).

% redetect_person
%transition(redetect_person(Person), learn_person(ID)) :-
%    property_expected(Person, class_label, person),
%    property_expected(Person, position, in_front_of(amigo)).
%transition(redetect_person(Person), redetect_person(Person)) :-
%    achieve(module_status(person_detection, on)).

%transition(return_drink(DrinkID, Person), hand_over(DrinkID, Person)) :-
%    property_expected(Person, position, PersonPos),
%    achieve(position(amigo, PersonPos)).
    


% % % % % % % % % % % % % % % FIND PERSON % % % % % % % % % % % % % % %

transition(find_person, start(PersonID, _), ok) :-
    property_expected(PersonID, class_label, person),
    property_expected(PersonID, position, in_front_of(amigo)),
    achieve(module_status(person_detection, off)).
transition(find_person, start(PersonID, [Waypoint|Waypoints]), look(PersonID, Waypoints)) :-
    achieve(position(amigo, Waypoint)).
transition(find_person, start(PersonID, []), fail) :-
    achieve(say(['I looked everywhere, but I could not find ', PersonID])).

transition(find_person, look(PersonID, Waypoints), start(PersonID, Waypoints)) :-
    achieve(module_status(person_detection, on)),
    achieve(wait(10)).

% % % % % % % % % % % % % % % LEARN PERSON % % % % % % % % % % % % % % %

transition(learn_person(PersonID), start, ok) :-
    known_person(PersonID),
    achieve(say('OK, thank you. Now I can find you back later.')),
    achieve(module_status(face_recognition, off)).
transition(learn_person(_), start, capture) :-
    achieve(say('Let me have a look at you')).
transition(learn_person(PersonID), capture, capture) :-
    achieve(module_status(face_recognition, on)),
    assert(known_person(PersonID)).

% % % % % % % % % % % % % % % FIND OBJECT % % % % % % % % % % % % % % %

% find object
transition(find_object(ObjectID, ObjectQuery, Waypoints), start(Waypoints), ok) :-
    call(ObjectQuery),
    property_expected(ObjectID, position, in_front_of(amigo)),
    achieve(module_status(object_recognition, off)).
transition(find_object(_, _), start(Waypoints), look(Waypoints)) :-
    achieve(position(amigo, waypoint)).
transition(find_object(_, _), start([]), fail) :-
        achieve(say(['I searched everywhere, but I could not find the object I was looking for.'])).

transition(find_object(_, _), look([Waypoint|Waypoints]), start(Waypoints)) :-
    region_of_interest(Waypoint, ROI),
    achieve(look_at(ROI)),
    achieve(module_status(object_recognition, on)),
    achieve(wait(10)).

% % % % % % % % % % % % % % % FIND OBJECT % % % % % % % % % % % % % % %

% grab object
transition(grab_object(_), start, ok).

% % % % % % % % % % % % % % % GRAB OBJECT % % % % % % % % % % % % % % %

state_machine(find_person).
state_machine(learn_person(_PersonID)).
state_machine(find_object(_ObjectID, _ObjectQuery, _Waypoints)).
state_machine(grab_object(_ObjectID)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                             ACHIEVERS                                %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set-up sub-state print_state_machine
achieve(Goal, check) :-
    current_state(Goal, ok, _),
    retractall(current_state(Goal, _, _)).
achieve(Goal, solution) :-
    state_machine(Goal),
    get_new_priority_level(Priority),
    assert(current_state(Goal, start, Priority)).

% skills

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

% look_at
achieve(look_at(point_3d(X, Y, Z)), check) :-
    add_action(look_at(X, Y, Z, '/map')). % assume call is blocking

% wait
achieve(wait(Seconds), check) :-
    add_action(wait(Seconds)). % assume call is blocking

% grab
achieve(grab(ObjectID), check) :-
    add_action(grab(ObjectID)).  % assume call is blocking

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                               ENGINE                                 %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

step(Actions, Warnings) :-
    retractall(action(_)),
    retractall(warning(_)),
    list_states(States), !,
    do_transitions(States),
    findall(Action, action(Action), Actions),
    findall(Warning, warning(Warning), Warnings).

% Return States in a list in order of priority
list_states(States) :-
    list_states(-1, States).
list_states(Priority, [state(Machine, State, Priority2)|States]) :-
    current_state(Machine, State, Priority2),
    Priority2 > Priority, !,
    list_states(Priority2, States).
list_states(_, []).

do_transitions([]).
do_transitions([State|States]) :-
    do_transition(State),
    do_transitions(States).

do_transition(state(Machine, State, Priority)) :-
    transition(Machine, State, NewState), 
    retractall(current_state(Machine, State, Priority)), 
    assert(current_state(Machine, NewState, Priority)), !,
    nl, write('Transition: '), write(Machine), write(':'), write(State), write(' ---> '), write(NewState), nl.
do_transition(state(Machine, State, _)) :-
    add_warning(['Could not find a transition for state ', Machine, ':', State]).


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

get_new_priority_level(LNew) :-
    current_state(_, _, L1),
    not((current_state(_, _, L2), L2 > L1)),
    LNew is L1 + 1.

%%%%%%%% HELPER PREDICATES %%%%%%%%%

terms_to_atoms([], []).
terms_to_atoms([X|Xs], [Atom|Atoms]) :-
    not(atom(X)),
    term_to_atom(X, Atom),
    terms_to_atoms(Xs, Atoms).
terms_to_atoms([X|Xs], [X|Atoms]) :-
    atom(X),
    terms_to_atoms(Xs, Atoms).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                            INTROSPECTION                             %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



print_state_machine :-
   % find all transitions
   findall(transition(A, B), clause(transition(A, B), _), Transitions),
   print_transitions(Transitions).

print_transitions([]).
print_transitions([transition(A, B)|Transitions]) :-
    write(A), write(' ---> '), write(B), nl,
    print_transitions(Transitions).
