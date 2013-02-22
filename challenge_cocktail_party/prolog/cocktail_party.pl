%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                                 INIT                                 %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- dynamic current_state/3.
:- retractall(current_state(_, _, _)).

:- dynamic current_goal/1.
:- retractall(current_goal(_)).

:- dynamic action/1.
:- retractall(action(_)).

:- dynamic warning/1.
:- retractall(warning(_)).

:- dynamic heard_words/1.
:- retractall(heard_words(_)).

:- dynamic known_person/1.
:- retractall(known_person(_)).

:- dynamic object_state/2.
:- retractall(object_state(_, _)).

% challenge specific

:- dynamic drink_ordered/2.
:- retractall(drink_ordered(_,_)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                               SYNONYMS                               %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

synonym(A, B) :-
    synonym1(A, B).
synonym(A, B) :-
    synonym1(B, A).

synonym1(coke, coke_can).
synonym1(fanta, fanta_can).
synonym1(water, bottle).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                           INITITIAL STATE                            %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set current state
%current_state(meta, wait_for_timeout, 0).
current_state(cp, init, 1).
%current_state(cp, test, 1).


%transition(cp, test, end) :-
%    achieve(grab(obj1)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                            TRANSITIONS                               %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

transition(meta, wait_for_timeout, wait_for_timeout).

% init
transition(cp, init, wait_for_door) :-
    achieve(module_status(template_matching, off)),
    achieve(module_status(face_detection, off)),
    achieve(module_status(face_recognition, off)),
    achieve(look_at(point_3d(10, 0, 1.5, '/base_link'))).

% wait_for_door
transition(cp, wait_for_door, find_person_for_order) :-
    object_state(entrance_door, open).

% find_person_for_order
transition(cp, find_person_for_order, learn_person(PersonID)) :-
    achieve(find_person(PersonID, [party_room]), ok).
transition(cp, find_person_for_order, find_person_for_order) :-
    achieve(find_person(PersonID, [party_room]), fail),
    achieve(say('This looks like a great party! Does anyone need a drink? If so, please come over here!')).

% learn_person
transition(cp, learn_person(PersonID), take_order(PersonID)) :-
    achieve(learn_person(PersonID)).

% taking the order
transition(cp, take_order(PersonID), confirm_order(PersonID, DrinkType)) :-
    achieve(ask('What would you like to drink?', [coke, fanta], DrinkType)).

transition(cp, confirm_order(PersonID, DrinkType), check_answer(PersonID, DrinkType, Answer)) :-
    achieve(ask(['I heard ', DrinkType, ', is that correct?'], [yes, no], Answer)).

transition(cp, check_answer(PersonID, DrinkType, yes), find_object(DrinkType)) :-
    assert(drink_ordered(PersonID, DrinkType)),
    achieve(say(['Alright! Ill get the ', DrinkType, ' for you!'])).
transition(cp, check_answer(PersonID, _, _), take_order(PersonID)) :-
    achieve(say('I am so sorry.')).

% find_object
transition(cp, find_object(Drink), NextState) :-
    findall(storage_room(A), waypoint(storage_room(A), _), Waypoints),
    achieve(find_object(
                DrinkID,
                ( synonym(Drink, ObjectType), property_expected(DrinkID, class_label, ObjectType) ),
                Waypoints
            ), Result),
    (
        Result = ok
    ->
        NextState = grab_object(DrinkID)
    ;
        achieve(say('What a pity. Maybe I have better luck with other people. Lets get back to the party!')),
        NextState = find_person_for_order,
        achieve(look_at(point_3d(10, 0, 1.5, '/base_link')))
    ).

% grab_object
transition(cp, grab_object(DrinkID), return_to_person(Person)) :-
    achieve(grab(DrinkID)),
    property_expected(DrinkID, class_label, Drink),
    drink_ordered(Person, Drink).

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

transition(find_person(PersonID, Waypoints), start, explore(PersonID, Waypoints)).

transition(find_person(PersonID, _), explore(PersonID, _), end(ok)) :-
    property_expected(PersonID, class_label, person),
    property_expected(PersonID, position, in_front_of(amigo)),
    achieve(module_status(face_detection, off)).
transition(find_person(PersonID, _), explore(PersonID, [Waypoint|Waypoints]), look(PersonID, Waypoints)) :-
    achieve(position(amigo, Waypoint)).
transition(find_person(PersonID, _), explore(PersonID, []), end(fail)).

transition(find_person(PersonID, _), look(PersonID, Waypoints), explore(PersonID, Waypoints)) :-
    achieve(module_status(face_detection, on)),
    achieve(wait(10)).

% % % % % % % % % % % % % % % LEARN PERSON % % % % % % % % % % % % % % %

transition(learn_person(PersonID), start, end(ok)) :-
    known_person(PersonID),
    achieve(say('Hey, I know you! Good to see you again!')),
    achieve(module_status(face_recognition, off)).
transition(learn_person(_), start, capture) :-
    achieve(say('Hi there! Always good to see a new face! Let me have a look at you')),
    achieve(look_at(point_3d(10, 0, 1.5, '/base_link'))).

transition(learn_person(PersonID), capture, end(ok)) :-
    known_person(PersonID),
    achieve(say('OK, thank you! I will try very hard to remember you!')),
    achieve(module_status(face_recognition, off)).
transition(learn_person(PersonID), capture, capture) :-
    achieve(module_status(face_recognition, on)),
    assert(known_person(PersonID)).

% % % % % % % % % % % % % % % FIND OBJECT % % % % % % % % % % % % % % %

% find object
transition(find_object(_, _, Waypoints), start, explore(Waypoints)).

transition(find_object(ObjectID, ObjectQuery, Waypoints), explore(Waypoints), end(ok)) :-
    call(ObjectQuery),
    property_expected(ObjectID, position, in_front_of(amigo)),
    achieve(module_status(template_matching, off)).
transition(find_object(_, _, _), explore([Waypoint|Waypoints]), look([Waypoint|Waypoints])) :-
    achieve(position(amigo, Waypoint)).
transition(find_object(_, _, _), explore([]), end(fail)) :-
        achieve(say(['I searched everywhere, but I could not find the object I was looking for.'])).

transition(find_object(_, _, _), look([Waypoint|Waypoints]), explore(Waypoints)) :-
    region_of_interest(Waypoint, ROI),
    achieve(look_at(ROI)),
    achieve(module_status(template_matching, on)),
    achieve(wait(10)).

transition(find_object(_, _, _), look([Waypoint|Waypoints]), explore(Waypoints)) :-
    not(region_of_interest(Waypoint, _)),
    achieve(say(['Hmmm, I dont know where to look here. Better continue exploring.'])).

% % % % % % % % % % % % % % % ASK ANSWER % % % % % % % % % % % % % % %

transition(ask(_, _, _), start, speak).
transition(ask(Sentence, _, _), speak, listen) :-
    achieve(say(Sentence)).
transition(ask(_, Options, Answer), listen, end(ok)) :-
    achieve(listen(Options, Answer), ok).
transition(ask(_, Options, Answer), listen, speak) :-
    achieve(listen(Options, Answer), fail),
    achieve(say('Hmmm... I did not hear what you said. ')).

transition(_, State, State).

state_machine(find_person(_, _)).
state_machine(learn_person(_PersonID)).
state_machine(find_object(_ObjectID, _ObjectQuery, _Waypoints)).
state_machine(ask(_, _, _)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                             ACHIEVERS                                %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

achieve(position(amigo, Waypoint), check, _) :-
    waypoint(Waypoint, pose_2d(X, Y, Phi)),
    property_expected(amigo, pose, pose_2d(X2, Y2, Phi2)),
    abs(X-X2, DX), abs(Y-Y2, DY), abs(Phi-Phi2, DPhi),
    DX < 0.1, DY < 0.1, (DPhi < 0.3; DPhi > 6.0).
achieve(position(amigo, Waypoint), solution, _) :-
    waypoint(Waypoint, pose_2d(X, Y, Phi)),
    add_action(navigate_to(X, Y, Phi, '/map')).

achieve(say(TextList), check, _) :-
    terms_to_atoms(TextList, TextAtoms),
    atomic_list_concat(TextAtoms, Text),
    add_action(say(Text)).
achieve(say(Text), check, _) :-
    add_action(say(Text)).

achieve(listen(_, Words), check, _) :-
    heard_words(Words),
    retractall(heard_words(_)).
achieve(listen(Options, _), solution, _) :-
    retractall(heard_words(_)),
    add_action(listen(Options)).
    
achieve(module_status(Module, Status), check, _) :-
    add_action(toggle_module(Module, Status)).

% look_at
achieve(look_at(point_3d(X, Y, Z)), check, _) :-
    add_action(look_at(X, Y, Z, '/map')). % assume call is blocking
achieve(look_at(point_3d(X, Y, Z, FrameID)), check, _) :-
    add_action(look_at(X, Y, Z, FrameID)). % assume call is blocking

% wait
achieve(wait(Seconds), check, _) :-
    add_action(wait(Seconds)). % assume call is blocking

% grab
achieve(grab(ObjectID), check, _) :-
    add_action(grab(ObjectID)).  % assume call is blocking

% set-up sub-state print_state_machine
achieve(Goal, check, Status) :-
    current_state(Goal, end(Status), _), !,
    retractall(current_state(Goal, _, _)).
achieve(Goal, solution, _) :-
    state_machine(Goal),
    get_new_priority_level(Priority),
    add_state(Goal, start, Priority).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                      %
%                               ENGINE                                 %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

step(Actions, Transitions, Warnings) :-
    retractall(action(_)),
    retractall(warning(_)),
    list_states(States), !,
    do_transitions(States, Transitions),
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

do_transitions([], []).
do_transitions([state(Machine, State, Priority)|States], [Transition|Transitions]) :-
    % since states may have been removed, check if state still exists
    current_state(Machine, State, Priority),
    do_transition(state(Machine, State, Priority), Transition),
    do_transitions(States, Transitions).
do_transitions([state(Machine, State, Priority)|States], Transitions) :-
    % since states may have been removed, check if state still exists
    not(current_state(Machine, State, Priority)),
    do_transitions(States, Transitions).

do_transition(state(Machine, State, Priority), transition(Machine, State, NewState)) :-
    transition(Machine, State, NewState), 
    retractall(current_state(Machine, State, Priority)), 
    add_state(Machine, NewState, Priority), !,
    nl, write('Transition: '), write(Machine), write(':'), write(State), write(' ---> '), write(NewState), nl.
do_transition(state(Machine, State, _), transition(Machine, State, not_found)) :-
    add_warning(['Could not find a transition for state ', Machine, ':', State]).


%make_true(position(amigo, pose_2d(X, Y, Phi, FrameID))) :- false. % TODO

achieve(Goal) :-
    achieve(Goal, ok).

achieve(Goal, Status) :-
    nl, write('Goal checked: '), write(Goal), nl,
    achieve(Goal, check, Status)
    ->
        (nl, write('Goal achieved: '), write(Goal), nl,
        retractall(current_goal(Goal))
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
        (   achieve(Goal, solution, _)
            ->
                assert(current_goal(Goal))
            ;
                add_warning(['No valid solution for goal ', Goal])
        )
    .

add_state(Machine, State, Priority) :-
    nl, write('ADDING STATE: '), write(Machine), write(':'), write(State), nl,
    assert(current_state(Machine, State, Priority)).

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
