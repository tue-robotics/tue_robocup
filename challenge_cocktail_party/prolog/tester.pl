:- module(tester, [open_door/0,
                   close_door/0,
                   reset_wire/0,
                   add_person/0,
                   say/1]).

open_door :-
    query((retractall(state(entrance_door, _)),
           assert(state(entrance_door, open)))).

close_door :-
    query((retractall(state(entrance_door, _)),
           assert(state(entrance_door, close)))).

add_person :-
	wire:add_object_evidence(john, person, 2, 1, 1).

say(X) :-
	query(assert(heard_words(X))).

reset_wire :-
	write('Currently not implemented.'), nl.
