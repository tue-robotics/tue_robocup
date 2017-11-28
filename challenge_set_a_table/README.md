# Responsible: Sam

## Testing

Setup
* Put the robot in front of the door (the usual initial pose)
* Make sure one of the combos is on the cabinet (look at the RoboCup knowledge for the possibilities). N.B., only the 'easy' items are relevant.
* When the robot asks what to do, answer 'Set the table'. The rest should speak for itself.

Launch procedure:
```
amigo-start (usually already running)
amigo-challenge-set-a-table
```

## CHALLENGE PLAN

	# PART I
	+ enter the arena and go to the designated position
	+ hear: "Set the table!"
	+ say: "What should I serve, master?"
	- receive order and proceed, with respect to it:
			- meal 1: breakfast
				- cereal box (easy)
				- milk box (easy)
				- glass (easy)
				- bowl (hard)
				- spoon (hard)
				- napkins (hard)
			- meal 2: lunch
				- glass (easy)
				- juice (easy)
				- plastic steak (easy)
				- bread (easy)
				- plate (hard)
				- knife (hard)
				- fork (hard)
				- napkins (hard)
		- set the table:
			- inspect the table (see what's missing)
			- grab the needed objects from their default place (3 easy & 3 hard to grasp objects)
			- open door to grab objects???
			- place the objects on the table
	- serve the meal: (2 options out of cereal and milk, sandwiches, toasts, Coffee and bread or something else)
		- pour milk in the bowl??? (We can bring our own food)
		- pour cereals in the bowl???
	- correct object positions:
		- inspect the table
		- move the objects to their original places???

	# PART II
	+ hear: "Clean the table!"
	- clear up the table:
		- grab 6 objects from table (3 easy & 3 hard to grasp objects)
		- place the objects to their default place
	- clean the table:
		- detect spots and spills???
		- clean them using a cleaning cloth???

## DONE

- scaffolding
- planning

## TODO

1. Check what of the plan is possible
2. Choose meals and food
3. Check for ready available functions and implement them
4. Make the missing functions
