# Responsible: Sam

## Testing

Launch procedure:
```
amigo-start
amigo-free-mode
rosrun challenge_set_a_table challenge_set_a_table.py amigo
```

## CHALLENGE PLAN

	# PART I
	- enter the arena and go to the designated position
	+ hear: "Set the table!"
	+ say: "What should I serve, master?"
	- receive order and proceed, with respect to it
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
2. Check for ready available functions and implement them
3. Make the missing functions
