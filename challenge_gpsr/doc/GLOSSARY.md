# HERO general-purpose service robot

The language of HERO doing what a person asks: the commands it hears, the plans it makes, what it knows about the arena and what it perceives. Shared by GPSR, the task planner, the action server and the world model.

## Commands and plans

**Command**:
What a person asks HERO to do, in their own words, spoken or typed.
_Avoid_: instruction, order, sentence

**Plan**:
The ordered steps HERO makes from one command.
_Avoid_: recipe, task, semantics

**Step**:
One action of a plan with its roles (object, person, source, destination, sentence).
_Avoid_: task, subtask

**Action**:
What one step does, from the fixed set HERO offers the planner.
Example: find, pick-up, place, hand-over, say, guide.
_Avoid_: skill (when a plain action is meant), command, behaviour

**Skill**:
An action that runs a whole tested routine of an existing challenge inside it.
Example: "open the dishwasher" from Clean the Table, "pour cereal" from Serve Breakfast.
_Avoid_: macro, state machine (when the planner's view is meant)

**Sub-goal**:
One part of a long task that the planner plans and checks on its own, stated as the situation to reach.
Example: for loading the dishwasher, "the dishwasher is open", then "all cups are in the rack".
_Avoid_: subtask, phase, milestone

**Description**:
What a step says about the object or person it needs, without naming one entity. HERO picks the entity that fits when it sees what is there.
Example: "grab a fruit from the dinner table" has the description {category: fruit}; "guide the waving person in the kitchen" has {gesture: waving} in the kitchen.
_Avoid_: descriptor, filter, query

**Attribute**:
One property in a description, with a value from a fixed list taken from the rulebook.
Example: category = fruit, gesture = waving, pose = sitting, clothing colour = red.
_Avoid_: descriptor, property, tag

## Arena knowledge

**Object list**:
The objects, categories, furniture, rooms and person names of an arena, as announced on the setup days. It is the truth for every name HERO uses.
_Avoid_: vocabulary, knowledge, ontology

**Object**:
A kind of thing in the object list that HERO can handle, with one name and one category ("coke", a drink).
_Avoid_: item, class

**Category**:
A group of objects in the object list ("drink", "fruit").
_Avoid_: class, kind

**Furniture**:
A fixed thing in a room on or in which objects are found or placed ("dinner_table").
_Avoid_: placement location, location (when furniture is meant)

**Alias**:
Another word for an object, a category or a piece of furniture, such as a detector's word for it ("dining table" for dinner_table).
_Avoid_: synonym

## Perception

**Detector**:
The model that finds things in a camera image and names each with a detector label.
_Avoid_: classifier, YOLO (when the role is meant, not the model)

**Detector label**:
The detector's own word for what it found ("bottle", "dining table").
_Avoid_: class, class name, type

**Label map**:
What each detector label means in the object list: an object, a category, drop, or nothing. It depends on the object list only, never on the detector.
_Avoid_: label mapping, translation table

**Drop**:
What a detector label means when the thing is not to be kept: a person or furniture.
_Avoid_: ignore, filter

**Undetectable object**:
An object that no detector label names; HERO can at most find it through its category.

**Entity**:
One thing in HERO's world model: an object, a piece of furniture, a room or a person.
_Avoid_: instance, segment, cluster

**Entity type**:
The object or category an entity is known as; empty when the detector could not tell.
_Avoid_: label, class, etype

**Vote**:
The choice of an entity's type over all its detections, so that one wrong detection does not change it.
