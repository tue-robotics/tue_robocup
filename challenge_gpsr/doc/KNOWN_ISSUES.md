# Known issues of GPSR with the LLM task planner

Found while building and sim testing the `feature/llm` branches. Each entry says whether it is a bug (the code does
something wrong) or a limitation (the design cannot do it yet), and where it is planned.

| # | Issue | Kind | Planned in |
|---|---|---|---|
| 1 | A coke can is not recognised as a drink | Limitation (detector) | 5b: a model that names the competition objects |
| 2 | HERO starts fetching while its gripper is full | Bug | Honest outcomes and feedback (next after people) |
| 3 | A new command does not know the previous one | Limitation (planning) | Honest outcomes and feedback |
| 4 | After an explicit navigate-to step HERO forgets what it holds | Bug | Honest outcomes and feedback |
| 5 | Messages say "a food" instead of the object found | Bug (cosmetic) | Fixed in the people part |
| 6 | Localization drifts in simulation | Limitation (known) | Not planned; sim tests judge what they validated |
| 7 | Some tests fail without roscore, `openai`, `telegram_ros`, or are broken | Test setup | Not planned |
| 8 | Finding a person crashed on HERO (`rightArm`) | Bug | Fixed in the people part |

## 1. A coke can is not recognised as a drink

COCO, the classes of our YOLO model, has no "can": there is no such label in `coco.names` or `coco.yaml`, the label map
or the object list. In the simulation the coke can was measured as `cup` (confidence 0.41). `cup` is an object of the
impuls list (category container), so the entity becomes a cup, not a drink, and "bring me a drink" cannot find it.
Mapping `cup` to drink would make every real cup a drink, so this is not fixed by an alias. It needs a detector that
knows the object: YOLOE with the prompt "coke can" as an alias of coke, or a model fine-tuned on the competition
objects (LLM_PLANNING_DISCUSSION.md section 5b).

## 2. HERO starts fetching while its gripper is full

When HERO still holds an object from an earlier command and is asked to pick up another one, it drives to the source
and inspects it, and only then fails to grab. Pick-up asks for a free arm (`UnoccupiedArmDesignator`) when it is set
up, but nothing checks that one exists (`action_server/.../actions/pick_up.py`, configure), so the failure only comes
when Grab runs, after the find. Wanted: check when the task is set up and either say "my gripper is full" or first put
the held object down or hand it over.

## 3. A new command does not know the previous one

Example: "bring me an apple from the dining table" fails (no apple found); then "a banana then?" is planned on its own,
so the planner asks what to do with the banana; after "yes, bring it to me" it plans "bring a banana" from the usual
place of food (the closet), not from the dinner table of the first command. Each command is planned without the earlier
ones, and a failed step is not reported to the planner. Wanted: give the planner the recent dialogue and the outcome of
the last task (feedback), so that "a banana then?" means "the same, with a banana".

## 4. After an explicit navigate-to step HERO forgets what it holds

"Pick up the coke; go to the cabinet; place it there" asks "What would you like me to pick up?": a step only inherits
the context of the step before it, and navigate-to does not pass on the arm and the object. Pinned as an expected
failure in `action_server/test/test_recipe_configuration.py` (`test_an_explicit_drive_keeps_the_held_object`). The
planner instructions tell it not to add such drives, so plans rarely hit it.

## 5. Messages say "a food" instead of the object found (fixed)

"I found a food", "I brought a food from dinner_table to operator", while HERO picked up the apple. Find and hand-over
named the object by the description. Fixed in the people part: they name the entity they found or hold ("I found a
apple"), and fall back to the description when there is none.

## 6. Localization drifts in simulation

The localization can drift in the simulation, so navigation never arrives and a test does not finish. Known limitation;
sim tests are judged on what they validated, and the whole sim is restarted before the next run. Seen on 2026-11-26:
a few minutes after start the `map` frame disappears entirely (`tf_echo map hero/base_link`: "map does not exist")
and navigation only reports "Received an empty transformed plan". Running a test right after starting, and keeping
the part under test free of driving where possible, works around it.

## 7. Some tests fail without roscore or `openai`

- `robot_smach_states`: `test_answer_questions`, `test_human_interaction` and one doctest of `HearOptionsExtra` call the
  NER parser of hmi, which reads a ROS parameter and needs a roscore.
- `action_server/test/test_llm_planner.py`: 6 tests need the `openai` package.
- `challenge_serving_drinks/test`: construction fails, `robot_smach_states.Initialize` no longer exists.
- `challenge_final/test`: needs `telegram_ros`, which is not installed here.

## 8. Finding a person crashed on HERO (fixed)

Find pointed at a found person with `robot.rightArm`, which only AMIGO has, so every successful person search through
the action server ended with an AttributeError on HERO. It was hidden because for an unnamed person Find never
resolved the person it found. Found in simulation; HERO now turns to the person and says so.
