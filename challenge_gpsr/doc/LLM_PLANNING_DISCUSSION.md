# LLM task planning for GPSR: knowledge, perception and plan representation

Running record of the design discussion about the LLM task planner (`action_server.llm`) for GPSR: what the robot
knows, where that knowledge comes from, what limits the tasks it can do, and the plan to improve it. Newest progress is
at the bottom, in [Progress log](#7-progress-log).

Last updated: 2026-11-24

## 1. Current setup

| Repository | Branch | Note |
|---|---|---|
| action_server | `feature/llm` | Planner in `action_server/src/action_server/llm/`. Settings in `action_server/.env/llm.env` (not committed). |
| conversation_engine | `feature/llm` | `parser: llm` uses the planner, `llm_fallback_to_ner` falls back to the NER model. |
| tue_robocup | `feature/llm` | Includes `fix/gpsr-python-interpreter` (`SegmentObjects` uses the YOLO labels instead of ED's removed `classify` service). `robocup_knowledge`: detector label map (section 5). `Find`: category matching. |
| hero_bringup | `feature/llm` | `parameters/interaction/conversation_engine.yaml` (parser), `parameters/llm/task_planner.md` (instructions for the model), rgbd plugin: label map and threshold. |
| ed_sensor_integration | `feature/llm` (from `master`) | Detector labels into the object list, vote per entity. |
| ed_python | `feature/llm` (from `master`) | `Entity.existence_probability`. |
| yolo_onnx_ros | `feature/llm` (from `ros1`; `master` is ROS 2) | Configurable confidence threshold. |
| robot_launch_files | `feature/llm` (from `ros1`; `master` is ROS 2) | Writes the detector label map before ED is configured. |

To switch all of them, from the `tue_robocup` repository:

```bash
challenge_gpsr/scripts/checkout_llm_branches.sh --check   # only report
challenge_gpsr/scripts/checkout_llm_branches.sh           # check, then check out feature/llm; then tue-make
```

It checks every repository first and checks out nothing when one has uncommitted changes, commits that are not on
GitHub (except on `feature/llm` itself), a detached HEAD, or no `feature/llm` branch.

For development the planner runs on the Claude Code CLI with its own login (`LLM_PROVIDER=claude_code`, `LLM_MODEL=sonnet`,
`LLM_TIMEOUT=60`, `LLM_EFFORT=low`). A command takes about 3 to 4 s and 5,200 tokens.

Running GPSR in simulation:

1. `hero-start`
2. `hero-free-mode`. This also starts the action server (node `/state_machine`, log in `~/.ros/log/latest/state_machine-*.log`).
   Do not start a second action server: both would serve `/hero/action_server/task`.
3. `hero-rviz`
4. `rosrun challenge_gpsr gpsr.py _robot_name:=hero _test_mode:=true _skip:=true _number_of_tasks:=3 _time_limit:=5 _parser:=llm _llm_fallback_to_ner:=false`
5. Give a command: `hero-hear could you get me a pepsi`

In simulation the HMI also runs `random_answerer` (`hmi/tools/random_answerer`, started when `ROBOT_REAL` is not set),
which answers GPSR's questions with random grammar sentences and races typed commands. Stop it after free mode is up:
`rosnode kill /hero/hmi/random_answerer`.

Decisions so far:

- **No waypoints in the planner's vocabulary.** GPSR commands name rooms, furniture, people and objects. Returning to
  the instruction point is the challenge's job, and "come back to me" is `navigate-to operator`. A tried change that
  exposed ED's waypoints was reverted. Named doors (the rulebook's Entrance and Exit) belong in the static place list
  instead.
- **Step 1 is connecting the detector to the object list** (section 5).

## 2. Where GPSR's knowledge comes from

1. **`robocup_knowledge/environments/<ROBOT_ENV>/common.py`**, written by hand per arena: names, rooms, furniture
   (room, category, manipulable), objects (name, category, colour, size, weight), the usual location per object category
   and inspect areas. The GPSR grammar, the NER entity linker and the LLM planner are all generated from it.
2. **`challenge_gpsr.py`** in the same folder: the grammar, and the waypoints the challenge itself drives to
   (`initial_pose`, `gpsr_meeting_point`, `gpsr_exit_door_2`).
3. **ED at runtime**: geometry, areas, and what is seen. Actions resolve names to ED entities. Objects get their type
   from the YOLO label (`ed_sensor_integration` kinect plugin: `req.setType(id, cluster.label)`).

For `impuls`, ED and `robocup_knowledge` agree: every room and piece of furniture of the knowledge exists in ED.

## 3. Findings

### 3.1 Our YOLO model has 80 classes (COCO)

Evidence, all from `build/yolo_onnx_ros/resources/yolo26m/`:

- The model's metadata: "Ultralytics YOLO26m model trained on .../coco.yaml" and the class list
  `{0: 'person', ..., 79: 'toothbrush'}`.
- The weights: the last layer of the class branch at each of the three scales (`model.23.one2one_cv3.{0,1,2}.2.weight`)
  has shape `[80, 256, 1, 1]`. The network outputs exactly 80 class scores.
- Inference with onnxruntime: output `[1, 300, 6]`; over 7 inputs (3 noise images, 4 real images) of 300 detections
  each, the highest class id was 79.
- The names file is optional because the end-to-end output gives a class *number*. Without `coco.names` the label is
  empty (`sam_seg_module.cpp` only looks a name up when it exists), and ED gives the object no type at all.

Where larger numbers come from: YOLO26 *classification* models have 1,000 ImageNet classes, but they give one label per
image or crop, without boxes or masks. YOLOv8 has Open Images V7 detectors (601 classes); I found no YOLO26 version.
YOLOE-26 (open vocabulary) detects the names it is given as text, or a built-in list of 4,585 names in prompt-free mode.

### 3.2 The object list and the detector do not match

| Object list | Objects | Also a COCO class |
|---|---|---|
| `impuls` (`robocup_knowledge`) | 36 | 8: apple, banana, bowl, cup, fork, knife, orange, spoon |
| RoboCup 2026 Incheon (official) | 34 | 6: apple, cup, spoon, knife, fork, bowl |

The planner can plan "bring me a coke", but `Find` matches the requested type against the YOLO label, which for a coke
is at best "bottle". Commands about objects outside COCO cannot succeed, however good the planner is.

The fix is to change the detector to the object list, not the list to COCO. The 2027 rulebook (§3.2.5) defines the
objects: at least 30, each with "a picture, an official name, and an Object Category", from YCB, four worldwide objects
(Red Bull can, red Pringles container, solved Rubik's cube, Colgate toothpaste box) and "Local household items bought
during the competition", published on the first setup day. "Similar Objects" also count, and objects can be requested by
category and by colour, relative size, relative weight, relative position and description.

The 2024 winner (NimbRo) fine-tuned YOLO / MaskDINO per task on "63,516 object instances from 104 classes", labelled
interactively with SAM, and switched to open-vocabulary detection (mmGrounding-DINO) where that was enough.

### 3.3 What a plan step can express

A step is now an action and five roles (`object`, `person`, `source`, `destination`, `sentence`), and an entity is
`EntityDescription(id, type, location, area, category)`. The current official GPSR generator (23 command types) also
needs:

| Command types | Needs | Status |
|---|---|---|
| goToLoc, takeObjFromPlcmt, bringMeObjFromPlcmt, findObjInRoom, place | navigate, find, pick, place, hand-over | Supported |
| findPrsInRoom, followPrsAtLoc, guidePrsFromBeacToBeac with a gesture or pose | find a person by attribute | Perception has it (`Person3D`: `posture`, `tags` such as LWaving/RWaving, `shirt_colors`, `pointing_pose`; `FindPeople(properties=...)` filters on them). Action server and planner have no field for it. |
| greetClothDscInRm, countClothPrsInRoom | clothing colour and garment | Colour yes, garment no |
| countPrsInRoom | count people matching a filter | Missing (`count-and-tell` counts objects) |
| tellObjPropOnPlcmt, tellCatPropOnPlcmt | compare objects (biggest, heaviest, thinnest, ...) | Missing |
| tellPrsInfoInLoc, tellPrsInfoAtLocToPrsAtLoc | pass the result of one step to a later one | Missing |
| answerToGestPrsInRoom | answer a quiz question | `answer-question` needs SPR knowledge; the LLM can answer itself |

Descriptor lists of the generator: gestures (waving; raising left/right arm; pointing left/right), poses (sitting,
standing, lying), clothes (7 colours × 6 garments), person info (name, pose, gesture), object comparisons (biggest,
largest, smallest, heaviest, lightest, thinnest). The 2027 rulebook adds: commands are rephrased by an LLM up to 3 times,
and there is a bonus for interleaving 3 commands that are given at once.

Ways to represent plans in the literature:

1. **Fixed frames / slot filling** (what we have). Checkable, but limited to the slots. SocRob (2nd in GPSR 2024) used a
   fine-tuned Phi-3.
2. **Typed function calls over a skill API.** ProgPrompt (Python-style header of actions and objects), the 2024 GPSR
   paper with exception handling (arXiv 2405.15646), NimbRo's function calling ("about 3 to 15 functions", executed "one
   function after another" with "appropriate textual feedback").
3. **Code as Policies**: the LLM writes Python with loops and conditionals. Most expressive, hardest to check before it
   moves a robot.
4. **LLM+P**: the LLM writes a PDDL goal and a classical planner plans. Good for interleaving, but needs a hand-written
   domain and handles perception-driven tasks poorly.
5. **Behaviour trees** (BTGenBot, BTGenBot-2, LLM-as-BT-Planner): reactive, with fallbacks; fits ROS.

Proposal: a restricted, typed plan language between 1 and 2.

- Entities are descriptions, not only names, with attribute values from the rulebook's lists, e.g.
  `{kind: person, location, attributes: {gesture: raising_left_arm, pose, clothing: {color, garment}}}` and
  `{kind: object, category, attributes: {color, rank: biggest, relative_position}}`. The schema still rejects anything
  unknown.
- Steps can store results that later steps use: `p1 = find(person{gesture: waving}, in: living_room)`,
  `n = ask(p1, info: name)`, `say(to: p2, text: n)`.
- Classify the command type first, then plan with that type's schema (LLM chaining, arXiv 2609.29043: 28 task types,
  about 45% shorter prompts, better consistency; only the abstract was read).
- Validate with the action server's `configure` step and send errors back to the model.

### 3.4 Static and dynamic knowledge

Three different needs:

1. **Resolve at execution time.** "The biggest drink on the dinner table" becomes
   `object{category: drink, rank: biggest}`, and `find` picks the object from what it sees. Most GPSR commands are like
   this; this needs the descriptors of 3.3, not world access during planning.
2. **World state during planning**, when the answer depends on the current state ("is there a coke in the kitchen?",
   "bring me something to drink", interleaving three commands, what HERO is holding). Either a snapshot of ED in the
   prompt (simple, grows and goes stale; SayPlan's semantic search cut the scene given to the LLM by up to 82.1%) or tools
   the LLM calls (`find_seen_objects`, `where_was`, `robot_state`), ReAct-style. The `claude_code` backend runs the CLI
   without tools, so tools need the Messages API backend or an MCP server for ED.
3. **Feedback during execution.** Each action's result goes back to the LLM, which re-plans or asks
   (Inner Monologue: "significantly improves high-level instruction completion"; SayPlan's replanning: "near-perfect
   executability"). Today a failed step ends the task.

The structure for this is a layered world model: static rooms and places, dynamic objects and agents (Hydra's 3D scene
graph; SayPlan, ConceptGraphs and DovSG plan with such graphs). It matches the rulebook: room names, predefined
locations, location classes, object names and categories, and person names are announced on the setup days; where
objects and people are changes during a test.

- *Static, per arena:* rooms; furniture (room, class, manipulable, areas); the Entrance and Exit doors; the object list
  (name, category, prompt or picture, properties such as fragile or pourable); category to predefined location; person
  names. `robocup_knowledge/<env>/common.py` is roughly this already.
- *Dynamic, ED at runtime:* object instances (type, on which furniture, colour, size, last seen), people (name if known,
  posture, gesture, clothes, position) and the robot's state.

The object *names* stay static: the robot must be able to plan "bring me a Red Bull from the kitchen" before it has seen
one. Only which objects exist where comes from perception.

### 3.5 Actions offered to the LLM

The action server has 20 actions; the planner offers 11. Offer every task-level skill, with a typed signature, its
preconditions (e.g. holding an object) and its effects:

- Leave out the internal ones: `arm-goal`, `gripper-goal`, `reset-wm`, `demo-presentation`, `send-picture`.
- Fix, then offer: `look-at`, `point-target`, `turn-toward-sound`, `answer-question` (answer with the LLM),
  `open-door` (commented out today).
- Add what the generator needs: count people with a filter, report a person's name/pose/gesture, compare objects,
  greet or introduce to a described person, relay information between people.

## 4. Order of work

1. Connect the detector to the object list (section 5). Fine-tuning on the announced objects comes later.
2. Descriptor attributes for entities (schema, action server `EntityDescription`, `find`), starting with people.
3. Execute one step at a time with feedback.
4. Step results (variables) and the missing actions.
5. A world-state query for planning; plan the three commands together for the interleaving bonus.

## 5. Detector labels and the object list (implemented)

Decided on 2026-11-25 (design session Q1 to Q27), implemented on the `feature/llm` branches.

**Principle.** The object list (`robocup_knowledge/environments/<ROBOT_ENV>/common.py`) is the truth. Every label of the
detector is translated once, where it enters ED, into an object of the list, a category of the list, *drop*, or no type.
ED then only holds names of the object list, so `Find`, the challenges and the planner speak one vocabulary. The same
design takes a model with more classes, a fine-tuned model (labels equal the names) or an open-vocabulary model (labels
generated from the names): only its files and its configuration change.

```
 launch: common.py (ROBOT_ENV) ──► detector_label_map ──► ~/.ros/detector_label_map.yaml ──► ED rgbd plugin
 run:    camera ─► YOLO "bottle" ─► map: drink ─► vote per entity ─► ED type "drink"
         GPSR / action_server / Find read common.py directly and compare with the ED types
```

| Part | Repository | What |
|---|---|---|
| Object list | tue_robocup `robocup_knowledge` | Optional fields in `common.py`: `aliases` on objects ("cola" for "coke"), `aliases` on locations (furniture labels, dropped), `category_aliases` (`drink: ["bottle", "wine glass"]`), `detector_ignore` (`["person"]`). Set for `impuls`. |
| Label map | tue_robocup `robocup_knowledge` | `detector_labels.py` and `rosrun robocup_knowledge detector_label_map`: builds the map (normalised: lower case, spaces, `-` and `_` alike); a label with two different meanings is an error, furniture sharing a label is not; an object named like its category wins. `--labels <model>/coco.names` prints what a model matches and which objects it cannot name; `--out <file> -- <command>` writes the map and runs the command. |
| Launch | robot_launch_files | `world_model.launch`: `ed_configure` runs through `detector_label_map`, so the map is current before ED is configured. If the map cannot be made, ED is configured anyway and uses the model's labels. |
| ED plugin | ed_sensor_integration | `labels.h/.cpp`: the map, and a vote per entity (confidence-weighted, totals fade by 0.9 per detection, a tie goes to the latest; a type that loses is removed from the entity). `sam_seg_module.cpp` translates labels after YOLO and drops boxes before SAM. `surface_label_map` removed (the drop list replaces it). |
| Threshold | yolo_onnx_ros, hero_bringup | `yolo_confidence_threshold` (0.4) in `world_model_plugin_rgbd.yaml`, used by both backends. |
| Probability | ed_python | `Entity.existence_probability` (merged `feature/entity-existence-probability-update`). |
| Find | tue_robocup `robot_smach_states` | `navigation/find.py`: an entity matches a category when it is an object of it, or when its type is the category itself. A named object stays strict: "the coke" never accepts a `drink`. |

Check what a model can tell during the setup days:

```bash
rosrun robocup_knowledge detector_label_map --labels $TUE_ENV_WS_DIR/build/yolo_onnx_ros/resources/yolo26m/coco.names
```

For COCO and `impuls`: 14 of the 80 labels match (8 objects: apple, banana, orange, bowl, cup, fork, knife, spoon;
`bottle` and `wine glass` → drink; `person`, `dining table`, `couch`, `refrigerator` dropped). 28 objects cannot be
named by this model.

Tests:

- `robocup_knowledge/test/test_detector_labels.py` (13): normalisation, objects, categories, drops, ambiguity, and the
  map of every environment.
- `ed_sensor_integration/test/test_labels.cpp` (7): normalisation, loading the map, and the vote (10× drink + 1× dog
  stays drink; a tie goes to the latest; a cup replaced by a bowl becomes a bowl after at most 6 detections).
- `robot_smach_states/test/navigation/test_find.py` (3): category matching and strict named objects.

Results in simulation (`impuls`): see the progress log.

Known limits:

- COCO is coarse: in simulation it sees the coke can as `cup` (0.41) and only weakly as `bottle` (0.05), and no banana
  at all from the inspection view. "Bring me the coke" cannot work with this model; that needs one of the models below.
- The existence probability is the winner's share of the vote: an entity seen with one type only gets 1.0, however low
  the detection confidence was. The threshold filters weak detections.
- The planner only uses categories in `find`; "bring me something to drink" makes it choose one drink by name.

## 5b. Next: a model that names the competition objects

The design above takes these as configuration:

- **Open vocabulary (YOLOE-26).** `ultralytics` 8.4.14 (installed) has a detection-only config `yoloe-26.yaml`;
  `set_classes(prompts)` and the exporter bake the prompts into an ordinary detector with one `[1, 300, 6]` output, which
  our C++ already reads. Prompts come from the object names, or from aliases (a descriptive prompt such as "red bull
  energy drink can" as alias of `red_bull`), so the labels map back through the label map. Check per object with the
  check command. Brand items may be weak; every official object comes with a picture for visual prompts.
- **Fine-tuned on the announced objects.** The labels are the object names; the label map matches them by name.

## 6. Object lists

**2027:** not published yet. The rulebook (2027 Rev-1 draft, §3.2.5) says the TC publishes at least 30 objects on the
first setup day. The minimal set of known objects it requires:

- Tableware: dish, bowl, cup (or mug), napkin
- Cutlery: fork, knife, spoon
- Bags (lightweight, stiff vertical handles); trays or baskets for two-handed manipulation
- A pourable object (such as a cereal box); a heavy object (1.0 to 1.5 kg); a tiny object (5 cm or smaller); a fragile
  object; a deformable object (cloth); laundry (a T-shirt); a laundry basket
- Common objects, available worldwide: Red Bull can, red Pringles container, solved Rubik's cube, Colgate toothpaste box
- YCB objects, and local household items bought during the competition

**Most recent official list: RoboCup 2026 Incheon**
([objects.md](https://github.com/RoboCupAtHome/Incheon2026/blob/main/objects/objects.md), 34 objects):

| Category | Objects |
|---|---|
| fabrics (fabric) | white_shirt, grey_shirt, blue_shirt, black_shirt, hand_towel |
| toys (toy) | rubiks_cube |
| snacks (snack) | pringles, seaweed |
| fruits (fruit) | apple, peach, mangostane, lemon, yellow_bellpepper, red_bellpepper |
| food (food) | instant_noodles, cornflakes |
| drinks (drink) | coke, red_bull, milk, soju, pepsi |
| cleaning_supplies (cleaning_supply) | dishwasher_tab, sponge, toothpaste |
| dishes (dish) | cup, spoon, plate, knife, fork, bowl |

Other competition data in the same organization: [GermanOpen2026](https://github.com/RoboCupAtHome/GermanOpen2026),
[PortugalOpen2026](https://github.com/RoboCupAtHome/PortugalOpen2026), [Salvador2025](https://github.com/RoboCupAtHome/Salvador2025),
[Eindhoven2024](https://github.com/RoboCupAtHome/Eindhoven2024). The command generator reads a competition folder in the
[CompetitionTemplate](https://github.com/johaq/CompetitionTemplate) layout (`objects/objects.md`, names, maps).

## 7. Progress log

- 2026-11-24: Planner runs in simulation with the `claude_code` backend; GPSR plans and executes "grab the apple from
  the dining table" after merging the YOLO/SAM `SegmentObjects` fix. Waypoint vocabulary tried and reverted. Design
  discussion recorded (section 3). Plan for the detector written (section 5); nothing implemented yet.
- 2026-11-25: Design of the detector labels decided (Q1 to Q27) and implemented on the `feature/llm` branches of eight
  repositories (section 5). Unit tests: 13 Python (label map, every environment), 7 C++ (map and vote), 3 + 3 Python
  (`Find`, `SegmentObjects`). In simulation, the map is written at launch and loaded by ED (59 labels; 14 of the 80
  COCO labels match `impuls`). Recipes sent to the action server, same order as a baseline run before the change:

  | Recipe | Before | After |
  |---|---|---|
  | find apple on the dinner table | ✓ | ✓ |
  | find a drink on the dinner table | ✗ | ✗ (COCO sees the coke can as `cup`, 0.41) |
  | find the coke on the dinner table | ✗ | ✗ (expected: a named object is strict) |
  | pick up the apple, place it on the salon table | ✗ (arm) | ✓ |

  GPSR with the LLM (Claude Code): "is there some food on the dinner table" → `find {category: food}` → succeeded
  ("I found a food"). "put the apple from the dinner table on the salon table" → found and grabbed the apple, then the
  drive to the salon table hung; "grab the apple from the dining table" → the drive to the dinner table hung. Both hangs
  are the known navigation/localisation issue, not the labels.

## Sources

- RoboCup@Home: [Rulebook 2027 Rev-1 (draft)](https://robocupathome.github.io/RuleBook/rulebook/master.pdf),
  [official CommandGenerator](https://github.com/RoboCupAtHome/CommandGenerator),
  [gpsr_commands.py](https://github.com/johaq/CommandGenerator/blob/master/gpsr_commands.py),
  [Incheon 2026 objects](https://github.com/RoboCupAtHome/Incheon2026/blob/main/objects/objects.md)
- Ultralytics: [YOLO26](https://docs.ultralytics.com/models/yolo26/), [YOLOE](https://docs.ultralytics.com/models/yoloe),
  [Open Images V7](https://docs.ultralytics.com/datasets/detect/open-images-v7),
  [YOLOE paper](https://arxiv.org/abs/2503.07465)
- GPSR systems: [NimbRo@Home 2024](https://arxiv.org/abs/2412.14989),
  [GPSR planning with exception handling](https://arxiv.org/abs/2405.15646),
  [LLM chaining for GPSR](https://arxiv.org/abs/2609.29043),
  [Taming the LLM](https://link.springer.com/chapter/10.1007/978-3-032-09037-9_24),
  [SocRob GPSR](https://fenix.tecnico.ulisboa.pt/downloadFile/2533669927387761/extended_abstract.pdf)
- Planning: [Code as Policies](https://arxiv.org/abs/2209.07753), [ProgPrompt](https://progprompt.github.io),
  [LLM+P](https://arxiv.org/abs/2304.11477), [Inner Monologue](https://arxiv.org/abs/2207.05608),
  [SayCan](https://arxiv.org/abs/2204.01691), [ReAct](https://arxiv.org/abs/2210.03629),
  [BTGenBot](https://arxiv.org/abs/2403.12761), [BTGenBot-2](https://arxiv.org/abs/2602.01870),
  [LLM-as-BT-Planner](https://arxiv.org/abs/2409.10444)
- World models: [SayPlan](https://arxiv.org/abs/2307.06135), [Hydra](https://arxiv.org/abs/2201.13360),
  [ConceptGraphs](https://arxiv.org/abs/2309.16650), [DovSG](https://arxiv.org/abs/2410.11989),
  [OK-Robot](https://arxiv.org/abs/2401.12202)
