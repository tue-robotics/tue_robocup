# Hierarchical / long-horizon task planning with LLMs for household robots

Research notes, 2026-11-26. Scope: how academia, industry and RoboCup@Home teams plan long-horizon
household tasks with LLMs/VLMs, and which patterns fit HERO's action server (flat step list, ~11 skills,
no feedback to the LLM). Text in "quotes" is verbatim from the cited source. Paragraphs marked
**Inference** are this note's own reading, not claims made by the sources. Numbers come from the
sources and are self-reported.

## Summary

1. Nearly every system that works splits the work in two: a slow reasoner (LLM/VLM) picks the next
   sub-goal or skill, and a fixed set of fast, separately tested skills executes it. This holds for
   SayCan, Inner Monologue, Gemini Robotics-ER, pi0.5/Hi Robot, Helix and GR00T, and for every RoboCup team found.
2. Open-loop planning (one LLM call, then execute) is the weakest pattern. The fixes in the literature
   are closed-loop feedback (Inner Monologue, ReAct, NimbRo), precondition checks inside the plan
   (ProgPrompt), or a symbolic planner/validator (LLM+P, LLM-DP, DELTA, SayPlan).
3. RoboCup@Home winners use LLMs to turn speech into structured skill calls or goals. They do **not** let the LLM
   improvise whole manipulation tasks. NimbRo (OPL 2024 winner, 2025 runner-up) uses LLM function calling with textual feedback for
   GPSR and "hierarchical state machines whose parameters can be derived from natural language". UT
   Austin/LisTex parse into JSON and then build a state machine, or a goal for a TAMP planner.
   Hibikino-Musashi's LLM "sequentially determines the skills and targets".
4. No team found publishes an LLM-planned Clean the Table / Serve Breakfast / Storing Groceries. Those were per-task
   state machines with open-vocabulary perception.
5. The 2027 draft rulebook folds the dishwasher, trash, cabinet and breakfast work into one **Pick and Place
   Challenge**: "The robot may determine the order and method for performing all pick-and-place tasks".
   The GPSR commands are now rephrased by an LLM, and GPSR adds an interleaved-task bonus.
6. Cloud LLMs are allowed ("this includes cloud services or online APIs"), but the network is not
   guaranteed and "robots are expected to be able to function without it". A local fallback is needed.
7. Best fit for us (inference): hierarchical decomposition into sub-goals, each expanded into our
   existing flat GPSR-style plan, executed closed-loop per sub-goal, with a deterministic validator
   and a structured result after every skill. The action server needs honest failure codes,
   world-model queries, variable binding and a bounded "for each" construct.
8. Behaviour trees (BTs) and PDDL are worth it only where we can write preconditions/effects for our skills. PDDL suits
   Pick and Place, where the goal state is clear. Pure ReAct is the simplest step up from today, but
   it adds about 3-10 s of LLM latency per step (Bode et al.: 11.6 s to 72 s per GPT-4 task).

## 1. Academia

| Work | Representation the LLM emits | Failure handling / replanning |
|---|---|---|
| SayCan (2022) [1] | Next skill from a fixed set, scored by LLM × affordance value function | Implicit: re-scores at every step; no explicit recovery |
| Inner Monologue (2022) [2] | Natural-language next step | Closed loop: success detection, scene description and human feedback fed back as text |
| Code as Policies (2022) [3] | Python calling perception/control APIs (hierarchical: undefined functions are generated recursively) | Code can contain feedback loops; no plan-level replanning |
| ProgPrompt (2022) [4] | Pythonic program with `assert` precondition checks and recovery branches | Asserts check state at run time and trigger recovery actions |
| ReAct (2022) [5] | Interleaved "thought" + action text | Closed loop: reads observation after each action |
| LLM+P (2023) [6] | PDDL problem file; classical planner solves it | Planner guarantees validity w.r.t. domain; LLM alone "fail[s] to provide even feasible plans for most problems" |
| LLM-DP (2023) [7] | PDDL goal + LLM-sampled beliefs; symbolic planner | Planner + LLM-sampled beliefs updated from observations (paper body); abstract: beats a ReAct baseline on ALFWorld |
| SayPlan (2023) [8] | Plan over a 3D scene graph | "iterative replanning ... using feedback from a scene graph simulator" before execution |
| DELTA (2024) [9] | LLM splits goal into sub-goals; PDDL domain/problem per sub-goal from scene graph | "decomposes long-term task goals with LLMs into an autoregressive sequence of sub-goals"; planner solves each |
| Text2Motion (2023) [10] | Skill sequence checked for geometric feasibility | "performing geometric feasibility planning during its search" |
| LLM³ (2024) [11] | Symbolic actions + continuous parameters | "iteratively refine its proposals by reasoning about motion failure" |
| HiP (2023) [12] | Language subgoals → video plans → actions | Consistency "via iterative refinement" between levels |
| Voyager (2023) [13] | Executable code; skills stored in a growing library | "environment feedback, execution errors, and self-verification" |
| BTGenBot (2024) [14] | Behaviour tree (XML) from ≤7B fine-tuned LLMs | "static syntactical analysis, a validation system" before execution |
| LLM-as-BT-Planner (2024/25) [15] | Task decomposition → parameterised plan → BT | In-context learning/fine-tuning for executability; human feedback at runtime |
| Embodied Agent Interface (2024) [16] | Benchmarks goal interpretation, subgoal decomposition, action sequencing, transition modelling | Classifies errors: "hallucination errors, affordance errors, various types of planning errors" |

Notes:

- **SayCan** reports "an overall plan success rate of 84% and execution success rate of 74% of 101
  tasks" [1]. The robot is the LLM's "hands and eyes"; value functions "provide the grounding".
- **Inner Monologue**: "closed-loop language feedback significantly improves high-level instruction
  completion on three domains" [2]. This is the clearest evidence that a failed skill should be reported
  back to the planner rather than ending the task.
- **ProgPrompt** prompts "with program-like specifications of the available actions and objects" and
  "example programs that can be executed" [4]. Its asserts work like a validator built into the plan.
- **LLM+P / LLM-DP / DELTA** need a hand-written domain (preconditions/effects per skill). In return
  they give correct plans for the domain and scale to long horizons. DELTA reports "higher planning
  success rates and significantly shorter planning times" [9].
- **TAMP + LLM** (Text2Motion, LLM³) matters for placing in a dishwasher rack and on a cluttered table, where a
  symbolically correct plan can still be geometrically infeasible. A 2025 study found LLM planners
  in PDDLStream settings have "lower success rates and significantly higher planning times than the
  base planners" [17].
- **Benchmarks**: ALFRED [18] (ALFWorld is its text version; used by ReAct and LLM-DP), VirtualHome [19]
  (programs as action scripts; used by ProgPrompt), BEHAVIOR-1K [20] ("1,000 everyday activities",
  "long-horizon and dependent on complex manipulation skills"). EAI [16] evaluates on BEHAVIOR and VirtualHome.
- **Real-robot long-horizon 2025**: LLM-GROP (UT Austin) uses an LLM for symbolic object-arrangement
  relations in table setting/rearrangement on a mobile manipulator [21]. WildLMa chains imitation-learned skills with an
  LLM planner [22]. Neither of these shows an LLM loading a dishwasher. That is shown only end-to-end by industry (Figure, below).

**Inference:** in academia the trend from 2022 to 2025 went from "LLM writes the whole plan" to "LLM
proposes, a checker (planner, simulator, geometric feasibility, asserts, scene graph) verifies, and
feedback triggers replanning". Hierarchy usually means natural-language or symbolic sub-goals on top and a
fixed skill vocabulary below.

## 2. Industry (2024-2026)

| System | High level | Low level | Interface between them |
|---|---|---|---|
| Google DeepMind Gemini Robotics 1.5 / ER 1.5 (Sep 2025) [23] | ER 1.5 VLM "orchestrates a robot's activities, like a high-level brain" | Gemini Robotics 1.5 VLA | "natural language instructions for each step"; ER can "call tools like Google Search ... or use any third-party user-defined functions" |
| Gemini Robotics ER 2 (Jul 2026) [24][25] | ER 2 "serves as the robot's high-level brain" | Gemini Robotics 2 VLA, or robot APIs (Spot) | "We use Gemini Robotics ER 2 to orchestrate Spot APIs, such as navigation and manipulator movement" |
| Gemini Robotics-ER API docs [26] | ER model via Gemini API | Your robot functions | "executes them by calling your robot functions or running generated code"; "progress understanding and success detection" |
| Physical Intelligence pi0.5 (Apr 2025) [27] | Same model, text subtask head | Flow-matching action chunk | "first produces a high-level subtask ... predicts the low-level robot action chunk based on this subtask" |
| PI Hi Robot (Feb 2025) [28] | VLM reasoning over prompts and user feedback | pi0 VLA | Language commands; "incorporate situated feedback during task execution" |
| Figure Helix (Feb 2025) [29] / Helix 02 (Jan 2026) [30] | S2: 7B VLM at 7-9 Hz | S1: 80M policy at 200 Hz (+ S0 1 kHz whole-body in Helix 02) | Latent vector, trained end-to-end; no symbolic plan |
| NVIDIA GR00T N1 (Mar 2025) [31] | "vision-language module (System 2)" | "diffusion transformer module (System 1)" | Learned embedding, end-to-end |
| TRI Large Behavior Models (Jul 2025) [32] | none stated | Multitask diffusion policy | Abstract describes "multitask robot manipulation policies"; no planner mentioned |

Notes:

- DeepMind's agentic setup is closest to ours. A general reasoning model calls user-defined functions,
  uses success detection, and watches progress. ER 2's blog: "robots can now track their own progress, adapt if
  something goes wrong, and know exactly when to move on" [24]. The docs warn: "Generative AI models
  can make mistakes, and physical robots can cause damage", and "high thinking levels can lead to
  increased processing times" [26].
- pi0.5 shows a long-horizon learned system, "cleaning a kitchen or bedroom, in entirely new homes" [27].
  Its hierarchy is still explicit: a text subtask comes first, then actions, and "the high-level inference
  process still runs at a lower frequency than low-level action inference" [27].
- Figure Helix 02 does dishwasher unload/reload as one "four-minute" continuous autonomous run [30]
  (company claim). That is a learned end-to-end policy with no inspectable plan.

**Inference:** industry splits into (i) end-to-end learned hierarchies (Figure, NVIDIA, PI), which need
large teleoperation datasets we do not have for HERO, and (ii) an orchestrator VLM calling
skills/functions with success detection (DeepMind ER). Option (ii) maps directly onto our action
server and can use a hosted model. DeepMind's emphasis on progress/success detection suggests the
skill result channel matters as much as the planner.

## 3. RoboCup@Home teams

| Team (source) | GPSR planning | Long-horizon manipulation tasks | Model / where |
|---|---|---|---|
| NimbRo@Home, Bonn (OPL winner 2024, runner-up 2025) [33][34][35] | LLM function calling, ~3-15 calls per command, textual feedback after each call; LLM can reject infeasible commands | Clean the Table / Serve Breakfast: task pipelines with Grounding-DINO prompts, no LLM planning described; "hierarchical state machines whose parameters can be derived from natural language" | GPT-4o via 5G router (2024) |
| Hibikino-Musashi@Home (DSPL 1st 2024 & 2025, "Best in GPSR & EGPSR" 2024) [36][37] | "The LLM selects appropriate skills from the skill set and determines their targets"; "sequentially determines the skills and targets until it is completed"; 12 predefined skills | Not described as LLM-planned | GPT-4o cited |
| UT Austin Villa@Home 2025 [38] | GPT-4o parses into JSON per command-generator grammar; "a state machine is assembled based on the task type and the parameters"; parsed all 7 GPSR/EGPSR commands in 2024 | Hierarchical FSMs; knowledge base for Storing Groceries; planner used "for tasks that cannot be statically decomposed" | GPT-4o; local Whisper because of "unreliable network connectivity" |
| LisTex United (UT Austin + SocRob/ISR Lisbon) 2026 [39] | LLM (e.g. GPT-5) → "structured JSON representation of goals for the CoCo-TAMP planner"; LLM "validate[s] the resulting schema" | CoCo-TAMP with LLM commonsense priors; Yasmin state machines (ROS 2) | GPT-5 |
| TRAIL, U. Tokyo (DSPL 2024) [40] | Two-stage CoT: rough "intermediate plan", then function calling over 23 skill functions | not covered | GPT-4 |
| Self-Recovery Prompting (Tokyo authors) [41] | LLM plan + recovery from "insufficient information, incorrect plan generation, and plan execution failure" | — | — |
| JSK, U. Tokyo (Japan Open 2022 GPSR 1st) [42] | Foundation models + "a state machine task executable" | — | — |
| Tidyboy-OPL (Pusan/SNU) [43] | Fine-tuned LLM (PEFT, 60,000 generated GPSR pairs) emits ordered action list | Serve Breakfast via motion primitives | Local, RTX 3090 training |
| RoBorregos (MICAI 2025) [44][45] | Fine-tuned local model + schema-aligned parsing (BAML) + pre-execution grounding against a knowledge base; reported (chapter abstract, not read in full) 96.5% on GPSR-style commands on embedded hardware vs 90.4% cloud, 29.6% non-fine-tuned | — | Local (Ollama) or cloud |
| LAR@Home 2025 [46] | "Confirmation LLM" asks clarifying question, then "Sequencing LLM" emits actions; function calls preferred because they limit "data input based on controlled parameters" | — | ChatGPT API, moving to offline LLM |
| Pumas 2024 [47] | Pre-LLM "Action Planner" combining hardwired functions | — | — |

Relevant study from NimbRo's group: Bode et al. 2024 [48] compared function-calling prompting
techniques on RoboCup-style tasks. The best results came from CoT/ReAct "combined with an example of a
successful completion of a related task". Latency was large: GPT-4 Fetch took 11.6 s at baseline,
about 31 s with ReAct plus example, and up to about 72 s on the hardest task. Quote: "the response time of an LLM
can significantly increase the execution time of a task".

The LisTex TDP gives a sober view: "LLMs fail on long-horizon planning tasks; can produce
inconsistent, unactionable plans; or simply fail to perform well-understood, easily-programmable
tasks" [39].

Not found: 2025-2026 primary sources with planning details for b-it-bots, LyonTech, eR@sers, Homer,
RoboFEI, SocRob (only the joint LisTex TDP) or Pumas (only 2024). No team reports per-task success
rates for LLM-planned Clean the Table / Serve Breakfast / Storing Groceries.

**Inference:** the competition-proven pattern is "LLM → validated structured call(s)", followed by either
closed-loop function calling (NimbRo, HMA) or a deterministic executor (UT Austin, JSK). For the
manipulation-heavy tasks, top teams still use hand-built task pipelines whose parameters the LLM may
fill in. No one has shown open-ended LLM planning winning those tasks yet.

## 4. Rulebook notes (2027 draft, RoboCupAtHome/RuleBook master @ 67bd6ad, 2026-11-24)

- Tasks in the draft: **GPSR**, **Human Robot Interaction Challenge**, **Pick and Place Challenge**,
  **Doing Laundry**, **Finals**. Restaurant is removed in the current draft (#1059) [49].
- **Pick and Place Challenge** (since 2026.1, #994) "Combines aspects of Storing Groceries, Serve
  Breakfast and Clean the Table" and "Robot is free to choose what parts to solve" [49]. Main goals:
  dirty tableware/cutlery into the dishwasher, trash into the bin, other objects into the cabinet
  "grouping them with similar items", and set up breakfast (bowl, spoon, cereal, milk). Optional: floor trash,
  dishwasher door/rack, dishwasher tab, pouring. Focus includes "task planning". Ordering: "The robot
  may determine the order and method for performing all pick-and-place tasks", but "Certain actions,
  such as pouring or closing the dishwasher, must occur after prerequisite steps". The robot may ask for
  help with the dishwasher door/racks without penalty; other assistance is penalised per item [50].
- **GPSR**: commands come from the command generator, then "it will be entered into an LLM to re-generate a
  similar phrase ... Each command may be re-phrased up to 3 times getting simpler". There is an "Interleaved
  Task Bonus" when three commands are given at once and executed in a meaningfully interleaved order
  [51]. *Inference:* this rewards a planner that can merge and reorder several sub-goal plans.
- **Finals**: find and solve at least 8 problems (floor trash, misplaced objects, closing the dishwasher,
  laundry basket, a guest at the door, person requests) [52]. This is open-ended, long-horizon, and has no fixed sequence.
- **Cloud/LLM use**: "Teams are allowed to use External Computing through the internet connection of
  the Arena Network; this includes cloud services or online APIs." The organizers "do not guarantee
  or take any responsibility for the availability or reliability of the network" and "Robots must be
  able to operate safely even if External Computing is unavailable" [53]. The scenario rules say
  "robots are expected to be able to function without it" [54].

## 5. Options for us

Today: one LLM call → flat list of {action, object, person, source, destination, sentence} → the
action server runs the steps in order and the task ends at the first failure.

### (a) Two-level decomposition, each sub-goal is a flat GPSR plan
The LLM first writes a short list of natural-language sub-goals ("put mug in dishwasher", "put apple
in cabinet with fruit", "fetch bowl to table" ...). Each sub-goal is expanded with the *existing*
flat planner. The executor runs the sub-goals in order. A failed sub-goal is skipped or retried
once; the task continues.
- Evidence: DELTA [9], LLM-as-BT-Planner stage (i) [15], TRAIL's two-stage CoT [40], pi0.5/ER's
  subtask level [23][27].
- Needs: per-sub-goal success/failure from the action server; a "skip/continue" policy.
- Pros: reuses the current planner and executor; the plans can be checked and logged; simple.
- Cons: still open-loop inside a sub-goal; it cannot expand "every object on the table" without seeing
  the table first, so a perception-first step is needed (inspect → list → decompose).
- Latency: 1 + N calls (can be batched); about 5-15 s up front (inference).

### (b) Closed-loop ReAct / function calling (step by step)
After each skill, the action server returns a textual/structured result, and the LLM picks the next skill
(NimbRo style).
- Evidence: Inner Monologue [2], ReAct [5], NimbRo [33], HMA [36], Gemini ER orchestration [23][26].
- Needs: honest structured results (`succeeded`, `failed: object_not_found`, `failed: grasp`, plus
  observations such as the list of seen entities); world-model query tool(s); a step/time budget;
  a whitelist of argument values.
- Pros: handles surprises, partial perception and recovery; smallest change to the skill set.
- Cons: latency (Bode et al.: 11.6 s → 31-72 s per task with GPT-4 [48]); harder to predict;
  LLM-driven loops can wander off; every step depends on the network.
- Latency: one LLM call per skill. With 30-60 skills in Pick and Place, that adds minutes (inference).

### (c) LLM writes a behaviour tree or restricted program, checked by a validator
The LLM emits a BT (or a tiny DSL with `for each`, `if failed`, variables). It is statically validated
(schema, known skills, argument types, preconditions), then executed by a deterministic engine.
- Evidence: BTGenBot [14], LLM-as-BT-Planner [15], ProgPrompt asserts [4], Code as Policies [3].
- Needs: loops, variables/bindings ("the object found"), condition nodes backed by world-state
  queries, a BT/DSL interpreter in or above the action server, and a validator.
- Pros: one LLM call; recovery branches are explicit and can be inspected before running; safe on a real robot.
- Cons: the most new infrastructure; LLM BT generation is brittle for small models [15]; recovery
  branches only cover failures the LLM anticipated.

### (d) LLM → PDDL/JSON goal + classical planner
The LLM translates the instruction (and the perceived scene) into a goal ("mug in dishwasher, apple
on shelf with fruit, bowl+spoon on table"). A planner (Fast Downward, or PDDLStream for geometry)
computes the skill sequence. Replan on failure from the updated world model.
- Evidence: LLM+P [6], LLM-DP [7], DELTA [9], UT Austin / LisTex CoCo-TAMP [38][39], RoBorregos'
  pre-execution grounding [44].
- Needs: PDDL domain for our ~11 skills (preconditions/effects), a world state exported from ED as
  facts, a plan-to-action-server bridge, and failure → replanning.
- Pros: correct by construction for the domain; ordering constraints (pour after placing, close
  dishwasher after loading) are encoded once; cheap replanning; works offline after the goal is parsed.
- Cons: domain-modelling effort; brittle for open-ended GPSR/Finals requests the domain does not cover;
  hides geometric failure (needs TAMP-style feedback [10][11][17]).

### (e) Task skeleton (state machine) with LLM-filled parameters
Keep a hand-written SMACH/BT skeleton for Pick and Place ("for each object on table: classify → pick
→ place to destination"). The LLM/VLM only decides the categories, destinations, order and arrangement.
- Evidence: NimbRo's "hierarchical state machines whose parameters can be derived from natural
  language" [35]; UT Austin's state machine assembled from parsed JSON [38]; JSK [42].
- Needs: little new in the action server; LLM classification calls.
- Pros: most robust on competition day; matches what winning teams actually do; works without the network
  if a local classifier is used.
- Cons: not general (does not solve GPSR or Finals); each new task is new engineering.

### (f) Hybrid: hierarchical plan + closed-loop execution per sub-goal + validator (recommended)
Level 1: decompose into sub-goals after an initial inspection, as in (a). Level 2: each sub-goal
expands to a flat step list validated by a deterministic checker (allowed skills, known entities,
simple preconditions such as "holding object before place"). Execution: on failure, send the structured
failure plus the current world state back to the LLM to re-plan **that sub-goal only**, at most k
times, then skip. Optionally use (d) for Pick and Place, where the goal state is well-defined.
- Evidence: SayPlan's iterative replanning against a checker [8]; Inner Monologue feedback [2];
  DELTA's sub-goals [9]; ER 2's progress tracking [24]; Self-Recovery Prompting [41]; LisTex's schema
  validation [39].
- Needs: all of (a) and (b)'s result channel, plus a validator and a retry budget.
- Pros: bounded LLM calls (decompose once, plus one call per failure rather than per step); failures stay
  local; the plan stays inspectable; GPSR interleaving comes almost free (merge sub-goal lists).
- Cons: more moving parts than (a) or (b); needs prompt/eval work for the decomposition level.

### Comparison

| Option | LLM calls | Handles surprises | Checkable before execution | New action-server features | Offline-friendly | Main evidence |
|---|---|---|---|---|---|---|
| (a) Sub-goals → flat plans | 1 + N | Low (skip/retry only) | Yes | Per-sub-goal result | Partly | DELTA, TRAIL, pi0.5 subtasks |
| (b) ReAct / function calling | 1 per step | High | No (step by step) | Structured results, WM queries, budget | No | Inner Monologue, ReAct, NimbRo, HMA, Gemini ER |
| (c) LLM → BT/DSL + validator | 1 (+ repair) | Medium (pre-planned branches) | Yes, strongly | Loops, variables, conditions, interpreter | Yes after generation | BTGenBot, LLM-as-BT-Planner, ProgPrompt |
| (d) LLM → goal + PDDL planner | 1 (+ on failure) | Medium-high (replan) | Yes, by construction | Domain model, ED → facts, replan hook | Yes after parsing | LLM+P, LLM-DP, DELTA, UT Austin/LisTex |
| (e) Skeleton + LLM parameters | Few | Low-medium (coded) | Yes | Almost none | Yes | NimbRo, UT Austin, JSK |
| (f) Hybrid hierarchical + closed loop per sub-goal | 1 + failures | High | Yes per sub-goal | Results, WM queries, validator, retry budget, bindings | Partly | SayPlan, Inner Monologue, DELTA, Self-Recovery, Gemini ER 2 |

### What the action server needs (inference, applies to most options)

1. **Honest, structured outcomes** per step: success/failure plus a reason code (not found, grasp
   failed, place failed, unreachable, person lost) and the observations made (entities seen, their ids).
   Today's "first failure ends the task" removes exactly the signal Inner Monologue/ReAct depend on [2][5].
2. **World-state queries** as a skill/tool: list entities on a furniture piece, what is in the gripper,
   known categories and default locations (UT Austin's knowledge base is the precedent [38]).
3. **Variables / bindings**: "pick up *it*" after "find *a fruit*"; plan steps referencing results of
   earlier steps (ProgPrompt/CaP-style).
4. **Bounded iteration**: "for each object found on the table" without the LLM unrolling a guessed
   count. Either a DSL construct (c) or re-planning per item (b/f).
5. **A validator** in front of execution: skill whitelist, argument types and known entities,
   simple pre/postconditions; reject or repair the plan before the robot moves (BTGenBot [14], RoBorregos [44]).
6. **Budgets and a fallback**: max retries, max LLM calls, time-aware skipping (NimbRo rejects
   infeasible or too-long commands [33]), and a local model or rule fallback for network loss [53][54].
7. **Ask-for-help as a skill**: the Pick and Place rules allow requests for dishwasher door/rack help
   without penalty [50].

### Latency / cost notes
- The measured figures: Bode et al. report 11.6 s (GPT-4, baseline) to about 31 s (ReAct+example) to about 72 s for a
  whole task [48]. Gemini ER docs: higher thinking levels increase latency; "use medium for a good
  balance" [26].
- Inference: in the 7-minute Pick and Place run (scoresheet `timelimit=7` [50]), a per-step ReAct loop (b) with 40+ skills can spend
  several minutes waiting on the LLM. Options (a), (d) and (f) keep LLM calls to about the number of sub-goals
  plus the number of failures.
- Cost is minor at competition scale. Network reliability and latency are the binding constraints.

## Sources

1. Ahn et al., "Do As I Can, Not As I Say" (SayCan), arXiv:2204.01691; https://arxiv.org/abs/2204.01691 ; numbers: https://say-can.github.io/
2. Huang et al., "Inner Monologue", arXiv:2207.05608; https://arxiv.org/abs/2207.05608
3. Liang et al., "Code as Policies", arXiv:2209.07753; https://arxiv.org/abs/2209.07753
4. Singh et al., "ProgPrompt", arXiv:2209.11302; https://arxiv.org/abs/2209.11302
5. Yao et al., "ReAct", arXiv:2210.03629; https://arxiv.org/abs/2210.03629
6. Liu et al., "LLM+P", arXiv:2304.11477; https://arxiv.org/abs/2304.11477
7. Dagan et al., "Dynamic Planning with a LLM" (LLM-DP), arXiv:2308.06391; https://arxiv.org/abs/2308.06391
8. Rana et al., "SayPlan", arXiv:2307.06135; https://arxiv.org/abs/2307.06135
9. Liu et al., "DELTA", arXiv:2404.03275; https://arxiv.org/abs/2404.03275
10. Lin et al., "Text2Motion", arXiv:2303.12153; https://arxiv.org/abs/2303.12153
11. Wang et al., "LLM³", arXiv:2403.11552; https://arxiv.org/abs/2403.11552
12. Ajay et al., "Compositional Foundation Models for Hierarchical Planning" (HiP), arXiv:2309.08587; https://arxiv.org/abs/2309.08587
13. Wang et al., "Voyager", arXiv:2305.16291; https://arxiv.org/abs/2305.16291
14. Izzo et al., "BTGenBot", arXiv:2403.12761; https://arxiv.org/abs/2403.12761
15. Ao et al., "LLM-as-BT-Planner", arXiv:2409.10444; https://arxiv.org/abs/2409.10444
16. Li et al., "Embodied Agent Interface", arXiv:2410.07166; https://arxiv.org/abs/2410.07166
17. "A Systematic Study of Large Language Models ..." (LLM planners on PDDLStream), arXiv:2510.00182; https://arxiv.org/abs/2510.00182
18. Shridhar et al., "ALFRED", arXiv:1912.01734; https://arxiv.org/abs/1912.01734
19. Puig et al., "VirtualHome", arXiv:1806.07011; https://arxiv.org/abs/1806.07011
20. Li et al., "BEHAVIOR-1K", arXiv:2403.09227; https://arxiv.org/abs/2403.09227
21. "LLM-GROP: Visually Grounded Robot Task and Motion Planning with LLMs", arXiv:2511.07727; https://arxiv.org/abs/2511.07727
22. WildLMa, "Long Horizon Loco-Manipulation in the Wild"; https://wildlma.github.io/
23. Google DeepMind, "Gemini Robotics 1.5 brings AI agents into the physical world" (2025-09-25); https://deepmind.google/blog/gemini-robotics-15-brings-ai-agents-into-the-physical-world/ ; tech report arXiv:2510.03342
24. Google, "Introducing Gemini Robotics ER 2" (2026-07-30); https://blog.google/innovation-and-ai/models-and-research/google-deepmind/gemini-robotics-er-2/
25. Google DeepMind, "Gemini Robotics 2 brings whole-body intelligence to robots" (2026-07-30); https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/
26. Gemini Robotics-ER developer docs; https://aistudio.google.com/docs/robotics-overview
27. Physical Intelligence, "π0.5", arXiv:2504.16054; https://arxiv.org/abs/2504.16054
28. Shi et al., "Hi Robot", arXiv:2502.19417; https://arxiv.org/abs/2502.19417
29. Figure, "Helix" (2025-02); https://www.figure.ai/news/helix
30. Figure, "Helix 02" (2026-01-27); https://www.figure.ai/news/helix-02
31. NVIDIA, "GR00T N1", arXiv:2503.14734; https://arxiv.org/abs/2503.14734
32. TRI LBM Team, "A Careful Examination of Large Behavior Models...", arXiv:2507.05331; https://arxiv.org/abs/2507.05331
33. Memmesheimer et al., "RoboCup@Home 2024 OPL Winner NimbRo", arXiv:2412.14989; https://arxiv.org/html/2412.14989v1
34. NimbRo 2024 paper PDF; https://www.ais.uni-bonn.de/papers/RoboCup_2024_NimbRo_Home.pdf
35. Memmesheimer et al., "Adaptive Domestic Service Robotics through Foundation Models...", GRC 2026; https://www.ais.uni-bonn.de/papers/GRC_2026_Memmesheimer_NimbRo_Home.pdf
36. Hibikino-Musashi@Home 2025 TDP, arXiv:2511.20180; https://arxiv.org/html/2511.20180
37. Hibikino-Musashi@Home 2026 TDP, arXiv:2611.16297; https://arxiv.org/pdf/2611.16297
38. UT Austin Villa@Home 2025 TDP; https://www.cs.utexas.edu/~AustinVilla/athome/UT_Austin_Villa_Home_2025_Team_Description_Paper.pdf
39. LisTex United 2026 TDP (UT Austin + ISR Lisbon); https://irsgroup.isr.tecnico.ulisboa.pt/wp-content/uploads/2026/03/UT_Austin_Villa_Home_2026_Team_Description_Paper.pdf
40. TRAIL TDP for RoboCup@Home 2024; https://athome.robocup.org/wp-content/uploads/DSPL-TRAIL-RoboCup_home_2024_Team_Description_Paper.pdf
41. Shirasaka et al., "Self-Recovery Prompting", arXiv:2309.14425; https://arxiv.org/abs/2309.14425
42. Obinata et al., "Foundation Model based Open Vocabulary Task Planning and Executive System for GPSR", arXiv:2308.03357; https://arxiv.org/abs/2308.03357
43. Tidyboy-OPL TDP; https://athome.robocup.org/wp-content/uploads/OPL-Tidyboy-OPL-TDP.pdf
44. "Taming the LLM: Reliable Task Planning for Robotics Using Parsing and Grounding" (MICAI 2025); https://link.springer.com/chapter/10.1007/978-3-032-09037-9_24 ; record: https://research.tec.mx/vivo-tec/display/ChapterSCO_105020834111
45. RoBorregos frida-cortex; https://github.com/RoBorregos/frida-cortex
46. LAR@Home 2025 TDP; https://lar.dei.uminho.pt/images/downloads/Team_Description_Paper_2025_LAR_at_Home.pdf
47. Pumas 2024 OPL TDP; https://athome.robocup.org/wp-content/uploads/OPL-Pumas-TDP2024.pdf
48. Bode et al., "A Comparison of Prompt Engineering Techniques for Task Planning and Execution in Service Robotics", Humanoids 2024, arXiv:2410.22997; https://arxiv.org/html/2410.22997v2
49. RoboCup@Home RuleBook CHANGELOG (2027 draft); https://github.com/RoboCupAtHome/RuleBook/blob/master/CHANGELOG.md ; PDF: https://robocupathome.github.io/RuleBook/rulebook/master.pdf
50. RuleBook `tasks/PickAndPlaceChallenge.tex`; https://github.com/RoboCupAtHome/RuleBook/blob/master/tasks/PickAndPlaceChallenge.tex
51. RuleBook `tasks/GPSR.tex`; https://github.com/RoboCupAtHome/RuleBook/blob/master/tasks/GPSR.tex
52. RuleBook `tasks/Finals.tex`; https://github.com/RoboCupAtHome/RuleBook/blob/master/tasks/Finals.tex
53. RuleBook `pages/general_rules/ExternalDevices.tex`; https://github.com/RoboCupAtHome/RuleBook/blob/master/pages/general_rules/ExternalDevices.tex
54. RuleBook `pages/general_rules/Scenario.tex` (Wireless network); https://github.com/RoboCupAtHome/RuleBook/blob/master/pages/general_rules/Scenario.tex
