# Every parser writes the same plan; one shared step turns it into a task

Status: accepted, not implemented yet (2026-11-26).

HERO has two parsers: the LLM task planner, and the NER model (BERT) as its fallback when the language model cannot be used. We want them to stay close in what they can do, so that falling back costs accuracy, not capabilities. Today they differ after understanding the command: the LLM writes steps (an action plus the roles object, person, source, destination, sentence and person_attributes), and `compile_plan` checks them against the names HERO knows and turns them into the action server's recipe; the NER model writes the recipe itself, with its own rules. As a result the fallback cannot ask for a category ("a drink" becomes one particular drink), loses person attributes ("the waving person" becomes anyone), and puts a person where a location is expected.

Decision: the plan of steps is the one format every parser writes, and only one shared module turns a plan into a recipe.

- The step definition (the actions, the roles each action takes, the attributes and their values) and `compile_plan` move out of `action_server.llm` into a module that knows no parser, e.g. `action_server.plan`. Checking the names, "it", "me", "someone", type versus category, and attributes then happen in one place for every parser.
- The NER mapper writes steps instead of a recipe.
- What each parser can produce is derived from that one definition: the LLM's JSON schema as today, and the NER model's labels (intents = actions, slots = roles plus one slot per attribute, linker choices = the same vocabulary with categories and attribute values). A test fails when the model's labels and the step definition disagree.
- The NER model is fine-tuned on steps, with the LLM planner as teacher: commands from the GPSR command generator, planned by the LLM, with each step value aligned back to the words of the command. One table of command -> expected plan tests both parsers; the recipe configuration tests cover everything after the plan.

## Considered options

- **Keep two separate mappers** (today). No work now, but every new role or attribute has to be added twice, and the fallback silently does less.
- **Let the NER model write recipes, but with the same rules copied in.** Keeps the model's output as it is, but duplicates `compile_plan` and drifts again.
- **Drop the NER fallback** and only use a local LLM. Simpler, but leaves HERO without a planner when no LLM can run, and a BERT model is far cheaper on the robot's GPU.

## Consequences

- Adding a role, action or attribute means retraining the NER model; the LLM only reads the new schema. The label test makes a forgotten retrain visible.
- The NER model still does worse at what is more than tagging words: commands that need reasoning ("the heaviest object"), free sentences to say (it can only copy words of the command), and wording far from the command generator's.
- The refactor of step 1 (moving the step definition and `compile_plan`) changes no behaviour and is covered by the existing tests, so it can be done before any retraining.
