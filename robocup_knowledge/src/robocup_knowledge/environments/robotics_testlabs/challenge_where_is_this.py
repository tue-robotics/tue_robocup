# WHERE IS THIS KNOWLEDGE FILE ROBOTICS TESTLABS

# System
from collections import namedtuple

# TU/e Robotics
from robocup_knowledge import knowledge_loader

BackupScenario = namedtuple("BackupScenario", ["entity_id", "sentence"])

backup_scenarios = [
    BackupScenario("fridge", "I will take you to the fridge for a cold beer"),
    BackupScenario("bed", "You look tired, I will take you to the bed"),
    BackupScenario("desk", "I'll have you sit down at the desk so you can work on my speech recognition skills"),
]

information_point_id = "where_is_this_information_point"
initial_pose_id = "initial_pose"

# Common knowledge
common = knowledge_loader.load_knowledge("common")

grammar_target = "T"

starting_point_grammar = """
T[A] -> LOCATION[A]
"""

for loc in common.location_names:
    starting_point_grammar += '\nLOCATION[%s] -> %s' % (loc, loc)

location_grammar = """
T[A] -> COURTESY_PREFIX VP[A] | VP[A]

COURTESY_PREFIX -> robot please | could you | could you please | please
"""

for loc in common.location_rooms:
    location_grammar += '\nLOCATION[%s] -> %s' % (loc, loc)

for loc in common.location_names:
    location_grammar += '\nLOCATION[%s] -> %s' % (loc, loc)

for loc in common.object_names:
    category = common.get_object_category(loc)
    if category not in common.category_locations:
        continue

    entity_id = common.get_object_category_location(category)[0]

    location_grammar += '\nLOCATION[%s] -> %s' % (entity_id, loc)

location_grammar += '\nDET_LOCATION[Y] -> LOCATION[Y] | the LOCATION[Y]'

location_grammar += """
V_GUIDE -> guide | bring | lead

VP[Y] -> DET_LOCATION[Y] | V_GUIDE me to DET_LOCATION[Y] | i want to go to DET_LOCATION[Y] | i would like to go to DET_LOCATION[Y] | i like to go to DET_LOCATION[Y] | tell me how to go to DET_LOCATION[Y] | tell me how to reach DET_LOCATION[Y]

"""

if __name__ == "__main__":
    print("Where is this Grammar:\n\n{}\n\n".format(location_grammar))

    from grammar_parser.cfgparser import CFGParser

    print("Location Grammar")
    grammar_parser = CFGParser.fromstring(location_grammar)
    grammar_parser.verify()
    grammar_parser.check_rules()
    print("Random Sentence:")
    sentence = grammar_parser.get_random_sentence("T")
    print(sentence)
    print("Resulting Semantics:")
    print(grammar_parser.parse("T", sentence))

    print("\n\nStarting Point Grammar")
    grammar_parser = CFGParser.fromstring(starting_point_grammar)
    grammar_parser.verify()
    grammar_parser.check_rules()
    print("Random Sentence:")
    sentence = grammar_parser.get_random_sentence("T")
    print(sentence)
    print("Resulting Semantics:")
    print(grammar_parser.parse("T", sentence))
