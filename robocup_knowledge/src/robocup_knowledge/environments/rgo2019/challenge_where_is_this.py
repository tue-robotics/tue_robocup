# RIPS KNOWLEDGE FILE RGO2019

# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

grammar_target = "T"

location_grammar = """
T[A] -> COURTESY_PREFIX VP[A] | VP[A]

COURTESY_PREFIX -> robot please | could you | could you please | please
"""

for loc in common.location_rooms:
    location_grammar += '\nLOCATION[{"id": "%s"}] -> %s' % (loc, loc)

for loc in common.location_names:
    location_grammar += '\nLOCATION[{"id": "%s"}] -> %s' % (loc, loc)

for loc in common.object_names:
    category = common.get_object_category(loc)
    if category not in common.category_locations:
        continue

    entity_id = common.get_object_category_location(category)[0]

    location_grammar += '\nLOCATION[{"id": "%s"}] -> %s' % (entity_id, loc)

location_grammar += '\nDET_LOCATION[Y] -> LOCATION[Y] | the LOCATION[Y]'

location_grammar += """
V_GUIDE -> guide | bring | lead

VP[{"target-location": Y}] -> DET_LOCATION[Y] | V_GUIDE me to DET_LOCATION[Y] | i want to go to DET_LOCATION[Y] | i would like to go to DET_LOCATION[Y] | i like to go to DET_LOCATION[Y] | tell me how to go to DET_LOCATION[Y] | tell me how to reach DET_LOCATION[Y]

"""

if __name__ == "__main__":
    print("Where is this Grammar:\n\n{}\n\n".format(location_grammar))

    from grammar_parser.cfgparser import CFGParser

    grammar_parser = CFGParser.fromstring(location_grammar)
    grammar_parser.verify()
    grammar_parser.check_rules()
