from robocup_knowledge.environments.rwc2016a.common import *
from robocup_knowledge import knowledge_functions

# During the open challenge, we have no rooms
rooms = []
for loc in locations:
    loc["room"] = ""

translations = { "bookcase" : "bocase" }
