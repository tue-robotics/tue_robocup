from robocup_knowledge.environments.rwc2016a.common import *

# During the final challenge, we have no rooms
rooms = []
for loc in locations:
    loc["room"] = ""

translations = { "bookcase" : "bocase" }
