from robocup_knowledge.environments.rwc2016a.common import *

del locations[:]
locations += [
    { "room" : "", "name" : "bar",    "location_category" : "" ,          "manipulation" : "yes" },
    { "room" : "", "name" : "table1", "location_category" : "",     "manipulation" : "yes" },
    { "room" : "", "name" : "table2", "location_category" : "",           "manipulation" : "yes"  },
    { "room" : "", "name" : "table3", "location_category" : "",    "manipulation" : "yes" }
]
# During the final challenge, we have no rooms
rooms = []
for loc in locations:
    loc["room"] = ""

translations = { "bookcase" : "bocase" }

bar_objects = ["tea", "pringles", "sponge"]
bar_id = "bar"
