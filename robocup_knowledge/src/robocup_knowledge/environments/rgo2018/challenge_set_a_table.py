from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

intermediate_1 = "registration_table1"

initial_pose = "initial_pose"
starting_pose = "gpsr_meeting_point"
starting_point = "initial_pose"

cupboard = "kitchen_cabinet"
cupboard_surface = "on_top_of"

kitchen = "couch_table"
kitchen_surface = "on_top_of"

table = "kitchen_table"
table_surface = "on_top_of"

# options = {
#     "fries": ["fries", "coke", "aquarius", "fork", "spoon", "plate"],
#     "curry": ["curry", "green_tea", "cold_brew", "chop_sticks", "spoon", "bowl"]
# }

options = {
    "apple": {"drink1": "coke", "drink2": "mixdrink", "food": "apple",
              "difficult1": "fork", "difficult2": "spoon", "difficult3": "plate"},
    "noodles": {"drink1": "coke", "drink2": "mixdrink", "food": "noodles",
              "difficult1": "fork", "difficult2": "spoon", "difficult3": "plate"}
}

