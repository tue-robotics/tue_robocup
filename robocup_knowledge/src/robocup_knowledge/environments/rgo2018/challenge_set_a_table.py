from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

initial_pose = "initial_pose"
starting_pose = "gpsr_meeting_point"

cupboard = "kitchen_cabinet"
cupboard_surface = "on_top_of"

kitchen = "kitchen_table"
kitchen_surface = "on_top_of"

table = "dining_table"
table_surface = "on_top_of"

# options = {
#     "fries": ["fries", "coke", "aquarius", "fork", "spoon", "plate"],
#     "curry": ["curry", "green_tea", "cold_brew", "chop_sticks", "spoon", "bowl"]
# }

options = {
    "apple": {"drink1": "water", "drink2": "malz", "food": "apple",
              "difficult1": "fork", "difficult2": "spoon", "difficult3": "plate"},
    "noodle": {"drink1": "coke", "drink2": "mixdrink", "food": "noodle",
              "difficult1": "fork", "difficult2": "spoon", "difficult3": "plate"}
}

