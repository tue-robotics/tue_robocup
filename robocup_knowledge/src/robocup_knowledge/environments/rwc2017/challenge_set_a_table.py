from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

initial_pose = "initial_pose"
starting_pose = "gpsr_meeting_point"

cupboard = "kitchen_counter"
cupboard_surface = "on_top_of"

table = "kitchen_table"
table_surface = "on_top_of"

# options = {
#     "fries": ["fries", "coke", "aquarius", "fork", "spoon", "plate"],
#     "curry": ["curry", "green_tea", "cold_brew", "chop_sticks", "spoon", "bowl"]
# }

options = {
    "fries": {"food": "fries", "drink1": "coke", "drink2": "aquarius",
              "difficult1": "fork", "difficult2": "spoon", "difficult3": "plate"},
    "curry": {"food": "green_tea", "drink1": "cold_brew", "drink2": "green_tea",
              "difficult1": "chop_sticks", "difficult2": "spoon", "difficult3": "bowl"}
}
