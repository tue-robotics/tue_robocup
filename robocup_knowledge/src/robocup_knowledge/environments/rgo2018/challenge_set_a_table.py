from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

initial_pose = "initial_pose"
starting_pose = "gpsr_meeting_point"

cupboard = "kitchen_cabinet"
cupboard_surface = "on_top_of"

table = "kitchen_table"
table_surface = "on_top_of"

# options = {
#     "fries": ["fries", "coke", "aquarius", "fork", "spoon", "plate"],
#     "curry": ["curry", "green_tea", "cold_brew", "chop_sticks", "spoon", "bowl"]
# }

options = {
    "noodles": {"food": "noodles", "drink1": "water", "drink2": "malz",
              "difficult1": "fork", "difficult2": "spoon", "difficult3": "plate"},
    "cookies": {"food": "cookies", "drink1": "coke", "drink2": "mixdrink",
              "difficult1": "fork", "difficult2": "spoon", "difficult3": "plate"}
}

