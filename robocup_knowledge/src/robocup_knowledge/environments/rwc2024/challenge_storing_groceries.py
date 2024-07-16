# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = "initial_pose"

# Detection
shelf = "kitchen_cabinet"
default_area = "shelf3"
inspect_area = "in_front_of"
place_areas = common.get_inspect_areas(shelf)

# Grasping
table = "dinner_table"
room = common.get_room(table)
