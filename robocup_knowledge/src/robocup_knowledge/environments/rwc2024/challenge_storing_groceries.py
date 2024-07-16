# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = "initial_pose"

# Detection
shelf = "kitchen_cabinet"
shelf_room = common.get_room(shelf)
default_area = "shelf3"
inspect_area = "in_front_of"
place_areas = common.get_inspect_areas(shelf)
cabinet_inspect_area = "shelf2"

# Grasping
table = "dinner_table"
table_room = common.get_room(table)
