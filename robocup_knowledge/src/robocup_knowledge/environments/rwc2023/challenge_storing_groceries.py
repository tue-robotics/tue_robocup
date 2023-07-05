# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'exit_1'

# Detection
shelf = "pantry"
default_area = "shelf3"
inspect_area = "in_front_of"

# Grasping
table = "storing_groceries_table"
room = common.get_room(table)
