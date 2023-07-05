# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'initial_pose'

# Detection
shelf = "pantry"
default_area = "shelf3"
inspect_area = "in_front_of"

# Grasping
table = "side_tables"
room = "kitchen"
