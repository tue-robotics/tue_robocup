# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'initial_pose'

# Detection
shelf = "closet"
object_types = [obj["name"] for obj in common.objects]

# Grasping
table = "dinner_table"
