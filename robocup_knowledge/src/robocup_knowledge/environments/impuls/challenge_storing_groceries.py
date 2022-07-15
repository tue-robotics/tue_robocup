# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'initial_pose'

# Detection
shelf = "closet"
object_shelves = ["shelf3", "shelf4", "shelf5"]
object_types = [obj["name"] for obj in common.objects]

# Grasping
table = "dinner_table"