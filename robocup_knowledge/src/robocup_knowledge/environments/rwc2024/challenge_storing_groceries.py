# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = "initial_pose"

# Detection
shelf = "kitchen_cabinet"
default_area = "shelf4"
inspect_area = "in_front_of"
object_shelves = ["shelf2", "shelf3", "shelf4"]  # TODO unused variable?
object_types = [obj["name"] for obj in common.objects]  # TODO unused variable?

# Grasping
table = "dinner_table"
room = common.get_room(table)
