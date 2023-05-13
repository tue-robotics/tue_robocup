# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'initial_pose'

# Detection
shelf = "cabinet"
default_area = "shelf4"
inspect_area = "inspect_area"
object_shelves = ["shelf3", "shelf4", "shelf5"] #TODO unused variable?
object_types = [obj["name"] for obj in common.objects] #TODO unused variable?

# Grasping
table = "side_board"
room = common.get_room(table)
