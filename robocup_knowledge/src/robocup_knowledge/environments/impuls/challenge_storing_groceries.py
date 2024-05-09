# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# General
starting_point = 'initial_pose'

# Detection
shelf = "closet"
shelf_room = common.get_room(shelf)
default_area = "shelf4"
cabinet_inspect_area = "shelf2"
inspect_area = "in_front_of"
object_shelves = ["shelf3", "shelf4", "shelf5"]  # TODO unused variable?
object_types = [obj["name"] for obj in common.objects]  # TODO unused variable?
place_areas = ["shelf2", "shelf3"]

# Grasping
table = "dinner_table"
table_room = common.get_room(table)

