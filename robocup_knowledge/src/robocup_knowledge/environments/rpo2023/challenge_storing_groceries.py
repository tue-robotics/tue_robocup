from robocup_knowledge import knowledge_functions

# General
starting_point = 'initial_pose'

# Detection
shelf = "cabinet"
default_area = "shelf4"
inspect_area = "inspect_area"
object_shelves = ["shelf3", "shelf4", "shelf5"] #TODO unused variable?
object_types = [obj["name"] for obj in knowledge_functions.objects] #TODO unused variable?

# Grasping
table = "side_board"
room = knowledge_functions.get_room(table)
