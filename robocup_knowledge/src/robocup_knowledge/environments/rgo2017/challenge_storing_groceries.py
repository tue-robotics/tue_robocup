from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# Detection
cabinet_amcl = "bookshelf"  # "cabinet"
object_shelves = ["shelf2", "shelf3", "shelf4", "shelf5"]
object_types = [obj["name"] for obj in common.objects]

# Grasping
grasp_surface = "couch_table"
room = "dining_room"

# Placing
default_place_entity = "bookshelf"  # "cabinet"
default_place_area = "shelf3"
