import copy

# Common knowledge
from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

# Entity where the shelves are part of
cabinet_slam = "bookcase_noback1"  # In case of slam
cabinet_amcl = "bookcase"          # In case of amcl

# Shelves where objects might be
# object_shelves = ["bookcase/shelf2", "bookcase/shelf3"]
object_shelves = ["shelf1", "shelf3", "shelf4", "shelf5"]

# Shelf where we will actually try to grasp
# grasp_shelf = "bookcase/shelf2"
grasp_shelf = "shelf3"

# Shelf where we will actually place stuff
place_shelf = "shelf2"

# Room where everything will take place
room = "livingroom"  # ToDo: update!!!

# Object types that can be recognized
object_types = copy.copy(common.object_names)
# object_types = ['beer', 'bifrutas', 'coffee_pads', 'coke',
#                 'deodorant', 'fanta', 'ice_tea', 'mentos',
#                 'sprite', 'tea', 'teddy_bear', 'water',
#                 'xylit24_spearmint', 'xylit24_white']

# Minimum and maximum height from which to grab an object
min_grasp_height = 0.0  # ToDo
max_grasp_height = 1.5  # ToDo
