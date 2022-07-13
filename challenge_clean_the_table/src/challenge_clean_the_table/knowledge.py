import PyKDL
from std_msgs.msg import ColorRGBA

ITEMS_CUTLERY = ["knife", "spoon"]
ITEM_MUG = "mug"
ITEM_PLATE = "plate"
ITEMS = ITEMS_CUTLERY + [ITEM_PLATE] + [ITEM_MUG]  # "bowl" could be added, but we will first set without it

ITEM_VECTOR_DICT = {
    "knife": PyKDL.Vector(0.03, -0.15, 0),
    "spoon": PyKDL.Vector(0.1, -0.04, 0),
    "mug": PyKDL.Vector(-0.1, 0.3, 0),
    "plate": PyKDL.Vector(-0.1, -0.3, 0),
    "bowl": PyKDL.Vector(0.1, 0.3, 0),
}

ITEM_COLOR_DICT = {
    "knife": ColorRGBA(1, 0, 1, 1),
    "spoon": ColorRGBA(0, 1, 1, 1),
    "mug": ColorRGBA(0, 0, 1, 1),
    "plate": ColorRGBA(1, 1, 0, 1),
    "bowl": ColorRGBA(0, 1, 0, 1),
}

ITEM_IMG_DICT = {
    "knife": "images/knife.jpg",
    "spoon": "images/spoon.jpg",
    "mug": "images/mug.jpg",
    "plate": "images/plate.jpg",
    "bowl": "images/bowl.jpg",
}

PICK_ID = "dinner_table"
PICK_AREA_ID = "in_front_of"

PLACE_ID = "dinner_table"
PLACE_AREA_ID = "in_front_of"

DISHWASHER_ID = "dishwasher"
DISHWASHER_AREA_ID = "in_front_of"

EXIT_ID = "starting_pose"

PICK_ROTATION = 2.0

JOINTS_HANDOVER = [0.4, -0.2, 0.0, -1.37, 0]
JOINTS_HANDOVER_PLATE = [0.4, -0.2, 0.0, -1.37, 1.57]

JOINTS_PRE_PRE_PLACE = [0.69, 0, 0, -0.7, 0]

JOINTS_PRE_PLACE_CUTLERY = [0.8, -1.2, 0, 0, 0]
JOINTS_PRE_PLACE_MUG = [0.8, -1.2, 0, -1.57, 0]
JOINTS_PRE_PLACE_PLATE = [0.65, -1.57, 0, -1.57, 0]

JOINTS_PLACE_CUTLERY = [0.8, -1.2, 0, 0, 0]
JOINTS_PLACE_MUG = [0.8, -1.2, 0, -1.57, 0]
JOINTS_PLACE_PLATE = [0.65, -1.57, 0, -1.57, 0]

JOINTS_RETRACT = [0.7, 0, 0, -1.57, 0]
