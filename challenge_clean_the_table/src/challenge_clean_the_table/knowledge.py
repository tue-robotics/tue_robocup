import PyKDL
from std_msgs.msg import ColorRGBA

ITEMS_PLATE = ["plate"]
ITEMS_MUG_BOWL = ["mug", "bowl"]
ITEMS_CUTLERY = ["knife", "spoon"]
ITEMS = ITEMS_PLATE + ITEMS_MUG_BOWL + ITEMS_CUTLERY

ITEM_VECTOR_DICT = {
    "knife": PyKDL.Vector(-0.66, -0.285, 0),
    "spoon": PyKDL.Vector(-0.66, -0.285, 0),
    "mug": PyKDL.Vector(-0.51, -0.285, 0),
    "plate": PyKDL.Vector(-0.51, -0.025, 0),
    "bowl": PyKDL.Vector(-0.66, -0.085, 0),
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

OPEN_DISHWASHER_VECTOR = PyKDL.Vector(-1.0, 0, 0)
OPEN_DISHWASHER_VECTOR_OPEN = PyKDL.Vector(OPEN_DISHWASHER_VECTOR.x() - 0.3, 0, 0.1)

PICK_ID = "table_salon"
PICK_AREA_ID = "in_front_of"

PLACE_ID = "table_salon"
PLACE_AREA_ID = "in_front_of"

DISHWASHER_ID = "dishwasher"
DISHWASHER_AREA_ID = "in_front_of"

EXIT_ID = "starting_pose"

PICK_ROTATION = 2.0

JOINTS_HANDOVER = [0, -0.3, 0, 0.3, 1.57]

JOINTS_PRE_PRE_PLACE = [0.69, 0, 0, -0.7, 0]

JOINTS_PRE_PLACE = [0.8, -1.57, 0, -1.57, 0]

LAST_JOINTS_PLACE = [-1.57, 0, -1.57, 0]
JOINTS_PLACE_PLATE = [0.57] + LAST_JOINTS_PLACE
JOINTS_PLACE_MUG_BOWL = [0.4] + LAST_JOINTS_PLACE
JOINTS_PLACE_CUTLERY = [0.5] + LAST_JOINTS_PLACE

JOINTS_RETRACT = [0.7, 0, 0, -1.57, 0]

JOINTS_OPEN_DISHWASHER = [0.377, -1.57, 0, 0, 1.57]
