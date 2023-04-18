import PyKDL
from std_msgs.msg import ColorRGBA

ITEMS_PLATE = ["plate"]
ITEMS_MUG_BOWL = ["mug", "bowl"]
ITEMS_CUTLERY = ["knife", "spoon", "fork"]
ITEMS = ITEMS_CUTLERY + ITEMS_MUG_BOWL + ITEMS_PLATE

ITEM_VECTOR_DICT = {
    "knife": PyKDL.Vector(-0.63, -0.325, 0),
    "fork": PyKDL.Vector(-0.63, -0.325, 0),
    "spoon": PyKDL.Vector(-0.63, -0.325, 0),
    "mug": PyKDL.Vector(-0.46, -0.325, 0),
    "plate": PyKDL.Vector(-0.46, -0.155, 0),
    "bowl": PyKDL.Vector(-0.63, -0.155, 0),
}

ITEM_COLOR_DICT = {
    "knife": ColorRGBA(1, 0, 1, 1),
    "fork": ColorRGBA(1, 0, 1, 1),
    "spoon": ColorRGBA(0, 1, 1, 1),
    "mug": ColorRGBA(0, 0, 1, 1),
    "plate": ColorRGBA(1, 1, 0, 1),
    "bowl": ColorRGBA(0, 1, 0, 1),
}

ITEM_IMG_DICT = {
    "knife": "images/knife.jpg",
    "fork": "images/fork.jpg",
    "spoon": "images/spoon.jpg",
    "mug": "images/mug.jpg",
    "plate": "images/plate.jpg",
    "bowl": "images/bowl.jpg",
}

OPEN_DISHWASHER_VECTOR = PyKDL.Vector(-0.985, -0.16, 0)
OPEN_DISHWASHER_VECTOR_OPEN = PyKDL.Vector(OPEN_DISHWASHER_VECTOR.x() - 0.45, OPEN_DISHWASHER_VECTOR.y(),
                                           OPEN_DISHWASHER_VECTOR.z())

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
JOINTS_PLACE_PLATE = [0.62] + LAST_JOINTS_PLACE
JOINTS_PLACE_MUG_BOWL = [0.47] + LAST_JOINTS_PLACE
JOINTS_PLACE_CUTLERY = [0.62] + LAST_JOINTS_PLACE

JOINTS_RETRACT = [0.7, 0, 0, -1.57, 0]

JOINTS_OPEN_DISHWASHER = [0.377, -1.57, 0, 0, 1.57]
