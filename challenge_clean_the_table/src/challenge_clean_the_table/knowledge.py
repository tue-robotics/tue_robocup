import math
import PyKDL
from std_msgs.msg import ColorRGBA

ITEMS_PLATE = ["plate"]
ITEMS_MUG_BOWL = ["mug", "bowl"]
ITEMS_CUTLERY = ["knife", "spoon", "fork"]
ITEMS = ITEMS_CUTLERY + ITEMS_MUG_BOWL + ITEMS_PLATE

ITEM_VECTOR_DICT = {
    "knife": PyKDL.Vector(-1.16, 0.04, 0),
    "fork": PyKDL.Vector(-1.16, 0.07, 0),
    "spoon": PyKDL.Vector(-1.16, 0.10, 0),
    "mug": PyKDL.Vector(-1.16, 0.15, 0),
    "plate": PyKDL.Vector(-1.16, -0.27, 0),
    "bowl": PyKDL.Vector(-1.16, -0.1, 0),
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

OPEN_DISHWASHER_VECTOR = PyKDL.Vector(-0.96, -0.15, math.pi)
OPEN_DISHWASHER_VECTOR_OPEN1 = OPEN_DISHWASHER_VECTOR + PyKDL.Vector(-0.15, 0, 0)
OPEN_DISHWASHER_VECTOR_OPEN2 = OPEN_DISHWASHER_VECTOR + PyKDL.Vector(-0.25, 0, 0)
OPEN_DISHWASHER_VECTOR_OPEN3 = OPEN_DISHWASHER_VECTOR + PyKDL.Vector(-0.30, 0, 0)

OPEN_DISHWASHER_VECTOR_OPEN4 = OPEN_DISHWASHER_VECTOR + PyKDL.Vector(-0.50, 0, 0)
OPEN_DISHWASHER_VECTOR_OPEN5 = OPEN_DISHWASHER_VECTOR + PyKDL.Vector(-0.30, 0, 0)
OPEN_DISHWASHER_VECTOR_OPEN6 = OPEN_DISHWASHER_VECTOR + PyKDL.Vector(-0.40, 0, 0)

PULL_DISHWASHER_RACK_PREPARE = OPEN_DISHWASHER_VECTOR_OPEN6 + PyKDL.Vector(0, -0.42, math.pi/2)
PULL_DISHWASHER_RACK_PREPARE_START = PULL_DISHWASHER_RACK_PREPARE + PyKDL.Vector(1.0, 0, 0)
PULL_DISHWASHER_RACK_OUT1 = PULL_DISHWASHER_RACK_PREPARE_START + PyKDL.Vector(0, 0, -0.34)
PULL_DISHWASHER_RACK_OUT2 = PULL_DISHWASHER_RACK_OUT1 + PyKDL.Vector(-0.55, 0, 0)

PICK_ID = "kitchen_table"
PICK_AREA_ID = "in_front_of"

PLACE_ID = "dishwasher"
PLACE_AREA_ID = "in_front_of_ctt"

DISHWASHER_ID = "dishwasher"
DISHWASHER_AREA_ID = "in_front_of"

EXIT_ID = "starting_pose"

PICK_ROTATION = 2.0

JOINTS_HANDOVER = [0, -0.3, 0, 0.3, 1.572]

JOINTS_PRE_PRE_PRE_PLACE = [0.6, 0, 0, -1.572, 0]

JOINTS_PRE_PRE_PLACE = [0.6, -1.572, 0, -1.572, 0]

JOINTS_PRE_PLACE = [0.6, -1.572, 0, -1.572, 0]

LAST_JOINTS_PLACE = [-1.572, 0, -1.572, 0]
JOINTS_PLACE_PLATE = [0.40] + LAST_JOINTS_PLACE
JOINTS_PLACE_MUG_BOWL = [0.31] + LAST_JOINTS_PLACE
JOINTS_PLACE_CUTLERY = [0.30] + LAST_JOINTS_PLACE

JOINTS_RETRACT = [0.7, 0, 0, -1.572, 0]

JOINTS_OPEN_DISHWASHER_PRE = [0.36, -1.572, 0, 0.62, 1.572]
JOINTS_OPEN_DISHWASHER = [0.37, -1.572, 0, 0.61, 1.572]
JOINTS_OPEN_DISHWASHER1 = [0.35, -1.572, 0, 0.65, 1.572]
JOINTS_OPEN_DISHWASHER2 = [0.28, -1.572, 0, 0.65, 1.572]
JOINTS_OPEN_DISHWASHER3 = [0.25, -1.572, 0, 0.65, 1.572]

JOINTS_OPEN_DISHWASHER4 = [0.60, -1.572, 0, -1.572, 0]
JOINTS_OPEN_DISHWASHER5 = [0.25, -1.572, 0, -1.572, 0]
JOINTS_OPEN_DISHWASHER6 = [0.07, -1.572, 0, -1.572, 0]

JOINTS_PULL_RACK_PREPARE = [0.10, -1.572, 0, 0, 0]
JOINTS_PULL_RACK_DOWN = [0.10, -1.572, 0, -0.7, 0]
