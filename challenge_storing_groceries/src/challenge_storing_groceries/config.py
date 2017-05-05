# TU/e
from robot_skills.util.kdl_conversions import FrameStamped

# RoboCup knowledge
from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('challenge_storing_groceries')

# Location
ENTITY_POSES = challenge_knowledge.entity_poses

# Inspection
CABINET = challenge_knowledge.cabinet_amcl
OBJECT_SHELVES = challenge_knowledge.object_shelves
OBJECT_TYPES = challenge_knowledge.object_types
DETECTED_OBJECTS_WITH_PROBS = []  # List with entities and types. This is used to write to PDF. # ToDo: no global?!
SEGMENTED_ENTITIES = []  # List with segmented entities such that we can also grasp unknown entities
CLASSIFICATION_THRESHOLD = 0.1  # Threshold for perception. If classification below threshold, the type is not added
# to the world model and further on considered unknown.
MAX_KNOWN_OBJECTS = 10  # Maximum number of known objects to store in the PDF
MAX_UNKNOWN_OBJECTS = 5  # Maximum number of unknown objects to store in the PDF
MIN_OBJECT_HEIGHT = 0.1

# List to skip: these won't be written to the pdf
SKIP_LIST = ["cloth", "cloths", "towel", "bag", "fork", "spoon", "knife", "plate"]

# Grasping
TABLE = challenge_knowledge.grasp_surface
# TABLE_POSE = FrameStamped(frame=challenge_knowledge.table_pose, frame_id="map")
GRAB_SURFACE = "on_top_of"
ROOM = challenge_knowledge.room
MIN_GRAB_OBJECT_HEIGHT = 0.075
MAX_GRAB_OBJECT_WIDTH = 0.15

# Placing
DEFAULT_PLACE_ENTITY = challenge_knowledge.default_place_entity
DEFAULT_PLACE_AREA = challenge_knowledge.default_place_area

# Debug
DEBUG = False


# # MAX_NUM_ENTITIES_IN_PDF = 10
# # MIN_GRASP_HEIGHT = challenge_knowledge.min_grasp_height
# # MAX_GRASP_HEIGHT = challenge_knowledge.max_grasp_height
#

# SEGMENTED_ENTITIES = []  # List with segmented entities such that we can also grasp unknown entities
#
# PREFERRED_ARM = "left"  # Must be "left" or "right"
#
# DEBUG = False
#
# ignore_ids = ['robotics_testlabs']
# ignore_types = ['waypoint', 'floor', 'room']
# PLACE_HEIGHT = 1.0
# PLACE_SHELF = challenge_knowledge.place_area
#
# # Criteria
# not_ignored = lambda entity: not entity.type in ignore_types and not entity.id in ignore_ids
# size = lambda entity: abs(entity.z_max - entity.z_min) < 0.4
# has_type = lambda entity: entity.type != ""
# min_entity_height = lambda entity: abs(entity.z_max - entity.z_min) > 0.04
#
# def max_width(entity):
#     max_bb_x = max(ch.x for ch in entity.convex_hull)
#     min_bb_x = min(ch.x for ch in entity.convex_hull)
#     max_bb_y = max(ch.y for ch in entity.convex_hull)
#     min_bb_y = min(ch.y for ch in entity.convex_hull)
#
#     x_size = abs(max_bb_x - min_bb_x)
#     y_size = abs(max_bb_y - min_bb_y)
#
#     x_ok = 0.02 < x_size < 0.15
#     y_ok = 0.02 < y_size < 0.15
#
#     return x_ok and y_ok
