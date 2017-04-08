# RoboCup knowledge
from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('challenge_storing_groceries')

# Inspection
CABINET = challenge_knowledge.cabinet_amcl
OBJECT_SHELVES = challenge_knowledge.object_shelves

# Grasping
TABLE = challenge_knowledge.grasp_surface
ROOM = challenge_knowledge.room

# Placing
DEFAULT_PLACE_ENTITY = challenge_knowledge.default_place_entity
DEFAULT_PLACE_AREA = challenge_knowledge.default_place_area

# OBJECT_TYPES = challenge_knowledge.object_types
# # MAX_NUM_ENTITIES_IN_PDF = 10
# # MIN_GRASP_HEIGHT = challenge_knowledge.min_grasp_height
# # MAX_GRASP_HEIGHT = challenge_knowledge.max_grasp_height
#
# DETECTED_OBJECTS_WITH_PROBS = []  # List with entities and types. This is used to write to PDF
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
