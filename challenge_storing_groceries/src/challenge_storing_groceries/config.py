# TU/e
from robot_skills.util.kdl_conversions import FrameStamped

# RoboCup knowledge
from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('challenge_storing_groceries')

# Inspection
CABINET = challenge_knowledge.cabinet_amcl
OBJECT_SHELVES = challenge_knowledge.object_shelves
OBJECT_TYPES = challenge_knowledge.object_types
DETECTED_OBJECTS_WITH_PROBS = []  # List with entities and types. This is used to write to PDF. # ToDo: no global?!
SEGMENTED_ENTITIES = []  # List with segmented entities such that we can also grasp unknown entities
CLASSIFICATION_THRESHOLD = 0.3  # Threshold for perception. If classification below threshold, the type is not added
# to the world model and further on considered unknown.
MAX_KNOWN_OBJECTS = 10  # Maximum number of known objects to store in the PDF
MAX_UNKNOWN_OBJECTS = 5  # Maximum number of unknown objects to store in the PDF
MIN_OBJECT_HEIGHT = 0.1

# List to skip: these won't be written to the pdf
SKIP_LIST = ["fork", "spoon", "chopsticks"]

# Grasping
GRAB_SURFACE = "on_top_of"
MIN_GRAB_OBJECT_HEIGHT = 0.075
MAX_GRAB_OBJECT_WIDTH = 0.15

# Debug
DEBUG = False
