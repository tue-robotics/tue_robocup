# System
import math

# ROS
import PyKDL as kdl

# TU/e Robotics
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# Detection
cabinet_amcl = "bookshelf"  # "cabinet"
object_shelves = ["shelf2", "shelf3", "shelf4", "shelf5"]
object_types = [obj["name"] for obj in common.objects]

# Grasping
grasp_surface = "side_table"
room = "dining_room"

# Placing
default_place_entity = "bookshelf"  # "cabinet"
default_place_area = "shelf3"

# Table pose
table_pose = kdl.Frame(kdl.Rotation().RPY(0, 0, -0.5 * math.pi), kdl.Vector(1.0, 4.5, 0.0))
