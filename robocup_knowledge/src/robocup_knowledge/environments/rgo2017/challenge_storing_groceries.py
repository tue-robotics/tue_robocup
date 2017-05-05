# System
import math

# ROS
import PyKDL as kdl

# TU/e Robotics
from robot_skills.util.kdl_conversions import FrameStamped
from robocup_knowledge import knowledge_loader

# Common knowledge
common = knowledge_loader.load_knowledge("common")

# Detection
cabinet_amcl = "bookshelf"  # "cabinet"
cabinet_poses = [FrameStamped(frame=kdl.Frame(kdl.Rotation(), kdl.Vector(0.144, 3.574, 0.0)), frame_id="map"),
                 FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 1.570796), kdl.Vector(2.271, -1.258, 0.0)),
                              frame_id="map")]
object_shelves = ["shelf2", "shelf3", "shelf4", "shelf5"]
object_types = [obj["name"] for obj in common.objects]

# Grasping
grasp_surface = "side_table"
room = "dining_room"

# Placing
default_place_entity = "bookshelf"  # "cabinet"
default_place_area = "shelf3"

# Table pose
table_poses = [FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, -0.5 * math.pi), kdl.Vector(1.0, 4.0, 0.0)),
                            frame_id="map"),
               FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi), kdl.Vector(3.0, 0.0, 0.0)),
                            frame_id="map")]
entity_poses = zip(cabinet_poses, table_poses)
