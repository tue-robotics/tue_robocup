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
cabinet_amcl = ["bookcase", "bookcase"]
cabinet_poses = [FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi), kdl.Vector(3.18, 2.713, 0.0)),
                              frame_id="map"),
                 FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi), kdl.Vector(3.18, 2.713, 0.0)),
                              frame_id="map")]
object_shelves = ["shelf3", "shelf4", "shelf5"]
object_types = [obj["name"] for obj in common.objects]

# Grasping
grasp_surface = ["dinner_table", "dinner_table"]
room = ["dining_room", "dining_room"]

# Placing
default_place_entity = cabinet_amcl
default_place_area = ["shelf2", "shelf2"]

# Table pose
table_poses = [FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(0.4, 1.92, 0.0)),
                            frame_id="map"),
               FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(0.4, 1.92, 0.0)),
                            frame_id="map")]

entity_poses = zip(cabinet_poses, table_poses, cabinet_amcl, grasp_surface, room, default_place_area)
