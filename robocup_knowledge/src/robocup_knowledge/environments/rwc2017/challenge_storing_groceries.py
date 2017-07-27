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
cabinet_amcl = ["kitchen_shelf", "kitchen_shelf"]
cabinet_poses = [FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, -0.0, 0.1292036732),
                                              kdl.Vector(2.39, -4.125, 0)),
                              frame_id="map"),
                 FrameStamped(frame=kdl.Frame(kdl.Rotation(),
                                              kdl.Vector(-2.27, -7.55, 0.0)),
                              frame_id="map")]
object_shelves = ["shelf2", "shelf3", "shelf4", "shelf5"]
object_types = [obj["name"] for obj in common.objects]

# Grasping
grasp_surface = ["kitchen_table", "bistro_table"]
room = ["kitchen", "balcony"]
#
# Placing
default_place_entity = cabinet_amcl
default_place_area = ["shelf3", "shelf3"]

# Table pose
# N.B.: kitchen table rotated 180 deg to have in front of at the right side
table_poses = [FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, -0.0, -1.57079632679),
                                            kdl.Vector(4.16, -2.86, 0)),
                            frame_id="map"),
               FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 3.1415),
                                            kdl.Vector(0.0, -7.7, 0.0)),
                            frame_id="map")]
entity_poses = zip(cabinet_poses, table_poses, cabinet_amcl, grasp_surface, room, default_place_area)
