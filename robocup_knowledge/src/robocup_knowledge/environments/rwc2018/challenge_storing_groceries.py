# STORING GROCERIES KNOWLEDGE FILE RWC2018

# System
import math

# ROS
import PyKDL as kdl

# TU/e Robotics
from robot_skills.util.kdl_conversions import FrameStamped
from robocup_knowledge import knowledge_loader
from collections import namedtuple

# Common knowledge
common = knowledge_loader.load_knowledge("common")

table_name = "storage_table"

# Detection
cabinet_amcl = ["cabinet", "storage_shelf"]
cabinet_poses = [FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0), kdl.Vector(0.144, 3.274, 0.0)),
                              frame_id="map"),
                 FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi), kdl.Vector(4.8, 2.5, 0.0)),
                              frame_id="map")]
object_shelves = ["shelf3", "shelf4", "shelf5"]
object_types = [obj["name"] for obj in common.objects]

# Placing
default_place_entity = cabinet_amcl


"""
EntityConfiguration defines a name, an pose estimate for the Entity and which volumes of that entity to use for
manipulation
"""
EntityConfiguration = namedtuple("EntityConfiguration", ["entity_id", "pose_estimate", "manipulation_volumes"])

"""
A Workspace for Storing Groceries has EntityConfigurations for the two main entities in the challenge:
the grasp_entity (usually a table) and a place_entity (usually some cabinet or shelf).
Additionally, it defines in which room the challenge takes place
"""
Workspace = namedtuple("Workspace", ["grasp_entity_conf", "place_entity_conf", "room"])

cupboard_ws = Workspace(grasp_entity_conf=
                       EntityConfiguration(entity_id="storage_table",
                                           pose_estimate=
                                           FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi),
                                                                        kdl.Vector(-2.82, 5.73, 0.0)),
                                                        frame_id="map"),
                                           manipulation_volumes=['on_top_of']
                                           ),
                       place_entity_conf=
                       EntityConfiguration(entity_id="cupboard",
                                           pose_estimate=FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi),
                                                                                      kdl.Vector(-2.69, 6.45, 0.0)),
                                                                      frame_id="map"),
                                           manipulation_volumes=['shelf2']
                                           ),
                       room="kitchen")

bookcase_ws = Workspace(
                        grasp_entity_conf=
                        EntityConfiguration(entity_id="storage_table",
                                            pose_estimate=
                                            FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi/2),
                                                                         kdl.Vector(-2.46, 1.24, 0.0)),
                                                         frame_id="map"),
                                            manipulation_volumes=['on_top_of']
                                            ),
                        place_entity_conf=
                        EntityConfiguration(entity_id="bookcase",
                                            pose_estimate=
                                            FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi/2),
                                                                         kdl.Vector(-1.46, 0.24, 0.0)),
                                                         frame_id="map"),
                                            manipulation_volumes=['shelf2']
                                            ),
                        room="dining_room")

workspaces = [cupboard_ws, bookcase_ws]
