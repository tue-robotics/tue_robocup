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

# Detection
cabinet_amcl = ["cabinet", "storage_shelf"]
cabinet_poses = [FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0), kdl.Vector(0.144, 3.274, 0.0)),
                              frame_id="map"),
                 FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi), kdl.Vector(4.8, 2.5, 0.0)),
                              frame_id="map")]
object_shelves = ["shelf3", "shelf4", "shelf5"]
object_types = [obj["name"] for obj in common.objects]

# Grasping
grasp_surface = ["dining_table", "dining_table"]
room = ["dining_room", "dining_room"]

# Placing
default_place_entity = cabinet_amcl
default_place_area = ["shelf2", "shelf2"]

table_poses = [FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi), kdl.Vector(2.447, 1.1695, 0.0)),
                            frame_id="map"),
               FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi), kdl.Vector(2.447, 1.1695, 0.0)),
                            frame_id="map")]

entity_poses = zip(cabinet_poses,       # Where should the cabinet go?
                   table_poses,         # Where should the table go?
                   cabinet_amcl,        # In what entity to place items
                   grasp_surface,       # From what entity to grasp?
                   room,                # No usages??
                   default_place_area)  # In what area of the entity to place that area

# TODO: define a 'Workspace' class, that defines a name and estimated location for the grasp entity, the place entity, grasp/place volumes etc.
# In storing_groceries/config, do a check that this Workspace is valid


"""EntityConfiguration defines a name, an pose estimate for the Entity and which volumes of that entity to use for manipulation"""
EntityConfiguration = namedtuple("EntityConfiguration", ["entity_id", "pose_estimate", "manipulation_volumes"])

"""A Workspace for Storing Groceries has EntityConfigurations for the two main entities in the challenge:
    the grasp_entity (usually a table) and a place_entity (usually some cabinet or shelf).
    Additionally, it defines in which room the challenge takes place
    """
Workspace = namedtuple("Workspace", ["grasp_entity_conf", "place_entity_conf", "room"])

cabinet_ws = Workspace(grasp_entity_conf=
                       EntityConfiguration(entity_id="dining_table",
                                           pose_estimate=FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi),
                                                                                      kdl.Vector(2.447, 1.1695, 0.0)),
                                                                      frame_id="map"),
                                           manipulation_volumes=['on_top_of']
                                           ),
                       place_entity_conf=
                       EntityConfiguration(entity_id="cabinet",
                                           pose_estimate=FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0),
                                                                                      kdl.Vector(0.144, 3.274, 0.0)),
                                                                      frame_id="map"),
                                           manipulation_volumes=['shelf2']
                                           ),
                       room="dining_room")

storage_shelf_ws = Workspace(
                        grasp_entity_conf=
                        EntityConfiguration(entity_id="dining_table",
                                           pose_estimate=FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi),
                                                                                      kdl.Vector(2.447, 1.1695, 0.0)),
                                                                      frame_id="map"),
                                           manipulation_volumes=['on_top_of']
                                           ),
                        place_entity_conf=
                        EntityConfiguration(entity_id="storage_shelf",
                                           pose_estimate=FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi),
                                                                                      kdl.Vector(4.8, 2.5, 0.0)),
                                                                      frame_id="map"),
                                           manipulation_volumes=['shelf2']
                                           ),
                        room="dining_room")

bookcase_ws = Workspace(grasp_entity_conf=
                        EntityConfiguration(entity_id="desk",
                                           pose_estimate=FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi),
                                                                                      kdl.Vector(6.0, 2.5, 0.0)),
                                                                      frame_id="map"),
                                           manipulation_volumes=['on_top_of']
                                           ),
                        place_entity_conf=
                        EntityConfiguration(entity_id="bookcase",
                                           pose_estimate=FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, math.pi),
                                                                                      kdl.Vector(9.7, 2.0, 0.0)),
                                                                      frame_id="map"),
                                           manipulation_volumes=['shelf2']
                                           ),
                        room="bedroom")

workspaces = [cabinet_ws, storage_shelf_ws, bookcase_ws]


