from __future__ import print_function

from typing import List, Optional

# System
from dataclasses import dataclass
import os
import traceback

import numpy as np
import yaml

from cv_bridge import CvBridge
import rospy
from geometry_msgs.msg import PointStamped
from pykdl_ros import VectorStamped, FrameStamped
import rospkg
import tf2_ros
# noinspection PyUnresolvedReferences
import tf2_geometry_msgs
# noinspection PyUnresolvedReferences
import tf2_pykdl_ros
import visualization_msgs.msg

from cb_base_navigation_msgs.msg import PositionConstraint
from ed_gui_server_msgs.srv import GetEntityInfo, GetEntityInfoResponse, Map, MapRequest, MapResponse
from ed_msgs.srv import Configure, Reset, SimpleQuery, SimpleQueryRequest, UpdateSrv
from ed_navigation_msgs.srv import GetGoalConstraint
from ed_people_recognition_msgs.srv import EdRecognizePeople
from ed_perception_msgs.srv import Classify
import ed_sensor_integration_msgs.srv as ed_sensor_srv

from ed.entity import from_entity_info

# Robot skills
from robot_skills.classification_result import ClassificationResult
from robot_skills.robot_part import RobotPart
from robot_skills.util import transformations
from robot_skills.util.decorators import deprecated


@dataclass(frozen=True)
class FloorPlan:
    map: np.ndarray
    map_pose: FrameStamped
    pixels_per_meter_width: float
    pixels_per_meter_height: float


class Navigation(RobotPart):
    def __init__(self, robot_name, tf_buffer):
        super(Navigation, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self._get_constraint_srv = self.create_service_client('/%s/ed/navigation/get_constraint' % robot_name,
                                                              GetGoalConstraint)

    def get_position_constraint(self, entity_id_area_name_map):
        try:
            res = self._get_constraint_srv(entity_ids=[k for k in entity_id_area_name_map],
                                           area_names=[v for k, v in entity_id_area_name_map.items()])
        except Exception as e:
            rospy.logerr("Can't get position constraint: {}".format(e))
            return None

        if res.error_msg != '':
            rospy.logerr(res.error_msg)
            return None

        return PositionConstraint(constraint=res.position_constraint_map_frame, frame="map")


class ED(RobotPart):

    def __init__(self, robot_name, tf_buffer):
        super(ED, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self._ed_simple_query_srv = self.create_service_client('/%s/ed/simple_query' % robot_name, SimpleQuery)
        self._ed_entity_info_query_srv = self.create_service_client('/%s/ed/gui/get_entity_info' % robot_name,
                                                                    GetEntityInfo)
        self._ed_update_srv = self.create_service_client('/%s/ed/update' % robot_name, UpdateSrv)
        self._ed_kinect_update_srv = self.create_service_client('/%s/ed/kinect/update' % robot_name,
                                                                ed_sensor_srv.Update)
        self._ed_classify_srv = self.create_service_client('/%s/ed/classify' % robot_name, Classify)
        self._ed_configure_srv = self.create_service_client('/%s/ed/configure' % robot_name, Configure)
        self._ed_reset_srv = self.create_service_client('/%s/ed/reset' % robot_name, Reset)
        self._ed_get_image_srv = self.create_service_client('/%s/ed/kinect/get_image' % robot_name,
                                                            ed_sensor_srv.GetImage)
        self._ed_ray_trace_srv = self.create_service_client('/%s/ed/ray_trace' % robot_name,
                                                            ed_sensor_srv.RayTrace)

        self._ed_detect_people_srv = self.create_service_client('/%s/ed/people_recognition/detect_people' % robot_name,
                                                                EdRecognizePeople)

        self._ed_map_srv = self.create_service_client(f"/{robot_name}/ed/gui/map", Map)

        self.navigation = Navigation(robot_name, tf_buffer)

        self._marker_publisher = rospy.Publisher("/" + robot_name + "/ed/simple_query", visualization_msgs.msg.Marker,
                                                 queue_size=10)
        self.__cv_bridge = None

    def wait_for_connections(self, timeout, log_failing_connections=True):
        """
        Waits for the connections until they are connected

        :param timeout: timeout in seconds
        :param log_failing_connections: (bool) whether to log errors if not connected. This is useful when checking
            multiple robot parts in a loop
        :return: bool indicating whether all connections are connected
        """
        return (super(ED, self).wait_for_connections(timeout, log_failing_connections) and
                self.navigation.wait_for_connections(timeout, log_failing_connections)
                )

    @property
    def _cv_bridge(self):
        if self.__cv_bridge is None:
            self.__cv_bridge = CvBridge()

        return self.__cv_bridge

    # ----------------------------------------------------------------------------------------------------
    #                                             QUERYING
    # ----------------------------------------------------------------------------------------------------

    def get_entities(self, etype="", center_point=None, radius=float('inf'), uuid="", ignore_z=False):
        """
        Get entities via Simple Query interface

        :param etype: Type of entity
        :param center_point: Point from which radius is measured
        :param radius: Distance between center_point and entity
        :param uuid: ID of entity
        :param ignore_z: Consider only the distance in the X,Y plane for the radius from center_point
        """
        if center_point is None:
            center_point = VectorStamped.from_xyz(0, 0, 0, rospy.Time(), frame_id=self.robot_name+"/base_link")
        self._publish_marker(center_point, radius)

        center_point_in_map = self.tf_buffer.transform(center_point, "map", new_type=PointStamped)
        query = SimpleQueryRequest(id=uuid, type=etype, center_point=center_point_in_map.point,
                                   radius=radius, ignore_z=ignore_z)

        try:
            entity_infos = self._ed_simple_query_srv(query).entities
            entities = list(map(from_entity_info, entity_infos))
        except Exception as e:
            rospy.logerr(
                f"get_entities(uuid={uuid}, etype={etype}, center_point={center_point}, radius={radius}, "
                f"ignore_z={ignore_z})\n{e}, "
                f"{traceback.format_exc()}")
            return []

        return entities

    def get_closest_entity(self, etype="", center_point=None, radius=float('inf')):
        if not center_point:
            center_point = VectorStamped.from_xyz(0, 0, 0, rospy.Time(), frame_id=self.robot_name+"/base_link")

        entities = self.get_entities(etype=etype, center_point=center_point, radius=radius)

        # HACK
        entities = [e for e in entities if e.shape is not None and e.etype != ""]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            center_in_map = self.tf_buffer.transform(center_point, "map")
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(center_in_map.vector))
        except Exception as e:
            rospy.logerr("Failed to sort entities: {}".format(e))
            return None

        return entities[0]

    def get_closest_room(self, center_point=None, radius=float('inf')):
        if not center_point:
            center_point = VectorStamped.from_xyz(0, 0, 0, rospy.Time(), frame_id=self.robot_name+"/base_link")

        return self.get_closest_entity(etype="room", center_point=center_point, radius=radius)

    def get_closest_laser_entity(self, etype="", center_point=None, radius=float('inf'), ignore_z=False):
        """
        Get the closest entity detected by the laser. The ID's of such entities are postfixed with '-laser'
        For the rest, this works exactly like get_closest_entity

        :param type: What type of entities to filter on
        :param center_point: combined with radius. Around which point to search for entities
        :param radius: how far from the center_point to look (in meters)
        :param ignore_z: Consider only the distance in the X,Y plane for the radius from center_point criterium.
        :return: list of Entity
        """
        if center_point is None:
            center_point = VectorStamped.from_xyz(0, 0, 0, rospy.Time(), frame_id=self.robot_name+"/base_link")

        entities = self.get_entities(etype="", center_point=center_point, radius=radius, ignore_z=ignore_z)

        # HACK
        entities = [e for e in entities if e.shape and e.etype == "" and e.uuid.endswith("-laser")]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(
                self.tf_buffer.transform(center_point, self.robot_name+"/base_link", new_type=VectorStamped).vector))
            # TODO: adjust for robot
        except Exception as e:
            rospy.logerr("Failed to sort entities: {}".format(e))
            return None

        return entities[0]

    def get_entity(self, uuid):
        entities = self.get_entities(uuid=uuid)
        if len(entities) == 0:
            rospy.logerr("Could not get_entity(uuid='{}')".format(uuid))
            return None

        return entities[0]

    def get_entity_info(self, uuid):
        try:
            return self._ed_entity_info_query_srv(uuid=uuid, measurement_image_border=20)
        except rospy.ServiceException as e:
            rospy.logerr("Cant get entity info of id='{}': {}".format(id, e))
            return GetEntityInfoResponse()

    def get_map(
        self, uuids: List[str], background: str = "white", print_labels: bool = True, width: int = 0, height: int = 0
    ) -> Optional[FloorPlan]:
        """
        :param uuids: Entities that should be in view in the generated map
        :param background: Background color of the map
        :param print_labels: Should entity labels be printed
        :param width: image width (default: 0, defaults to 1024)
        :param height: image height (default: 0, defaults to 600)
        :returns: The generated map, the position of the top-left corner of the image and the pixels/meter in both axis
        """
        req = MapRequest()
        req.entities_in_view = uuids
        req.background = req.WHITE  # default
        if hasattr(req, background.upper()):
            req.background = getattr(req, background.upper())
        else:
            rospy.logwarn(f"[ed.get_map] provided background color doesn't exist: '{req.background}'."
                          f"Using default: 'WHITE'")
        req.print_labels = print_labels
        req.image_width = width
        req.image_height = height

        try:
            res = self._ed_map_srv.call(req)  # type: MapResponse
        except Exception as e:
            rospy.logerr(f"Could not get ED map for entities: {uuids}\nreason: {e}")
            return None

        floormap = self._cv_bridge.imgmsg_to_cv2(res.map, 'bgr8')
        fs = tf2_ros.convert(res.pose, FrameStamped)

        return FloorPlan(floormap, fs, res.pixels_per_meter_width, res.pixels_per_meter_height)

    # ----------------------------------------------------------------------------------------------------
    #                                             UPDATING
    # ----------------------------------------------------------------------------------------------------

    def selfreset(self, keep_all_shapes=True):
        try:
            return self._ed_reset_srv(keep_all_shapes=keep_all_shapes)
        except rospy.ServiceException as e:
            rospy.logerr("Could not reset ED: {0}".format(e))
            return False

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def update_entity(self, uuid, etype=None, frame_stamped=None, flags=None, add_flags=None, remove_flags=None, action=None):
        """
        Updates entity

        :param uuid: entity id
        :param etype: entity type
        :param frame_stamped: If specified, the entity is updated to be at this FrameStamped
        :param flags: (OBSOLETE, use add_flags and remove_flags): (list of) dict(s) containing key 'add' or 'remove' and value of the flag to set,  e.g., 'perception'
        :param add_flags: list of flags which will be added to the specified entity
        :param remove_flags: list of flags which will removed from the specified entity
        :param action: update_action, e.g. remove
        """
        if add_flags is None:
            add_flags = []
        if remove_flags is None:
            remove_flags = []
        json_entity = '"id" : "%s"' % uuid
        if etype:
            json_entity += ', "type": "%s"' % etype

        if action:
            json_entity += ', "action": "%s"' % action

        if frame_stamped:
            if frame_stamped.header.frame_id != "/map" and frame_stamped.header.frame_id != "map":
                rospy.loginfo('update_entity: frame not in map, transforming')
                frame_stamped = self.tf_buffer.transform(frame_stamped, "map")

            Z, Y, X = frame_stamped.frame.M.GetEulerZYX()
            t = frame_stamped.frame.p
            json_entity += ', "pose": { "x": %f, "y": %f, "z": %f, "X": %f, "Y": %f, "Z": %f }' % (
            t.x(), t.y(), t.z(), X, Y, Z)

        if flags or add_flags or remove_flags:
            json_entity += ', "flags": ['
            first = True

            if isinstance(flags, dict):
                flags = [flags]

            if isinstance(flags, list):
                for flag in flags:
                    if not isinstance(flag, dict):
                        print("update_entity - Error: flags need to be a list of dicts or a dict")
                        return False
                    for k, v in flag.items():
                        if not first:
                            json_entity += ','
                        json_entity += '{"%s":"%s"}' % (k, v)
                        first = False

            for flag in add_flags:
                if not first:
                    json_entity += ','
                json_entity += '{"add":"%s"}' % flag
                first = False

            for flag in remove_flags:
                if not first:
                    json_entity += ','
                json_entity += '{"remove":"%s"}' % flag
                first = False

            json_entity += ']'

        json = '{"entities":[{%s}]}' % json_entity
        rospy.logdebug(json)

        return self._ed_update_srv(request=json)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def remove_entity(self, uuid):
        """ Removes entity with the provided id to the world model

        :param uuid: string with the ID of the entity to remove
        """
        return self.update_entity(uuid=uuid, action="remove")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def lock_entities(self, lock_ids, unlock_ids):
        for uuid in lock_ids:
            self.update_entity(uuid=uuid, add_flags=['locked'])

        for uuid in unlock_ids:
            self.update_entity(uuid=uuid, remove_flags=['locked'])

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def get_closest_possible_person_entity(self, center_point=None, radius=float('inf')):
        """ Returns the 'possible_human' entity closest to a certain center point.

        :param center_point: (VectorStamped) indicating where the human should be close to
        :param radius: (float) radius to look for possible humans
        :return: (Entity) entity (if found), None otherwise
        """
        if center_point is None:
            center_point = VectorStamped.from_xyz(0, 0, 0, rospy.Time(), frame_id=self.robot_name+"/base_link")
            center_point = self.tf_buffer.transform(center_point, "map")
        assert center_point.header.frame_id.endswith("map"), "Other frame ids not yet implemented"

        # Get all entities
        entities = self.get_entities(etype="", center_point=center_point, radius=radius)

        # Filter on 'possible humans'
        entities = [e for e in entities if e.is_a('possible_human')]

        # If nothing found, return None
        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(center_point.vector))
            rospy.logdebug("entities sorted closest to robot = {}".format(entities))
        except Exception as e:
            rospy.logerr("Failed to sort entities: {}".format(e))
            return None

        return entities[0]

    # ----------------------------------------------------------------------------------------------------
    #                                  KINECT INTEGRATION AND PERCEPTION
    # ----------------------------------------------------------------------------------------------------

    def update_kinect(self, area_description="", background_padding=0):
        """
        Update ED based on kinect (depth) images

        :param area_description: An entity id or area description, e.g. "a08d537e-e051-11e5-a34e-6cc217ec9f41" or "on_top_of cabinet-11"
        :param background_padding: The maximum distance to which kinect data points are associated to existing objects (in meters).
               Or, in other words: the padding that is added to existing objects before they are removed from the point cloud
        :return: Update result
        """
        # Check the area description
        if area_description == "":
            rospy.logwarn("No area_description provided for 'update_kinect'. This is probably a bad idea.")

        # Save the image (logging)
        self.save_image(path_suffix=area_description.replace(" ", "_"))

        res = self._ed_kinect_update_srv(area_description=area_description, background_padding=background_padding)
        if res.error_msg:
            rospy.logerr("Could not segment objects: %s" % res.error_msg)

        return res

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def classify(self, uuids, types=None, unknown_threshold=0.0):
        # type: (List[str], List[str], float) -> List[ClassificationResult]
        """
        Classifies the entities with the given IDs

        :param ids: list with IDs
        :type ids: List[str]
        :param types: list with types to identify
        :type: types: List[str]
        :param unknown_threshold: objects with a probability lower than this unknown_threshold are not set as a type
        :type unknown_threshold: float
        :return: List of classification results
        :rtype: List[ClassificationResult]
        """

        res = self._ed_classify_srv(ids=uuids, unknown_probability=unknown_threshold)
        if res.error_msg:
            rospy.logerr("While classifying entities: %s" % res.error_msg)

        posteriors = [dict(zip(distr.values, distr.probabilities)) for distr in res.posteriors]

        # Filter on types if types is not None
        return [ClassificationResult(uuid, exp_val, exp_prob, distr) for uuid, exp_val, exp_prob, distr
                in zip(res.ids, res.expected_values, res.expected_value_probabilities, posteriors) if
                types is None or exp_val in types]

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def save_image(self, path="", path_suffix="", filename=""):
        import os
        import time

        if not path:
            home_dir = os.environ["HOME"]
            path = home_dir + "/ed/kinect/" + time.strftime("%Y-%m-%d")
            if path_suffix:
                path += "/" + path_suffix

        if not os.path.exists(path):
            os.makedirs(path)

        if not filename:
            filename = time.strftime("%Y-%m-%d-%H-%M-%S")

        fname = path + "/" + filename

        res = self._ed_get_image_srv(filename=filename)
        if res.error_msg:
            rospy.logerr("Could not save image: %s" % res.error_msg)

        with open(fname + ".rgbd", "wb") as f:
            f.write(bytearray(res.rgbd_data))

        with open(fname + ".json", "w") as f:
            f.write(res.json_meta_data)

        # rgbd to png
        os.system('rosrun rgbd rgbd_to_png %s' % (fname + ".rgbd"))  # ToDo: very very very ugly

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    @deprecated
    def mesh_entity_in_view(self, uuid, etype=""):
        # Takes the biggest one in view
        # return self._ed_mesh_entity_in_view_srv(uuid=uuid, etype=etype)
        rospy.logwarn("[world_model_ed.py] Function 'mesh_entity_in_view' is obsolete.")
        return None

    # ----------------------------------------------------------------------------------------------------
    #                                                MISC
    # ----------------------------------------------------------------------------------------------------

    def get_full_id(self, short_id):
        """Get an entity's full ID based on the first characters of its ID like you can do with git hashes"""
        all_entities = self.get_entities()
        matches = filter(lambda fill_id: fill_id.startswith(short_id), [entity.uuid for entity in all_entities])
        return matches

    def ray_trace(self, pose):
        """ Performs a ray trace

        :param pose: geometry_msg PoseStamped. Position is the origin of ray. x-axis is pointing in the ray direction
        :return: RayTraceResult. This struct contains intersection_point (geometry_msgs/PoseStamped) and entity_id
            (string)
        """
        return self._ed_ray_trace_srv(raytrace_pose=pose)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def _transform_center_point_to_map(self, pointstamped):
        point_in_map = transformations.tf_transform(pointstamped.point, pointstamped.header.frame_id, "map",
                                                    self.tf_buffer)
        return point_in_map

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def _publish_marker(self, center_point, radius):
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = center_point.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = 2

        marker.pose.position.x = center_point.vector.x()
        marker.pose.position.y = center_point.vector.y()
        marker.pose.position.z = center_point.vector.z()
        marker.lifetime = rospy.Duration(20.0)
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius

        marker.color.a = 0.5
        marker.color.r = 1

        self._marker_publisher.publish(marker)

    def sync_furniture_poses(self):
        """ Syncs the furniture poses of the world model to disc. Hereto, it
        i) gets all entities and filters these on being furniture
        ii) loads the current ed object models file
        iii) iterates over the furniture object entities (i) and the data in the dict (ii). If the IDs match, the pose
             of (ii) is updated with the pose of (i)
        iv) dump the updated dict to the models file

        """
        # Get all entities
        entities = self.get_entities()
        furniture_objects = [e for e in entities if e.is_a("furniture")]

        # Load the model.yaml of the current environment
        # ToDo: make sure that this works
        filename = os.path.join(rospkg.RosPack().get_path("ed_object_models"), "models", os.environ.get("ROBOT_ENV"),
                                "model.yaml")
        with open(filename, "r") as f:
            file_data = yaml.load(f)

        composition_list = file_data.get("composition", None)
        if composition_list is None:
            raise RuntimeError("No composition in {}".format(filename))

        # Iterate over all furniture objects
        for wm_object in furniture_objects:
            for file_object in composition_list:
                if wm_object.uuid == file_object.get("id", ""):
                    # Yeehah, we found something we need to update
                    _, _, Z = wm_object.pose.frame.M.GetRPY()
                    pos = wm_object.pose.frame.p
                    file_object['pose'] = {'x': pos.x(), 'y': pos.y(), 'z': pos.z(), 'Z': Z}

        # Dump the data to file
        with open(filename, "w") as f:
            yaml.dump(file_data, f)

    def detect_people(self, rgb, depth, cam_info):
        """
        Detect people in the given color message, depth image aided by the depth camera's camera info

        :param rgb: Color image
        :type rgb: sensor_msgs/Image
        :param depth: Depth image
        :type depth: sensor_msgs/Image
        :param cam_info: CamInfo for the camera that recorded the depth image.
        :type cam_info: sensor_msgs/CamInfo
        :return: bool success and a list strings with the IDs of the detected persons
        :rtype: (bool, [str])
        """
        try:
            result = self._ed_detect_people_srv(rgb, depth, cam_info)
            return result.success, result.detected_person_ids
        except Exception as e:
            rospy.logerr('_ed_detect_people_srv failed!: {}'.format(e))
            return False, []
