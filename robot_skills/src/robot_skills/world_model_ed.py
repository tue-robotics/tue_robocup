# System
import os
import yaml
from math import sqrt

# ROS
import rospkg
import rospy
import visualization_msgs.msg

# TU/e
import ed_msgs.srv
from ed_msgs.srv import SimpleQuery, SimpleQueryRequest, UpdateSrv, Configure
import ed_sensor_integration_msgs.srv as ed_sensor_srv
from ed_people_recognition_msgs.srv import EdRecognizePeople
from ed_perception_msgs.srv import Classify
from ed_gui_server_msgs.srv import GetEntityInfo, GetEntityInfoResponse
from ed_navigation_msgs.srv import GetGoalConstraint
from cb_planner_msgs_srvs.msg import PositionConstraint

# Robot skills
from robot_skills.util import transformations
from robot_skills.util.decorators import deprecated
from robot_skills.util.kdl_conversions import VectorStamped, kdl_vector_to_point_msg
from robot_skills.classification_result import ClassificationResult
from robot_skills.util.entity import from_entity_info
from robot_skills.robot_part import RobotPart


class Navigation(RobotPart):
    def __init__(self, robot_name, tf_listener):
        super(Navigation, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._get_constraint_srv = self.create_service_client('/%s/ed/navigation/get_constraint' % robot_name,
                                                              GetGoalConstraint)

    def get_position_constraint(self, entity_id_area_name_map):
        try:
            res = self._get_constraint_srv(entity_ids=[k for k in entity_id_area_name_map],
                                           area_names=[v for k, v in entity_id_area_name_map.iteritems()])
        except Exception as e:
            rospy.logerr("Can't get position constraint: {}".format(e))
            return None

        if res.error_msg != '':
            rospy.logerr(res.error_msg)
            return None

        return PositionConstraint(constraint=res.position_constraint_map_frame, frame="/map")


class ED(RobotPart):

    def __init__(self, robot_name, tf_listener):
        super(ED, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._ed_simple_query_srv = self.create_service_client('/%s/ed/simple_query' % robot_name, SimpleQuery)
        self._ed_entity_info_query_srv = self.create_service_client('/%s/ed/gui/get_entity_info' % robot_name,
                                                                    GetEntityInfo)
        self._ed_update_srv = self.create_service_client('/%s/ed/update' % robot_name, UpdateSrv)
        self._ed_kinect_update_srv = self.create_service_client('/%s/ed/kinect/update' % robot_name,
                                                                ed_sensor_srv.Update)
        self._ed_classify_srv = self.create_service_client('/%s/ed/classify' % robot_name, Classify)
        self._ed_configure_srv = self.create_service_client('/%s/ed/configure' % robot_name, Configure)
        self._ed_reset_srv = self.create_service_client('/%s/ed/reset' % robot_name, ed_msgs.srv.Reset)
        self._ed_get_image_srv = self.create_service_client('/%s/ed/kinect/get_image' % robot_name,
                                                            ed_sensor_srv.GetImage)
        self._ed_ray_trace_srv = self.create_service_client('/%s/ed/ray_trace' % robot_name,
                                                            ed_sensor_srv.RayTrace)

        self._ed_detect_people_srv = self.create_service_client('/%s/ed/people_recognition/detect_people' % robot_name,
                                                                EdRecognizePeople)

        self.navigation = Navigation(robot_name, tf_listener)

        self._marker_publisher = rospy.Publisher("/" + robot_name + "/ed/simple_query", visualization_msgs.msg.Marker,
                                                 queue_size=10)

        self.robot_name = robot_name

    def wait_for_connections(self, timeout, log_failing_connections=True):
        """ Waits for the connections until they are connected

        :param timeout: timeout in seconds
        :param log_failing_connections: (bool) whether to log errors if not connected. This is useful when checking
            multiple robot parts in a loop
        :return: bool indicating whether all connections are connected
        """
        return (super(ED, self).wait_for_connections(timeout, log_failing_connections) and
                self.navigation.wait_for_connections(timeout, log_failing_connections)
                )

    # ----------------------------------------------------------------------------------------------------
    #                                             QUERYING
    # ----------------------------------------------------------------------------------------------------

    def get_entities(self, type="", center_point=VectorStamped(), radius=float('inf'), id="", parse=True):
        self._publish_marker(center_point, radius)

        center_point_in_map = center_point.projectToFrame("/map", self.tf_listener)
        query = SimpleQueryRequest(id=id, type=type, center_point=kdl_vector_to_point_msg(center_point_in_map.vector),
                                   radius=radius)

        try:
            entity_infos = self._ed_simple_query_srv(query).entities
            entities = map(from_entity_info, entity_infos)
        except Exception as e:
            rospy.logerr("ERROR: robot.ed.get_entities(id=%s, type=%s, center_point=%s, radius=%s)" % (
                id, type, str(center_point), str(radius)))
            rospy.logerr("L____> [%s]" % e)
            return []

        return entities

    def get_closest_entity(self, type="", center_point=None, radius=float('inf')):
        if not center_point:
            center_point = VectorStamped(x=0, y=0, z=0, frame_id="/" + self.robot_name + "/base_link")

        entities = self.get_entities(type=type, center_point=center_point, radius=radius)

        # HACK
        entities = [e for e in entities if e.shape is not None and e.type != ""]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            center_in_map = center_point.projectToFrame("/map", self.tf_listener)
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(center_in_map.vector))
        except Exception as e:
            rospy.logerr("Failed to sort entities: {}".format(e))
            return None

        return entities[0]

    def get_closest_room(self, center_point=None, radius=float('inf')):
        if not center_point:
            center_point = VectorStamped(x=0, y=0, z=0, frame_id="/" + self.robot_name + "/base_link")

        return self.get_closest_entity(type="room", center_point=center_point, radius=radius)

    def get_closest_laser_entity(self, type="", center_point=VectorStamped(), radius=float('inf'), ignore_z=False):
        """
        Get the closest entity detected by the laser. The ID's of such entities are postfixed with '-laser'
        For the rest, this works exactly like get_closest_entity

        :param type: What type of entities to filter on
        :param center_point: combined with radius. Around which point to search for entities
        :param radius: how far from the center_point to look (in meters)
        :param ignore_z: Consider only the distance in the X,Y plane for the radius from center_point criterium.
        :return: list of Entity
        """
        if ignore_z:
            # ED does not allow to ignore Z through its interface, so it has to be solved here.
            #
            # If we want to ignore the z of entities when checking their distance from center_point,
            # the simplest thing to do is to set the Z of the center_point to the same value,
            # so that delta Z is always 0.
            # But then we first need to know Z (preferably without an additonal parameter or something robot specific)
            # So, we query ED for other laser entities. These are *assumed* to all have the same Z,
            # so we can just take one and use its Z and substitute that into the center_point.
            # To 'just take one', we take entities from a larger range.
            entities_for_height = self.get_entities(type="", center_point=center_point, radius=sqrt(2**2 + radius**2))
            if entities_for_height:
                override_z = entities_for_height[0].frame.extractVectorStamped().vector.z()
                rospy.logwarn("ignoring Z, so overriding z to be equal to laser height: {}".format(override_z))
                center_point = VectorStamped(center_point.vector.x(),
                                             center_point.vector.y(),
                                             override_z)
            else:
                return None

        entities = self.get_entities(type="", center_point=center_point, radius=radius)

        # HACK
        entities = [e for e in entities if e.shape and e.type == "" and e.id.endswith("-laser")]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(
                center_point.projectToFrame("/%s/base_link" % self.robot_name,
                                            self.tf_listener).vector))  # TODO: adjust for robot
        except Exception as e:
            rospy.logerr("Failed to sort entities: {}".format(e))
            return None

        return entities[0]

    def get_entity(self, id, parse=True):
        entities = self.get_entities(id=id, parse=parse)
        if len(entities) == 0:
            rospy.logwarn("Could not get_entity(id='{}')".format(id))
            return None

        return entities[0]

    def get_entity_info(self, id):
        try:
            return self._ed_entity_info_query_srv(id=id, measurement_image_border=20)
        except rospy.ServiceException as e:
            rospy.logerr("Cant get entity info of id='{}': {}".format(id, e))
            return GetEntityInfoResponse()

    # ----------------------------------------------------------------------------------------------------
    #                                             UPDATING
    # ----------------------------------------------------------------------------------------------------

    def reset(self, keep_all_shapes=True):
        try:
            return self._ed_reset_srv(keep_all_shapes=keep_all_shapes)
        except rospy.ServiceException as e:
            rospy.logerr("Could not reset ED: {0}".format(e))
            return False

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def update_entity(self, id, type=None, frame_stamped=None, flags=None, add_flags=[], remove_flags=[], action=None):
        """
        Updates entity

        :param id: entity id
        :param type: entity type
        :param frame_stamped: If specified, the entity is updated to be at this FrameStamped
        :param flags: (OBSOLETE, use add_flags and remove_flags): (list of) dict(s) containing key 'add' or 'remove' and value of the flag to set,  e.g., 'perception'
        :param add_flags: list of flags which will be added to the specified entity
        :param remove_flags: list of flags which will removed from the specified entity
        :param action: update_action, e.g. remove
        """
        json_entity = '"id" : "%s"' % id
        if type:
            json_entity += ', "type": "%s"' % type

        if action:
            json_entity += ', "action": "%s"' % action

        if frame_stamped:
            if frame_stamped.frame_id != "/map":
                rospy.loginfo('update_entity: frame not in map, transforming')
                frame_stamped = frame_stamped.projectToFrame("/map", self.tf_listener)

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
                    for k, v in flag.iteritems():
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

    def remove_entity(self, id):
        """ Removes entity with the provided id to the world model

        :param id: string with the ID of the entity to remove
        """
        return self.update_entity(id=id, action="remove")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def lock_entities(self, lock_ids, unlock_ids):
        for eid in lock_ids:
            self.update_entity(id=eid, add_flags=['locked'])

        for eid in unlock_ids:
            self.update_entity(id=eid, remove_flags=['locked'])

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def get_closest_possible_person_entity(self, center_point=VectorStamped(), radius=float('inf')):
        """ Returns the 'possible_human' entity closest to a certain center point.

        :param center_point: (VectorStamped) indicating where the human should be close to
        :param radius: (float) radius to look for possible humans
        :return: (Entity) entity (if found), None otherwise
        """
        assert center_point.frame_id.endswith("map"), "Other frame ids not yet implemented"

        # Get all entities
        entities = self.get_entities(type="", center_point=center_point, radius=radius)

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

    def classify(self, ids, types=None, unknown_threshold=0.0):
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

        res = self._ed_classify_srv(ids=ids, unknown_probability=unknown_threshold)
        if res.error_msg:
            rospy.logerr("While classifying entities: %s" % res.error_msg)

        posteriors = [dict(zip(distr.values, distr.probabilities)) for distr in res.posteriors]

        # Filter on types if types is not None
        return [ClassificationResult(_id, exp_val, exp_prob, distr) for _id, exp_val, exp_prob, distr
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
        os.system('rosrun rgbd rgbd_to_rgb_png %s' % (fname + ".rgbd"))  # ToDo: very very very ugly

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    @deprecated
    def mesh_entity_in_view(self, id, type=""):
        # Takes the biggest one in view
        # return self._ed_mesh_entity_in_view_srv(id=id, type=type)
        rospy.logwarn("[world_model_ed.py] Function 'mesh_entity_in_view' is obsolete.")
        return None

    # ----------------------------------------------------------------------------------------------------
    #                                                MISC
    # ----------------------------------------------------------------------------------------------------

    def get_full_id(self, short_id):
        """Get an entity's full ID based on the first characters of its ID like you can do with git hashes"""
        all_entities = self.get_entities(parse=False)
        matches = filter(lambda fill_id: fill_id.startswith(short_id), [entity.id for entity in all_entities])
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
        point_in_map = transformations.tf_transform(pointstamped.point, pointstamped.header.frame_id, "/map",
                                                    self.tf_listener)
        return point_in_map

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def _publish_marker(self, center_point, radius):
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = center_point.frame_id
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
                if wm_object.id == file_object.get("id", ""):
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
