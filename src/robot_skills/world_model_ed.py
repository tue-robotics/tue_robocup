#! /usr/bin/env python
import rospy
from ed.srv import SimpleQuery, SimpleQueryRequest, UpdateSrv, Configure
# from ed_sensor_integration.srv import LockEntities, MeshEntityInView, Segment
import ed_sensor_integration.srv
import ed_perception.srv
from ed_perception.srv import Classify, AddTrainingInstance
from ed_gui_server.srv import GetEntityInfo
from ed_navigation.srv import GetGoalConstraint
from cb_planner_msgs_srvs.msg import PositionConstraint
from geometry_msgs.msg import Point, PointStamped
from math import hypot

from .util import transformations

import ed.srv
from std_srvs.srv import Empty

import tf
import visualization_msgs.msg

import os


import yaml

from .classification_result import ClassificationResult


class Navigation:
    def __init__(self, robot_name, tf_listener, wait_service=False):
        self._get_constraint_srv = rospy.ServiceProxy('/%s/ed/navigation/get_constraint'%robot_name, GetGoalConstraint)

    def get_position_constraint(self, entity_id_area_name_map):
        try:
            res = self._get_constraint_srv(entity_ids=[ k for k in entity_id_area_name_map ], area_names=[ v for k,v in entity_id_area_name_map.iteritems() ])
        except Exception, e:
            rospy.logerr(e)
            return None

        if res.error_msg != '':
            rospy.logerr(res.error_msg)
            return None

        return PositionConstraint(constraint=res.position_constraint_map_frame, frame="/map")

class ED:

    def __init__(self, robot_name, tf_listener, wait_service=False):
        self._ed_simple_query_srv = rospy.ServiceProxy('/%s/ed/simple_query'%robot_name, SimpleQuery)
        self._ed_entity_info_query_srv = rospy.ServiceProxy('/%s/ed/gui/get_entity_info'%robot_name, GetEntityInfo)
        self._ed_update_srv = rospy.ServiceProxy('/%s/ed/update'%robot_name, UpdateSrv)
        # self._ed_lock_entities_srv = rospy.ServiceProxy('/%s/ed/kinect/lock_entities'%robot_name, LockEntities)
        # self._ed_mesh_entity_in_view_srv = rospy.ServiceProxy('/%s/ed/kinect/mesh_entity_in_view'%robot_name, MeshEntityInView)
        # self._ed_segment_srv = rospy.ServiceProxy('/%s/ed/kinect/segment'%robot_name, Segment)
        self._ed_kinect_update_srv = rospy.ServiceProxy('/%s/ed/kinect/update'%robot_name, ed_sensor_integration.srv.Update)

        self._ed_classify_srv = rospy.ServiceProxy('/%s/ed/classify'%robot_name, Classify)
        self._ed_perception_add_training_instance_srv = rospy.ServiceProxy('/%s/ed/add_training_instance'%robot_name, AddTrainingInstance)
        self._ed_configure_srv = rospy.ServiceProxy('/%s/ed/configure'%robot_name, Configure)

        self._ed_reset_srv = rospy.ServiceProxy('/%s/ed/reset'%robot_name, ed.srv.Reset)

        self._ed_get_image_srv = rospy.ServiceProxy('/%s/ed/kinect/get_image'%robot_name, ed_sensor_integration.srv.GetImage)

        # Person recognition
        self._learn_person_srv = rospy.ServiceProxy('/%s/learn_person'%robot_name, ed_perception.srv.LearnPerson)
        self._clear_persons_srv = rospy.ServiceProxy('/%s/clear_persons' % robot_name, Empty)
        self._recognize_person_srv = rospy.ServiceProxy('/%s/recognize_person'%robot_name, ed_perception.srv.RecognizePerson)

        self._tf_listener = tf_listener

        self.navigation = Navigation(robot_name, tf_listener, wait_service)

        self._marker_publisher = rospy.Publisher("/" + robot_name + "/ed/simple_query",  visualization_msgs.msg.Marker, queue_size=10)

    # ----------------------------------------------------------------------------------------------------
    #                                           CONFIGURATION
    # ----------------------------------------------------------------------------------------------------

    def enable_plugins(self, plugin_names):
        """Enables the specified plugins in ED"""
        return self._set_plugin_status(plugin_names, '"enabled":1')

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def disable_plugins(self, plugin_names):
        """Disables the specified plugins in ED"""
        return self._set_plugin_status(plugin_names, '"enabled":0')

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def _set_plugin_status(self, plugin_names, status):
        if not plugin_names:
            return

        yaml = '{"plugins":['
        for name in plugin_names:
            yaml += '{"name":"%s",%s},' % (name, status)

        # remove last comma
        yaml = yaml[:-1]
        yaml += ']}'

        resp = self._ed_configure_srv(request=yaml)
        if not resp.error_msg:
            return True

        rospy.logerr("[ED]: While requesting '%s': %s" % (yaml, resp.error_msg))
        return False


    # ----------------------------------------------------------------------------------------------------
    #                                             QUERYING
    # ----------------------------------------------------------------------------------------------------

    def get_entities(self, type="", center_point=Point(), radius=0, id="", parse=True):
        if isinstance(center_point, PointStamped):
            center_point = self._transform_center_point_to_map(center_point)

        self._publish_marker(center_point, radius)

        query = SimpleQueryRequest(id=id, type=type, center_point=center_point, radius=radius)

        try:
            entities = self._ed_simple_query_srv(query).entities
        except Exception, e:
            rospy.logerr("ERROR: robot.ed.get_entities(id=%s, type=%s, center_point=%s, radius=%s)" % (id, type, str(center_point), str(radius)))
            rospy.logerr("L____> [%s]" % e)
            return []

        # Parse to data strings to yaml
        if parse:
            for e in entities:
                e.data = yaml.load(e.data)

        return entities

    def get_closest_entity(self, type="", center_point=Point(), radius=0):
        if isinstance(center_point, PointStamped):
            center_point = self._transform_center_point_to_map(center_point)

        entities = self.get_entities(type="", center_point=center_point, radius=radius)

        # HACK
        entities = [ e for e in entities if len(e.convex_hull) > 0 and e.type == "" ]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: hypot(center_point.x - entity.pose.position.x, center_point.y - entity.pose.position.y))
        except:
            print "Failed to sort entities"
            return None

        return entities[0]

    def get_closest_laser_entity(self, type="", center_point=Point(), radius=0):
        if isinstance(center_point, PointStamped):
            center_point = self._transform_center_point_to_map(center_point)

        entities = self.get_entities(type="", center_point=center_point, radius=radius)

        # HACK
        entities = [ e for e in entities if len(e.convex_hull) > 0 and e.type == "" and e.id.endswith("-laser") ]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: hypot(center_point.x - entity.pose.position.x, center_point.y - entity.pose.position.y))
        except:
            print "Failed to sort entities"
            return None

        return entities[0]

    def get_entity(self, id, parse=True):
        entities = self.get_entities(id=id, parse=parse)
        if len(entities) == 0:
            return None

        return entities[0]

    def get_entity_info(self, id):
        return self._ed_entity_info_query_srv(id=id, measurement_image_border=20)

    # ----------------------------------------------------------------------------------------------------
    #                                             UPDATING
    # ----------------------------------------------------------------------------------------------------

    def reset(self, keep_all_shapes=False):
        """Reset world model to initial state, also clears trained persons"""
        self.clear_persons()

        try:
            self._ed_reset_srv(keep_all_shapes=keep_all_shapes)
        except rospy.ServiceException, e:
            rospy.logerr("Could not reset ED: {0}".format(e))

        rospy.sleep(.2)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def update_entity(self, id, type = None, posestamped = None, flags = None, add_flags = [], remove_flags = [], action = None):
        """
        Updates entity
        :param id: entity id
        :param type: entity type
        :param posestamped: ???
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

        if posestamped:
            X, Y, Z = tf.transformations.euler_from_quaternion([posestamped.pose.orientation.x, posestamped.pose.orientation.y, posestamped.pose.orientation.z, posestamped.pose.orientation.w])
            t = posestamped.pose.position
            json_entity += ', "pose": { "x": %f, "y": %f, "z": %f, "X": %f, "Y": %f, "Z": %f }' % (t.x, t.y, t.z, X, Y, Z)

        if flags or add_flags or remove_flags:
            json_entity += ', "flags": ['
            first = True

            if isinstance(flags, dict):
                flags = [flags]

            if isinstance(flags, list):
                for flag in flags:
                    if not isinstance(flag, dict):
                        print "update_entity - Error: flags need to be a list of dicts or a dict"
                        return False
                    for k,v in flag.iteritems():
                        if not first:
                            json_entity += ','
                        json_entity += '{"%s":"%s"}' % (k,v)
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

        json = '{"entities":[{%s}]}'%json_entity
        print json

        return self._ed_update_srv(request=json)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def remove_entity(self, id):
        """ Removes entity with the provided id to the world model
        Args:
            id: string with the ID of the entity to remove
        """
        return self.update_entity(id=id, action="remove")

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def lock_entities(self, lock_ids, unlock_ids):
        for eid in lock_ids:
            self.update_entity(id=eid, add_flags=['locked'])

        for eid in unlock_ids:
            self.update_entity(id=eid, remove_flags=['locked'])

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def get_closest_possible_person_entity(self, type="", center_point=Point(), radius=0, room = ""):
        if isinstance(center_point, PointStamped):
            center_point = self._transform_center_point_to_map(center_point)

        entities = self.get_entities(type="", center_point=center_point, radius=radius)
        #print "entities 1 in get_closest_possible_person_entity = ", entities

        # HACK
        entities = [ e for e in entities if len(e.convex_hull) > 0 and e.type == "" and 'possible_human' in e.flags ]
        #print "entities 2 in get_closest_possible_person_entity = ", entities

        # if only the persons in a certain room should be found:
        # if not (room == "" and len(entities) == 0):
        #     print "room in ed = ", room
        #     room_entity = self.get_entity(id=str(room))
        #     x_max_room = room_entity.data['areas'][0]['shape'][0]['box']['max']['x']+room_entity.data['pose']['x']
        #     print "x_max_room = ", x_max_room
        #     x_min_room = room_entity.data['areas'][0]['shape'][0]['box']['min']['x']+room_entity.data['pose']['x']
        #     print "x_min_room = ", x_min_room
        #     y_max_room = room_entity.data['areas'][0]['shape'][0]['box']['max']['y']+room_entity.data['pose']['y']
        #     print "y_max_room = ", y_max_room
        #     y_min_room = room_entity.data['areas'][0]['shape'][0]['box']['min']['y']+room_entity.data['pose']['y']
        #     print "y_min_room = ", y_min_room

        #     entities = [e for e in entities if e.pose.position.x > x_min_room and e.pose.position.x < x_max_room and e.pose.position.y > y_min_room and e.pose.position.y < y_max_room]

        #     print "entities sorted in room = ", entities

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: hypot(center_point.x - entity.pose.position.x, center_point.y - entity.pose.position.y))
            print "entities sorted closest to robot = ", entities
        except:
            print "Failed to sort entities"
            return None

        return entities[0]

    # ----------------------------------------------------------------------------------------------------
    #                                  KINECT INTEGRATION AND PERCEPTION
    # ----------------------------------------------------------------------------------------------------

    def update_kinect(self, area_description="", background_padding=0):
        """
        Update ED based on kinect (depth) images

        :param area_description An entity id or area description, e.g. "a08d537e-e051-11e5-a34e-6cc217ec9f41" or "on_top_of cabinet-11"
        :param background_padding The maximum distance to which kinect data points are associated to existing objects (in meters).
               Or, in other words: the padding that is added to existing objects before they are removed from the point cloud
        :returns Update result
        """

        # Save the image (logging)
        self.save_image(path_suffix=area_description.replace(" ", "_"))

        res = self._ed_kinect_update_srv(area_description = area_description, background_padding = background_padding)
        if res.error_msg:
            rospy.logerr("Could not segment objects: %s" % res.error_msg)

        return res

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def get_perception_model_path(self, perception_model_name = ""):
        import rospkg
        rospack = rospkg.RosPack()

        try:
            import os
            robot_env = os.environ['ROBOT_ENV']
            return rospack.get_path('ed_perception_models') + "/models/" + robot_env + "/" + perception_model_name
        except KeyError:
            rospy.logerr("Wile classifying: could not get 'ROBOT_ENV' environment variable.")
            return ""

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def classify(self, ids, perception_model_name="", property="type", types=None):
        """ Classifies the entities with the given IDs. If we are simulating instead of acting on the real robot,
        a random type from the list of possible types is given.
        # ToDo: @Luis: how does this work???
        Args:
            ids: list with IDs
            perception_model_name:
            property:
            types: list with types to identify

        Returns: list with ClassificationResults, which is a named tuple with id, type and probability

        """
        perception_model_path = self.get_perception_model_path(perception_model_name)
        if not perception_model_path:
            rospy.logerr("No perception model path")
            return []

        res = self._ed_classify_srv(ids=ids, property=property, perception_models_path=perception_model_path)
        if res.error_msg:
            rospy.logerr("While classifying entities: %s" % res.error_msg)

        # if there is a set of expected types, only report the one with the highest probability
        if types:
            import os
            if os.environ.get('ROBOT_REAL', 'false') in ['true', 'True', 'TRUE']:
                # This is what we do for real
                # for idx, id, type in enumerate (res.ids):
                # print "TODO: finish type filtering in Classification"
                return [ClassificationResult(_id, exp_val, exp_prob) for _id, exp_val, exp_prob in zip(res.ids, res.expected_values, res.expected_value_probabilities) if exp_val in types]
            else:
                # This is what we do in simulation
                import random
                extypes = types + [""]
                exvalues = []
                exprobs = []
                for id in ids:
                    exvalues.append(random.choice(extypes))
                    exprobs.append(random.random())
                    self.update_entity(id=id, type=exvalues[-1])
                    print "ID: {0}: {1} (prob = {2})".format(id, exvalues[-1], exprobs[-1])
                return [ClassificationResult(_id, exp_val, exp_prob) for _id, exp_val, exp_prob in zip(ids, exvalues, exprobs) if exp_val in types]
        else:
            return [ClassificationResult(_id, exp_val, exp_prob) for _id, exp_val, exp_prob in zip(res.ids, res.expected_values, res.expected_value_probabilities)]

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def classify_with_probs(self, ids, types):
        rospy.logwarn("Is classify_with_probs function deprecated?")  # ToDo: @Luis: is this true?
        res = self._ed_classify_srv(ids = ids, types = types)
        return zip(res.types, res.probabilities)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def add_perception_training_instance(self, id, property, value, perception_model_name = ""):
        perception_model_path = self.get_perception_model_path(perception_model_name)
        if not perception_model_path:
            return False

        res = self._ed_perception_add_training_instance_srv(id = id, property = property, value = value, perception_models_path = perception_model_path)
        if res.error_msg:
            rospy.logerr("While adding perception training instance: %s" % res.error_msg)
            return False

        return True

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def clear_persons(self):
        try:
            res = self._clear_persons_srv()
        except rospy.ServiceException, e:
            rospy.logerr("Could not Reset Persons ED: {0}".format(e))
        return True

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def learn_person(self, name):
        try:
            res = self._learn_person_srv(person_name=name)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        if res.error_msg:
            rospy.logerr("Learn person failed: %s" % res.error_msg)
            return False
        return True

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def detect_persons(self, external_api_request=False):
        try:
            res = self._recognize_person_srv(external_api_request=external_api_request)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        if res.error_msg:
            rospy.logerr("Detect persons failed: %s" % res.error_msg)
            return None
        else:
            return res.person_detections

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def save_image(self, path = "", path_suffix = "", filename = ""):
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
        os.system('rosrun rgbd rgbd_to_rgb_png %s' % (fname + ".rgbd"))

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def mesh_entity_in_view(self, id, type=""):
        # Takes the biggest one in view
        return self._ed_mesh_entity_in_view_srv(id=id, type=type)

    # ----------------------------------------------------------------------------------------------------
    #                                                MISC
    # ----------------------------------------------------------------------------------------------------

    def get_full_id(self, short_id):
        """Get an entity's full ID based on the first characters of its ID like you can do with git hashes"""
        all_entities = self.get_entities(parse=False)
        matches = filter(lambda fill_id: fill_id.startswith(short_id), [entity.id for entity in all_entities])
        return matches

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def _transform_center_point_to_map(self, pointstamped):
        point_in_map = transformations.tf_transform(pointstamped.point, pointstamped.header.frame_id, "/map", self._tf_listener)
        return point_in_map

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def _publish_marker(self, center_point, radius):
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.type = 2
        marker.pose.position.x = center_point.x
        marker.pose.position.y = center_point.y
        marker.pose.position.z = center_point.z
        marker.lifetime = rospy.Duration(20.0)
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius

        marker.color.a = 0.5
        marker.color.r = 1

        self._marker_publisher.publish(marker)

    # ----------------------------------------------------------------------------------------------------
    #                                               OBSOLETE
    # ----------------------------------------------------------------------------------------------------

    def configure_kinect_segmentation(self, continuous=None, max_sensor_range=0):
        raise NotImplementedError("Method 'configure_kinect_segmentation' has become obsolete - don't use it")


    def configure_perception(self, continuous):
        raise NotImplementedError("Method 'configure_perception' has become obsolete - don't use it")
