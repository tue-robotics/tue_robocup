#! /usr/bin/env python
import rospy
from ed.srv import SimpleQuery, SimpleQueryRequest, UpdateSrv, Configure
from ed.srv import GetGUICommand, GetGUICommandResponse
# from ed_sensor_integration.srv import LockEntities, MeshEntityInView, Segment
import ed_sensor_integration.srv
from ed_perception.srv import Classify
from ed_gui_server.srv import GetEntityInfo
from ed_navigation.srv import GetGoalConstraint
from cb_planner_msgs_srvs.msg import PositionConstraint
from geometry_msgs.msg import Point, PointStamped
import robot_skills.util.msg_constructors as msgs
from math import hypot

from robot_skills.util import transformations

from std_srvs.srv import Empty #Reset Ed

import tf
import visualization_msgs.msg


import yaml


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
        self._ed_configure_srv = rospy.ServiceProxy('/%s/ed/configure'%robot_name, Configure)

        self._ed_reset_srv = rospy.ServiceProxy('/%s/ed/reset'%robot_name, Empty)

        self._ed_get_image_srv = rospy.ServiceProxy('/%s/ed/kinect/get_image'%robot_name, ed_sensor_integration.srv.GetImage)

        self._tf_listener = tf_listener

        self.navigation = Navigation(robot_name, tf_listener, wait_service)

        self._marker_publisher = rospy.Publisher("/" + robot_name + "/ed/simple_query",  visualization_msgs.msg.Marker, queue_size=10)

    def get_entities(self, type="", center_point=Point(), radius=0, id="", parse=True):
        if isinstance(center_point, PointStamped):
            center_point = self._transform_center_point_to_map(center_point)

        self._publish_marker(center_point, radius)
        query = SimpleQueryRequest(id=id, type=type, center_point=center_point, radius=radius)

        try:
            entities = self._ed_simple_query_srv(query).entities
        except Exception, e:
            rospy.logerr(e)
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

    def get_entity(self, id, parse=True):
        entities = self.get_entities(id=id, parse=parse)
        if len(entities) == 0:
            return None

        return entities[0]

    def get_entity_info(self, id):
        return self._ed_entity_info_query_srv(id=id, measurement_image_border=20)

    def reset(self):
        try:
            self._ed_reset_srv()
        except rospy.ServiceException, e:
            rospy.logerr("Could not reset ED: {0}".format(e))
        rospy.sleep(1.0)

    def _transform_center_point_to_map(self, pointstamped):
        point_in_map = transformations.tf_transform(pointstamped.point, pointstamped.header.frame_id, "/map", self._tf_listener)
        return point_in_map

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

    def update_entity(self, id, type = None, posestamped = None, flags = None, add_flags = [], remove_flags = []):
        """
        Updates entity
        :param id: entity id
        :param type: entity type
        :param posestamped: ???
        :param flags: (list of) dict(s) containing key 'add' or 'remove' and value of the flag to set,
        e.g., 'perception'
        """
        json_entity = '"id" : "%s"' % id
        if type:
            json_entity += ', "type": "%s"' % type

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
        if (res.error_msg):
            rospy.logerr("Could not save image: %s" % res.error_msg)

        with open(fname + ".rgbd", "wb") as f:
            f.write(bytearray(res.rgbd_data))

        with open(fname + ".json", "w") as f:
            f.write(res.json_meta_data)

    # If continuous is None, the continuous mode of segmentation is left unchanged (on if it was on, off if it was off). However,
    # if 'continuous' is NOT None, it actively sets the mode of segmentation (True = on, False = off)
    def configure_kinect_segmentation(self, continuous=None, max_sensor_range=0):
        res = self._ed_segment_srv(enable_continuous_mode = (continuous == True), disable_continuous_mode = (continuous == False), max_sensor_range = max_sensor_range)
        return res.entity_ids

    def update_kinect(self, area_description):
        # Save the image (logging)
        self.save_image(path_suffix=area_description.replace(" ", "_"))

        res = self._ed_kinect_update_srv(update_space_description = area_description)
        if res.error_msg:
            rospy.logerr("Could not segment objects: %s" % res.error_msg)

        return res

    #def configure_perception(self, continuous):
    #    self._ed_classify_srv(enable_continuous_mode = continuous, disable_continuous_mode = (not continuous))

    def classify(self, ids, perception_model_name = "", property = "type"):
        import rospkg
        rospack = rospkg.RosPack()

        try:
            import os
            robot_env = os.environ['ROBOT_ENV']
            path = rospack.get_path('ed_perception_models') + "/models/" + robot_env + "/" + perception_model_name
        except KeyError:
            rospy.logerr("Wile classifying: could not get 'ROBOT_ENV' environment variable.")
            return []

        res = self._ed_classify_srv(ids = ids, property = property, perception_models_path=path)
        if (res.error_msg):
            rospy.logerr("While classifying entities: %s" % res.error_msg)

        return zip(res.ids, res.expected_values, res.expected_value_probabilities)

    def classify_with_probs(self, ids, types):
        res = self._ed_classify_srv(ids = ids, types = types)
        return zip(res.types, res.probabilities)

    def enable_plugins(self, plugin_names):
        return self._set_plugin_status(plugin_names, '"enabled":1')

    def disable_plugins(self, plugin_names):
        return self._set_plugin_status(plugin_names, '"enabled":0')

    def lock_entities(self, lock_ids, unlock_ids):
        for eid in lock_ids:
            self.update_entity(id=eid, add_flags=['locked'])

        for eid in unlock_ids:
            self.update_entity(id=eid, remove_flags=['locked'])

    def mesh_entity_in_view(self, id, type=""):
        # Takes the biggest one in view
        return self._ed_mesh_entity_in_view_srv(id=id, type=type)

    def get_full_id(self, short_id):
        """Get an entity's full ID based on the first characters of its ID like you can do with git hashes"""
        all_entities = self.get_entities(parse=False)
        matches = filter(lambda fill_id: fill_id.startswith(short_id), [entity.id for entity in all_entities])
        return matches

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


