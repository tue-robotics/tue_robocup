#! /usr/bin/env python
import rospy
from ed.srv import SimpleQuery, SimpleQueryRequest, UpdateSrv, Configure
# from ed_sensor_integration.srv import LockEntities, MeshEntityInView, Segment
import ed_sensor_integration.srv
import ed_perception.srv
from ed_perception.srv import Classify
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

import PyKDL as kdl
from robot_skills.util.kdl_conversions import poseMsgToKdlFrame, pointMsgToKdlVector, VectorStamped, kdlVectorToPointMsg
import os


import yaml

from .classification_result import ClassificationResult

from robot_skills.util.entity import from_entity_info

def _create_service_proxy(service_name, service_type, wait_service):
    if wait_service:
        rospy.wait_for_service(service_name)
    return rospy.ServiceProxy(service_name, service_type)


class Navigation:
    def __init__(self, robot_name, tf_listener, wait_service=False):
        self._get_constraint_srv = _create_service_proxy('/%s/ed/navigation/get_constraint'%robot_name, GetGoalConstraint, wait_service)

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
        self._ed_simple_query_srv = _create_service_proxy('/%s/ed/simple_query'%robot_name, SimpleQuery, wait_service)
        self._ed_entity_info_query_srv = _create_service_proxy('/%s/ed/gui/get_entity_info'%robot_name, GetEntityInfo, wait_service)
        self._ed_update_srv = _create_service_proxy('/%s/ed/update'%robot_name, UpdateSrv, wait_service)
        self._ed_kinect_update_srv = _create_service_proxy('/%s/ed/kinect/update'%robot_name, ed_sensor_integration.srv.Update, wait_service)
        self._ed_classify_srv = _create_service_proxy('/%s/ed/classify'%robot_name, Classify, wait_service)
        self._ed_configure_srv = _create_service_proxy('/%s/ed/configure'%robot_name, Configure, wait_service)
        self._ed_reset_srv = _create_service_proxy('/%s/ed/reset'%robot_name, ed.srv.Reset, wait_service)
        self._ed_get_image_srv = _create_service_proxy('/%s/ed/kinect/get_image'%robot_name, ed_sensor_integration.srv.GetImage, wait_service)

        self._tf_listener = tf_listener

        self.navigation = Navigation(robot_name, tf_listener, wait_service)

        self._marker_publisher = rospy.Publisher("/" + robot_name + "/ed/simple_query",  visualization_msgs.msg.Marker, queue_size=10)

    # ----------------------------------------------------------------------------------------------------
    #                                             QUERYING
    # ----------------------------------------------------------------------------------------------------

    def get_entities(self, type="", center_point=VectorStamped(), radius=0, id="", parse=True):
        self._publish_marker(center_point, radius)

        center_point_in_map = center_point.projectToFrame("/map", self._tf_listener)
        query = SimpleQueryRequest(id=id, type=type, center_point=kdlVectorToPointMsg(center_point_in_map.vector), radius=radius)

        try:
            entity_infos= self._ed_simple_query_srv(query).entities
            entities = map(from_entity_info, entity_infos)
        except Exception, e:
            rospy.logerr("ERROR: robot.ed.get_entities(id=%s, type=%s, center_point=%s, radius=%s)" % (id, type, str(center_point), str(radius)))
            rospy.logerr("L____> [%s]" % e)
            return []

        return entities

    def get_closest_entity(self, type="", center_point=kdl.Vector(), radius=0):
        if isinstance(center_point, PointStamped):
            center_point = self._transform_center_point_to_map(center_point)

        entities = self.get_entities(type="", center_point=center_point, radius=radius)

        # HACK
        entities = [ e for e in entities if e.convex_hull and e.type == "" ]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(center_point))
        except:
            print "Failed to sort entities"
            return None

        return entities[0]

    def get_closest_laser_entity(self, type="", center_point=kdl.Vector(), radius=0):
        if isinstance(center_point, PointStamped):
            center_point = self._transform_center_point_to_map(center_point)

        entities = self.get_entities(type="", center_point=center_point, radius=radius)

        # HACK
        entities = [ e for e in entities if e.convex_hull and e.type == "" and e.id.endswith("-laser") ]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(center_point))
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

    def reset(self, keep_all_shapes=True):
        try:
            self._ed_reset_srv(keep_all_shapes=keep_all_shapes)
        except rospy.ServiceException, e:
            rospy.logerr("Could not reset ED: {0}".format(e))

        rospy.sleep(.2)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def update_entity(self, id, type=None, frame_stamped=None, flags=None, add_flags=[], remove_flags=[], action=None):
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

        if frame_stamped:
            if frame_stamped.frame_id != "/map":
                frame_stamped = frame_stamped.projectToFrame("/map", self._tf_listener)

            Z, Y, X = frame_stamped.frame.M.GetEulerZYX()
            t = frame_stamped.frame.p
            json_entity += ', "pose": { "x": %f, "y": %f, "z": %f, "X": %f, "Y": %f, "Z": %f }' % (t.x(), t.y(), t.z(), X, Y, Z)

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

        # HACK
        entities = [ e for e in entities if e.convex_hull and e.type == "" and 'possible_human' in e.flags ]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(center_point))
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
        # Check the area description
        if area_description == "":
            rospy.logwarn("No area_description provided for 'update_kinect'. This is probably a bad idea.")

        # Save the image (logging)
        self.save_image(path_suffix=area_description.replace(" ", "_"))

        res = self._ed_kinect_update_srv(area_description = area_description, background_padding = background_padding)
        if res.error_msg:
            rospy.logerr("Could not segment objects: %s" % res.error_msg)

        return res

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def classify(self, ids, types=None):
        """ Classifies the entities with the given IDs
        Args:
            ids: list with IDs
            types: list with types to identify

        Returns: list with ClassificationResults, which is a named tuple with id, type and probability

        """

        res = self._ed_classify_srv(ids=ids)
        if res.error_msg:
            rospy.logerr("While classifying entities: %s" % res.error_msg)


        posteriors = [dict(zip(distr.values, distr.probabilities)) for distr in res.posteriors]

        # Filter on types if types is not None
       	return [ClassificationResult(_id, exp_val, exp_prob, distr) for _id, exp_val, exp_prob, distr
       				in zip(res.ids, res.expected_values, res.expected_value_probabilities, posteriors) if types is None or exp_val in types]

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
