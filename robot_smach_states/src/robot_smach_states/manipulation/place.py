# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills.arms import PublicArm
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import kdl_frame_stamped_from_XYZRPY, FrameStamped
from robot_smach_states.navigation import NavigateToPlace
from robot_smach_states.world_model import Inspect
from robot_smach_states.util.designators.ed_designators import Designator
from robot_smach_states.utility import LockDesignator
from robot_smach_states.util.designators.utility import LockingDesignator
from robot_smach_states.util.designators import check_type

from visualization_msgs.msg import MarkerArray, Marker
from cb_planner_msgs_srvs.msg import PositionConstraint


class EmptySpotDesignator(Designator):
    """Designates an empty spot on the empty placement-shelve.
    It does this by querying ED for entities that occupy some space.
        If the result is no entities, then we found an open spot.

    To test this in the robotics_test_lab with amigo-console:
    robot = amigo
    CABINET = "bookcase"
    PLACE_SHELF = "shelf2"
    cabinet = ds.EntityByIdDesignator(robot, id=CABINET, name="pick_shelf")
    arm = ds.UnoccupiedArmDesignator(robot, {})
    place_position = ds.LockingDesignator(ds.EmptySpotDesignator(robot, cabinet, arm, name="placement", area=PLACE_SHELF),
                                          name="place_position")
    """

    def __init__(self, robot, place_location_designator, arm_designator, name=None, area=None):
        """
        Designate an empty spot (as PoseStamped) on some designated entity
        :param robot: Robot whose worldmodel to use
        :param place_location_designator: Designator resolving to an Entity, e.g. EntityByIdDesignator
        :param arm_designator: Designator resolving to an arm robot part, e.g. OccupiedArmDesignator
        :param name: name for introspection purposes
        :param area: (optional) area where the item should be placed
        """
        super(EmptySpotDesignator, self).__init__(resolve_type=FrameStamped, name=name)
        self.robot = robot

        self.place_location_designator = place_location_designator
        self.arm_designator = arm_designator
        self._edge_distance = 0.05  # Distance to table edge
        self._spacing = 0.15
        self._area = area

        self.marker_pub = rospy.Publisher('/empty_spots', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()

    def _resolve(self):
        """
        :return: Where can an object be placed
        :returns: FrameStamped
        """
        place_location = self.place_location_designator.resolve()
        place_frame = FrameStamped(frame=place_location._pose, frame_id="/map")

        # points_of_interest = []
        if self._area:
            vectors_of_interest = self.determine_points_of_interest_with_area(place_location, self._area)
        else:
            vectors_of_interest = self.determine_points_of_interest(place_frame.frame, z_max=place_location.shape.z_max,
                                                                    convex_hull=place_location.shape.convex_hull)

        assert all(isinstance(v, FrameStamped) for v in vectors_of_interest)

        open_POIs = filter(self.is_poi_occupied, vectors_of_interest)

        base_pose = self.robot.base.get_location()
        open_POIs_dist = [(poi, self.distance_to_poi_area_heuristic(poi, base_pose, self.arm_designator)) for poi in open_POIs]

        # We don't care about small differences
        nav_threshold = 0.5 / 0.05  # Distance (0.5 m) divided by resolution (0.05)
        open_POIs_dist = [f for f in open_POIs_dist if (f[1] - open_POIs_dist[0][1]) < nav_threshold]

        open_POIs_dist.sort(key=lambda tup: tup[1]) # sorts in place

        for poi in open_POIs_dist:
            if self.distance_to_poi_area(poi[0]):
                selection = self.create_selection_marker(poi[0])
                self.marker_pub.publish(MarkerArray([selection]))
                return poi[0]

        rospy.logerr("Could not find an empty spot")
        return None

    def is_poi_occupied(self, frame_stamped):
        entities_at_poi = self.robot.ed.get_entities(center_point=frame_stamped.extractVectorStamped(),
                                                     radius=self._spacing)
        return not any(entities_at_poi)

    def distance_to_poi_area_heuristic(self, frame_stamped, base_pose, arm_designator):
        arm = arm_designator.resolve()
        bo = arm.base_offset

        offset_pose = base_pose.frame * bo
        offset_pose_x = offset_pose[0]
        offset_pose_y = offset_pose[1]

        x = frame_stamped.frame.p.x()
        y = frame_stamped.frame.p.y()

        dist = math.hypot(offset_pose_x - x, offset_pose_y - y)
        return dist

    def distance_to_poi_area(self, frame_stamped):

        # ToDo: cook up something better: we need the arm that we're currently using but this would require a
        # rather large API break (issue #739)
        base_offset = self.robot.arms.values()[0].base_offset
        radius = math.hypot(base_offset.x(), base_offset.y())

        x = frame_stamped.frame.p.x()
        y = frame_stamped.frame.p.y()
        radius -= 0.1
        ro = "(x-%f)^2+(y-%f)^2 < %f^2" % (x, y, radius + 0.075)
        ri = "(x-%f)^2+(y-%f)^2 > %f^2" % (x, y, radius - 0.075)
        pos_constraint = PositionConstraint(constraint=ri + " and " + ro, frame=frame_stamped.frame_id)

        plan_to_poi = self.robot.base.global_planner.getPlan(pos_constraint)

        if plan_to_poi:
            distance = len(plan_to_poi)
            # print "Distance to {fs}: {dist}".format(dist=distance, fs=frame_stamped.frame.p)
        else:
            distance = None
        return distance

    def create_marker(self, x, y, z):
        marker = Marker()
        marker.id = len(self.marker_array.markers)
        marker.type = 2
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1
        marker.color.a = 1

        marker.lifetime = rospy.Duration(10.0)
        return marker

    def create_selection_marker(self, selected_pose):
        marker = Marker()
        marker.id = len(self.marker_array.markers) + 1
        marker.type = 2
        marker.header.frame_id = selected_pose.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = selected_pose.frame.p.x()
        marker.pose.position.y = selected_pose.frame.p.y()
        marker.pose.position.z = selected_pose.frame.p.z()
        marker.pose.orientation.w = 1
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.g = 1
        marker.color.a = 0.7

        marker.lifetime = rospy.Duration(30.0)
        return marker

    def determine_points_of_interest_with_area(self, entity, area):
        """ Determines the points of interest using an area
        :type entity: Entity
        :param area: str indicating which volume of the entity to look at
        :rtype: [FrameStamped]
        """

        # We want to give it a convex hull using the designated area

        if area not in entity.volumes:
            return []

        box = entity.volumes[area]

        if not hasattr(box, "bottom_area"):
            rospy.logerr("Entity {0} has no shape with a bottom_area".format(entity.id))

        # Now we're sure to have the correct bounding box
        # Make sure we offset the bottom of the box
        top_z = box.min_corner.z() - 0.04  # 0.04 is the usual offset
        return self.determine_points_of_interest(entity._pose, top_z, box.bottom_area)

    def determine_points_of_interest(self, center_frame, z_max, convex_hull):
        """
        Determine candidates for place poses
        :param center_frame: kdl.Frame, center of the Entity to place on top of
        :param z_max: float, height of the entity to place on, w.r.t. the entity
        :param convex_hull: [kdl.Vector], convex hull of the entity
        :return: [FrameStamped] of candidates for placing
        """

        points = []

        if len(convex_hull) == 0:
            rospy.logerr('determine_points_of_interest: Empty convex hull')
            return []

        # Convert convex hull to map frame
        ch = offsetConvexHull(convex_hull, center_frame)

        # Loop over hulls
        self.marker_array.markers = []

        for i in xrange(len(ch)):
            j = (i + 1) % len(ch)

            dx = ch[j].x() - ch[i].x()
            dy = ch[j].y() - ch[i].y()

            length = kdl.diff(ch[j], ch[i]).Norm()

            d = self._edge_distance
            while d < (length - self._edge_distance):
                # Point on edge
                xs = ch[i].x() + d / length * dx
                ys = ch[i].y() + d / length * dy

                # Shift point inwards and fill message
                fs = kdl_frame_stamped_from_XYZRPY(x=xs - dy / length * self._edge_distance,
                                                   y=ys + dx / length * self._edge_distance,
                                                   z=center_frame.p.z() + z_max,
                                                   frame_id="/map")

                # It's nice to put an object on the middle of a long edge. In case of a cabinet, e.g., this might
                # prevent the robot from hitting the cabinet edges
                # print "Length: {}, edge score: {}".format(length, min(d, length-d))
                setattr(fs, 'edge_score', min(d, length-d))

                points += [fs]

                self.marker_array.markers.append(self.create_marker(fs.frame.p.x(), fs.frame.p.y(), fs.frame.p.z()))

                # ToDo: check if still within hull???
                d += self._spacing

        self.marker_pub.publish(self.marker_array)

        return points

    def __repr__(self):
        return "EmptySpotDesignator(place_location_designator={}, name='{}', area='{}')"\
                    .format(self.place_location_designator, self._name, self._area)


class PreparePlace(smach.State):
    def __init__(self, robot, arm):
        """
        Drive the robot back a little and move the designated arm to place the designated item at the designated pose
        :param robot: Robot to execute state with
        :param arm: Designator -> arm to place with, so Arm that holds entity_to_place, e.g. via
        ArmHoldingEntityDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Check types or designator resolve types
        check_type(arm, PublicArm)

        # Assign member variables
        self._robot = robot
        self._arm_designator = arm

    def execute(self, userdata=None):

        arm = self._arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        # Torso up (non-blocking)
        self._robot.torso.reset()

        # Arm to position in a safe way
        arm.send_joint_trajectory('prepare_place', timeout=0)
        arm.wait_for_motion_done()

        # When the arm is in the prepare_place configuration, the grippoint is approximately at height torso_pos + 0.6
        # Hence, we want the torso to go to the place height - 0.6
        # Note: this is awefully hardcoded for AMIGO
        # Sending it to 'high' seems to work much better...
        # torso_goal = placement_fs.frame.p.z() - 0.6
        # torso_goal = max(0.09, min(0.4, torso_goal))
        # rospy.logwarn("Torso goal before placing: {0}".format(torso_goal))
        # self._robot.torso._send_goal(torso_pos=[torso_goal])

        return 'succeeded'

# ----------------------------------------------------------------------------------------------------


class Put(smach.State):

    def __init__(self, robot, item_to_place, placement_pose, arm):
        """
        Drive the robot back a little and move the designated arm to place the designated item at the designated pose
        :param robot: Robot to execute state with
        :param item_to_place: Designator that resolves to the entity to place. e.g EntityByIdDesignator
        :param placement_pose: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        :param arm: Designator -> arm to place with, so Arm that holds entity_to_place, e.g. via
        ArmHoldingEntityDesignator
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Check types or designator resolve types
        check_type(item_to_place, Entity)
        check_type(placement_pose, FrameStamped)
        check_type(arm, PublicArm)

        # Assign member variables
        self._robot = robot
        self._item_to_place_designator = item_to_place
        self._placement_pose_designator = placement_pose
        self._arm_designator = arm

    def execute(self, userdata=None):

        item_to_place = self._item_to_place_designator.resolve()
        if not item_to_place:
            rospy.logerr("Could not resolve item_to_place")
            # return "failed"

        placement_fs = self._placement_pose_designator.resolve()
        if not placement_fs:
            rospy.logerr("Could not resolve placement_pose")
            return "failed"

        arm = self._arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        rospy.loginfo("Placing")

        # placement_pose is a PyKDL.Frame
        place_pose_bl = placement_fs.projectToFrame(self._robot.robot_name+'/base_link',
                                                    tf_listener=self._robot.tf_listener)

        # Wait for torso and arm to finish their motions
        self._robot.torso.wait_for_motion_done()
        arm.wait_for_motion_done()

        try:
            height = place_pose_bl.frame.p.z()
        except KeyError:
            height = 0.8

        # Pre place
        if not arm.send_goal(kdl_frame_stamped_from_XYZRPY(place_pose_bl.frame.p.x(),
                                                           place_pose_bl.frame.p.y(),
                                                           height+0.15, 0.0, 0.0, 0.0,
                                                           frame_id="/{0}/base_link".format(self._robot.robot_name)),
                             timeout=10,
                             pre_grasp=True):
            # If we can't place, try a little closer
            place_pose_bl.frame.p.x(place_pose_bl.frame.p.x() - 0.025)

            rospy.loginfo("Retrying preplace")
            if not arm.send_goal(kdl_frame_stamped_from_XYZRPY(place_pose_bl.frame.p.x(),
                                                               place_pose_bl.frame.p.y(),
                                                               height+0.15, 0.0, 0.0, 0.0,
                                                               frame_id="/{0}/base_link".format(self._robot.robot_name)
                                                               ), timeout=10, pre_grasp=True):
                rospy.logwarn("Cannot pre-place the object")
                arm.cancel_goals()
                return 'failed'

        # Place
        place_pose_bl = placement_fs.projectToFrame(self._robot.robot_name+'/base_link',
                                                    tf_listener=self._robot.tf_listener)
        if not arm.send_goal(kdl_frame_stamped_from_XYZRPY(place_pose_bl.frame.p.x(),
                                                           place_pose_bl.frame.p.y(),
                                                           height+0.1, 0.0, 0.0, 0.0,
                                                           frame_id="/{0}/base_link".format(self._robot.robot_name)),
                             timeout=10, pre_grasp=False):
            rospy.logwarn("Cannot place the object, dropping it...")

        place_entity = arm.occupied_by
        if not place_entity:
            rospy.logerr("Arm not holding an entity to place. This should never happen")
        else:
            self._robot.ed.update_entity(place_entity.id, frame_stamped=placement_fs)
            arm.occupied_by = None

        # Open gripper
        # Since we cannot reliably wait for the gripper, just set this timeout
        arm.send_gripper_goal('open', timeout=2.0)

        arm.occupied_by = None

        # Retract
        place_pose_bl = placement_fs.projectToFrame(self._robot.robot_name+'/base_link',
                                                    tf_listener=self._robot.tf_listener)
        arm.send_goal(kdl_frame_stamped_from_XYZRPY(place_pose_bl.frame.p.x() - 0.1,
                                                    place_pose_bl.frame.p.y(),
                                                    place_pose_bl.frame.p.z() + 0.15, 0.0, 0.0, 0.0,
                                                    frame_id='/'+self._robot.robot_name+'/base_link'),
                      timeout=0.0)

        arm.wait_for_motion_done()
        self._robot.base.force_drive(-0.125, 0, 0, 1.5)

        if not arm.wait_for_motion_done(timeout=5.0):
            rospy.logwarn('Retraction failed')
            arm.cancel_goals()

        # Close gripper
        arm.send_gripper_goal('close', timeout=0.0)

        arm.reset()
        arm.wait_for_motion_done()
        self._robot.torso.reset()
        self._robot.torso.wait_for_motion_done()

        return 'succeeded'


class Place(smach.StateMachine):

    def __init__(self, robot, item_to_place, place_pose, arm, place_volume=None, update_supporting_entity=False):
        """
        Drive the robot to be close to the designated place_pose and move the designated arm to place the designated
        item there
        :param robot: Robot to execute state with
        :param item_to_place: Designator that resolves to the entity to place. e.g EntityByIdDesignator
        :param place_pose: The place pose can be one of three things:
         1: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
         2: EdEntityDesignator resolving to an object on which the robot should place something
         3: A string identifying an object on which the robot should place something
        :param arm: Designator -> arm to place with, so Arm that holds entity_to_place, e.g. via
        ArmHoldingEntityDesignator
        :param place_volume (optional) string identifying the volume where to place the object, e.g., 'on_top_of',
        'shelf3'
        :param update_supporting_entity (optional) bool to indicate whether the supporting entity should be updated.
        This can only be used if the supporting entity is supplied, case 2 or 3 mentioned under item_to_place
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        # Check types or designator resolve types
        assert(item_to_place.resolve_type == Entity or type(item_to_place) == Entity)
        assert(arm.resolve_type == PublicArm or type(arm) == PublicArm)
        #assert(place_volume.resolve_type == str or (type(place_volume) == str))

        # parse place volume
        if place_volume is not None:
            if isinstance(place_volume, str):
                place_area = place_volume
            elif place_volume.resolve_type == str:
                place_area = place_volume.resolve()
            else:
                raise AssertionError("Cannot place in {}".format(place_volume))

        # Case 3
        if isinstance(place_pose, str):
            furniture_designator = EdEntityDesignator(robot=robot, id=place_pose)
            place_designator = EmptySpotDesignator(robot=robot, place_location_designator=furniture_designator,
                                                   arm_designator=arm, area=place_area)
        # Case 1
        elif place_pose.resolve_type == FrameStamped or type(place_pose) == FrameStamped:
            furniture_designator = None
            place_designator = place_pose
        # Case 2
        elif place_pose.resolve_type == Entity:
            furniture_designator = place_pose
            place_designator = EmptySpotDesignator(robot=robot, place_location_designator=furniture_designator,
                                                   arm_designator=arm, area=place_area)
        else:
            raise AssertionError("Cannot place on {}".format(place_pose))

        locking_place_designator = LockingDesignator(place_designator)

        with self:

            if furniture_designator is not None:
                smach.StateMachine.add('INSPECT',
                                       Inspect(robot, furniture_designator, navigation_area="in_front_of"),
                                       transitions={'done': 'PREPARE_PLACE',
                                                    'failed': 'failed'}
                                       )

            smach.StateMachine.add('PREPARE_PLACE', PreparePlace(robot, arm),
                                   transitions={'succeeded': 'LOCK_DESIGNATOR',
                                                'failed': 'failed'})

            smach.StateMachine.add('LOCK_DESIGNATOR', LockDesignator(locking_place_designator),
                                   transitions={'locked': 'NAVIGATE_TO_PLACE'})

            smach.StateMachine.add('NAVIGATE_TO_PLACE', NavigateToPlace(robot, locking_place_designator, arm),
                                   transitions={'unreachable': 'failed',
                                                'goal_not_defined': 'failed',
                                                'arrived': 'PUT'})

            smach.StateMachine.add('PUT', Put(robot, item_to_place, locking_place_designator, arm),
                                   transitions={'succeeded': 'done',
                                                'failed': 'failed'})


if __name__ == "__main__":

    from robot_skills import get_robot_from_argv
    from robot_smach_states.util.designators import EdEntityDesignator, ArmDesignator

    rospy.init_node('state_machine')

    robot = get_robot_from_argv(index=1)

    robot.ed.update_entity(id="bla")
    place_entity = EdEntityDesignator(robot, id="bla")
    arm = ArmDesignator(robot, {})

    sm = Place(robot=robot, item_to_place=place_entity, place_pose='dinner_table', arm=arm, place_volume='on_top_of')
    print(sm.execute())
