#! /usr/bin/env python

#
import math

# ROS
import rospy
import smach
from geometry_msgs import msg as gm

# Tech United Eindhoven
from ed.msg import EntityInfo

# Executive stuff
import robot_smach_states.util.designators as ds
from robot_smach_states.navigation import NavigateToObserve, NavigateToSymbolic
from robot_smach_states.util import geometry_helpers


class PersonDesignator(ds.Designator):
    """ Designator that can be used to drive to a person near a furniture object
    """
    def __init__(self, robot, room_designator):
        """ Constructor
        :param robot: robot object
        :param room_designator: EdEntityDesignator for the room in which to look for a person
        :return:
        """
        super(PersonDesignator, self).__init__(resolve_type=EntityInfo)
        self._robot = robot
        self._room_designator = room_designator

    def _resolve(self):

        room_entity = self._room_designator.resolve()
        if not room_entity:
            rospy.logwarn('Cannot find room entity')
            return None

        # Get the bounding box of the room
        convex_hull = []
        for testarea in room_entity.data['areas']:
            ''' See if the area is in the list of inspection areas '''
            if testarea['name'] == 'in':
                ''' Check if we have a shape '''
                if 'shape' not in testarea:
                    rospy.logwarn("No shape in area {0}".format(testarea['name']))
                    continue
                ''' Check if length of shape equals one '''
                if not len(testarea['shape']) == 1:
                    rospy.logwarn("Shape of area {0} contains multiple entries, don't know what to do".format(testarea['name']))
                    continue
                ''' Check if the first entry is a box '''
                if not 'box' in testarea['shape'][0]:
                    rospy.logwarn("No box in {0}".format(testarea['name']))
                    continue
                box = testarea['shape'][0]['box']
                if 'min' not in box or 'max' not in box:
                    rospy.logwarn("Box in {0} either does not contain min or max".format(testarea['name']))
                    continue
                # Now we're sure to have the correct bounding box
                convex_hull.append(gm.Point(box['min']['x'], box['min']['y'], box['min']['z']))  # 1
                convex_hull.append(gm.Point(box['max']['x'], box['min']['y'], box['min']['z']))  # 2
                convex_hull.append(gm.Point(box['max']['x'], box['max']['y'], box['min']['z']))  # 3
                convex_hull.append(gm.Point(box['min']['x'], box['max']['y'], box['min']['z']))  # 4

        # In principle (i.e., in 2016), we don't need to enable/disable laser_integration: it is enabled by default
        # #self._robot.ed.enable_plugins(plugin_names=["laser_integration"])

        possible_humans = self._robot.get_entities(type="possible_human")
        if not possible_humans:
            rospy.logwarn("No possible humans found")
            return None

        # Check which entities are in the room
        persons_in_room = []
        for ph in possible_humans:
            if geometry_helpers.isPointInsideHull(ph.pose.point, convex_hull):
                persons_in_room.append(ph)
        if not persons_in_room:
            rospy.logwarn("None of the found possible humans was in the room")

        # Sort according to distance to center pose
        persons_in_room = sorted(persons_in_room,
                                 key=lambda ph: math.hypot(ph.pose.point.x - room_entity.pose.point.x,
                                                           ph.pose.point.y - room_entity.pose.point.y))

        # Return the best one
        return persons_in_room[0]


class FindPerson(smach.StateMachine):
    """ Class to find ANY person within a certain room. It does NOT look for a particular person by trying to recognize
    people. To find anyone, a laser rangefinder will be used """
    def __init__(self, robot, room_designator):
        """ Constructor
        :param robot: robot object
        :param room_designator: EdEntityDesignator for the room in which to look for a person
        :return:
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        person_designator = PersonDesignator(robot=robot,
                                             room_designator=room_designator)

        with self:
            smach.StateMachine.add("NAVIGATE_TO_ROOM",
                                   NavigateToSymbolic(robot, {room_designator:"in"}, room_designator),
                                   transitions={'arrived': 'succeeded',
                                                'goal_not_defined': 'failed',
                                                'unreachable': 'failed'})

            smach.StateMachine.add("NAVIGATE_TO_PERSON",
                                   NavigateToObserve(robot=robot, entity_designator=person_designator, radius=0.7),
                                   transitions={'arrived': 'succeeded',
                                                'goal_not_defined': 'failed',
                                                'unreachable': 'failed'})



    #
    #     # Check types or designator resolve types
    #     check_type(item, ed.msg.EntityInfo)
    #     check_type(arm, Arm)
    #
    #     with self:
    #         smach.StateMachine.add('PREPARE_GRASP', PrepareEdGrasp(robot, arm, item),
    #                                transitions={ 'succeeded'    : 'NAVIGATE_TO_GRAB',
    #                                              'failed'       : 'failed'})
    #
    #         smach.StateMachine.add('NAVIGATE_TO_GRAB', NavigateToGrasp(robot, item, arm),
    #                                transitions={'unreachable':      'failed',
    #                                             'goal_not_defined': 'failed',
    #                                             'arrived':          'GRAB'})
    #
    #         smach.StateMachine.add('GRAB', PickUp(robot, arm, item),
    #                                transitions={'succeeded': 'done',
    #                                             'failed':    'failed'})

if __name__ == "__main__":
    rospy.init_node('simple_navigate')

    import sys
    import robot_skills
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        print "unknown robot"
        sys.exit()

    robot = Robot()

    room_designator = ds.EdEntityDesignator(robot, id="livingroom")

    sm = FindPerson(robot, room_designator)
    sm.execute()
