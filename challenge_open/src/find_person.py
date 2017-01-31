#! /usr/bin/env python

#
import math
import PyKDL as kdl

# ROS
import rospy
import smach
from geometry_msgs import msg as gm

# Tech United Eindhoven
from ed.msg import EntityInfo

# Executive stuff
import robot_smach_states.util.designators as ds
import robot_smach_states as states
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

        # If a room designator is specified, get it from ED
        if self._room_designator:
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
                    roompose = kdl.Frame(kdl.Rotation.Quaternion(room_entity.pose.orientation.x,
                                                                room_entity.pose.orientation.y,
                                                                room_entity.pose.orientation.z,
                                                                room_entity.pose.orientation.w),
                                         kdl.Vector(room_entity.pose.position.x,
                                                    room_entity.pose.position.y,
                                                    room_entity.pose.position.z))
                    hpose = roompose * kdl.Frame(kdl.Rotation(),
                                                 kdl.Vector(box['min']['x'], box['min']['y'], box['min']['z']))
                    convex_hull.append(hpose.p)
                    hpose = roompose * kdl.Frame(kdl.Rotation(),
                                                 kdl.Vector(box['max']['x'], box['min']['y'], box['min']['z']))
                    convex_hull.append(hpose.p)
                    hpose = roompose * kdl.Frame(kdl.Rotation(),
                                                 kdl.Vector(box['max']['x'], box['max']['y'], box['min']['z']))
                    convex_hull.append(hpose.p)
                    hpose = roompose * kdl.Frame(kdl.Rotation(),
                                                 kdl.Vector(box['min']['x'], box['max']['y'], box['min']['z']))
                    convex_hull.append(hpose.p)

        entities = self._robot.ed.get_entities(parse=True)
        possible_humans = []
        for e in entities:
            if e.is_a('possible_human'):
                possible_humans.append(e)

        if not possible_humans:
            rospy.logwarn("No possible humans found")
            return None

        # If we have a room designator, we try to pick a person in the room, as close to the center point as possible
        if self._room_designator:
            # Check which entities are in the room
            persons_in_room = []
            for ph in possible_humans:
                phposition = kdl.Vector(ph.pose.position.x, ph.pose.position.y, ph.pose.position.z)
                if geometry_helpers.isPointInsideHull(phposition, convex_hull):
                    persons_in_room.append(ph)
            if not persons_in_room:
                rospy.logwarn("None of the found possible humans was in the room")
                return None

            # Sort according to distance to center pose
            persons_in_room = sorted(persons_in_room,
                                     key=lambda ph: ph.distance_to_2d(room_entity._pose.p))

            # Return the best one
            return persons_in_room[0]
        else:
            # We just pick the person closest to the robot
            bp = self._robot.base.get_location()
            possible_humans = sorted(possible_humans,
                                     key=lambda ph: ph.distance_to_2d(bp.p))
            return possible_humans[0]


class FindPerson(smach.StateMachine):
    """ Class to find ANY person within a certain room. It does NOT look for a particular person by trying to recognize
    people. To find anyone, a laser rangefinder will be used """
    def __init__(self, robot, room_designator=None):
        """ Constructor
        :param robot: robot object
        :param room_designator: EdEntityDesignator for the room in which to look for a person
        :return:
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])


        if not room_designator:
            room_designator = ds.EdEntityDesignator(robot)
            person_designator = PersonDesignator(robot=robot,
                                                 room_designator=None)
            use_room = False
        else:
            use_room = True
            person_designator = PersonDesignator(robot=robot,
                                                 room_designator=room_designator)

        with self:
            smach.StateMachine.add("NAVIGATE_TO_ROOM",
                                   NavigateToSymbolic(robot, {room_designator:"in"}, room_designator),
                                   transitions={'arrived': 'NAVIGATE_TO_PERSON',
                                                'goal_not_defined': 'ROOM_NOT_DEFINED',
                                                'unreachable': 'SAY_CANNOT_REACH_ROOM'})

            smach.StateMachine.add("NAVIGATE_TO_PERSON",
                                   NavigateToSymbolic(robot=robot,
                                                      entity_designator_area_name_map={person_designator: "near",
                                                                                       room_designator: "in"},
                                                      entity_lookat_designator=person_designator),
                                   transitions={'arrived': 'SAY_FOUND',
                                                'goal_not_defined': 'SAY_NO_PERSON_YET',
                                                'unreachable': 'SAY_CANNOT_REACH_PERSON'})

            smach.StateMachine.add("SAY_FOUND",
                                   states.Say(robot=robot,
                                              sentence="I found you", block=True),
                                   transitions={'spoken': 'succeeded'})

            smach.StateMachine.add("SAY_NO_PERSON_YET",
                                   states.Say(robot=robot,
                                              sentence="I cannot find you, can you please stand up", block=True),
                                   transitions={'spoken': 'WAIT_NO_PERSON_YET'})

            smach.StateMachine.add("WAIT_NO_PERSON_YET",
                                   states.WaitTime(robot=robot, waittime=5.0),
                                   transitions={'waited': 'NAVIGATE_TO_PERSON_NOT_FOUND',
                                                'preempted': 'NAVIGATE_TO_PERSON_NOT_FOUND'})

            smach.StateMachine.add("NAVIGATE_TO_PERSON_NOT_FOUND",
                                   NavigateToSymbolic(robot=robot,
                                                      entity_designator_area_name_map={person_designator: "near",
                                                                                       room_designator: "in"},
                                                      entity_lookat_designator=person_designator),
                                   transitions={'arrived': 'SAY_FOUND',
                                                'goal_not_defined': 'SAY_CANNOT_FIND_PERSON',
                                                'unreachable': 'SAY_CANNOT_REACH_PERSON2'})

            smach.StateMachine.add("SAY_CANNOT_REACH_PERSON",
                                   states.Say(robot=robot,
                                              sentence="I cannot reach you, can you please move to an open space", block=True),
                                   transitions={'spoken': 'WAIT_CANNOT_REACH_PERSON'})

            smach.StateMachine.add("WAIT_CANNOT_REACH_PERSON",
                                   states.WaitTime(robot=robot, waittime=5.0),
                                   transitions={'waited': 'NAVIGATE_TO_PERSON_UNREACHABLE',
                                                'preempted': 'NAVIGATE_TO_PERSON_UNREACHABLE'})

            smach.StateMachine.add("NAVIGATE_TO_PERSON_UNREACHABLE",
                                   NavigateToSymbolic(robot=robot,
                                                      entity_designator_area_name_map={person_designator: "near",
                                                                                       room_designator: "in"},
                                                      entity_lookat_designator=person_designator),
                                   transitions={'arrived': 'SAY_FOUND',
                                                'goal_not_defined': 'SAY_CANNOT_FIND_PERSON',
                                                'unreachable': 'SAY_CANNOT_REACH_PERSON2'})

            smach.StateMachine.add("ROOM_NOT_DEFINED",
                                   states.Say(robot=robot,
                                              sentence="The room is not well specified, I cannot go there", block=True),
                                   transitions={'spoken': 'failed'})

            smach.StateMachine.add("SAY_CANNOT_REACH_ROOM",
                                   states.Say(robot=robot,
                                              sentence="I cannot reach the room", block=True),
                                   transitions={'spoken': 'failed'})

            smach.StateMachine.add("SAY_CANNOT_FIND_PERSON",
                                   states.Say(robot=robot,
                                              sentence="I cannot find you, I am so sorry", block=True),
                                   transitions={'spoken': 'failed'})

            smach.StateMachine.add("SAY_CANNOT_REACH_PERSON2",
                                   states.Say(robot=robot,
                                              sentence="I cannot reach you, I am so sorry", block=True),
                                   transitions={'spoken': 'failed'})

            smach.StateMachine.add("NAVIGATE_TO_PERSON_WITHOUT_ROOM",
                                   NavigateToObserve(robot=robot, entity_designator=person_designator),
                                   transitions={'arrived': 'SAY_FOUND',
                                                'goal_not_defined': 'SAY_NO_PERSON_YET_WITHOUT_ROOM',
                                                'unreachable': 'SAY_CANNOT_REACH_PERSON_WITHOUT_ROOM'})

            smach.StateMachine.add("SAY_CANNOT_REACH_PERSON_WITHOUT_ROOM",
                                   states.Say(robot=robot,
                                              sentence="I cannot reach you, can you please move to an open space", block=True),
                                   transitions={'spoken': 'WAIT_WITHOUT_ROOM'})

            smach.StateMachine.add("SAY_NO_PERSON_YET_WITHOUT_ROOM",
                                   states.Say(robot=robot,
                                              sentence="I cannot find you, can you please stand up", block=True),
                                   transitions={'spoken': 'WAIT_WITHOUT_ROOM'})

            smach.StateMachine.add("WAIT_WITHOUT_ROOM",
                                   states.WaitTime(robot=robot, waittime=5.0),
                                   transitions={'waited': 'NAVIGATE_TO_PERSON_WITHOUT_ROOM2',
                                                'preempted': 'NAVIGATE_TO_PERSON_WITHOUT_ROOM2'})

            smach.StateMachine.add("NAVIGATE_TO_PERSON_WITHOUT_ROOM2",
                                   NavigateToObserve(robot=robot, entity_designator=person_designator),
                                   transitions={'arrived': 'SAY_FOUND',
                                                'goal_not_defined': 'SAY_CANNOT_FIND_PERSON',
                                                'unreachable': 'SAY_CANNOT_REACH_PERSON'})

        # If the room is not specified, the robot can't go there so will start looking at its current location
        if not use_room:
            self.set_initial_state(["NAVIGATE_TO_PERSON_WITHOUT_ROOM"])


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

    if len(sys.argv) > 2:
        room = sys.argv[2]
    else:
        room = "kitchen"

    robot = Robot()

    room_designator = ds.EdEntityDesignator(robot, id=room)

    sm = FindPerson(robot, room_designator)
    sm = FindPerson(robot, None)
    sm.execute()
