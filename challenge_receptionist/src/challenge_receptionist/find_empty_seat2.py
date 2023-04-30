#! /usr/bin/env python

from __future__ import print_function

from typing import List

import rospy
from ed.entity import Entity

from pykdl_ros import VectorStamped

from robot_smach_states.human_interaction import Say
from robot_smach_states.designator_iterator import IterateDesignator
from robot_smach_states.world_model import CheckVolumeEmpty
from robot_smach_states.reset import ResetArms
from challenge_receptionist.point_at_receptionist import PointAtReception
import robot_smach_states.util.designators as ds
import smach


class SeatsInRoomDesignator(ds.Designator):
    def __init__(self, robot, seat_ids, room, name=None):
        super(SeatsInRoomDesignator, self).__init__(resolve_type=[Entity], name=name)

        self.robot = robot

        ds.check_type(seat_ids, [str])
        ds.check_type(room, Entity)

        self.room = room
        self.seat_ids = seat_ids

    def _resolve(self):
        room = self.room.resolve() if hasattr(self.room, 'resolve') else self.room  # type: Entity
        if not room:
            rospy.logwarn("Room is None, so cannot find seats there")
            return None
        seats = [self.robot.ed.get_entity(seat_id) for seat_id in self.seat_ids]  # type: List[Entity]

        true_seats = [seat for seat in seats if seat is not None]  # get_entity returns None if entity does not exist

        seats_in_room = room.entities_in_volume(true_seats, "in")

        return seats_in_room

    def __repr__(self):
        return "SeatsInRoomDesignator({}, {}, {}, {})".format(self.robot, self.seat_ids, self.room, self.name)


class PersonInSeatDesignator(ds.Designator):
    def __init__(self, robot, seat: Entity, room: Entity=None, name=None):
        super(PersonInSeatDesignator, self).__init__(resolve_type=[Entity], name=name)

        self.robot = robot

        ds.check_type(seat, Entity)
        if room is not None:
            ds.check_type(room, Entity)

        self.room = room
        self.seat = seat

    def _resolve(self) -> Entity:

        if self.room:
            room = self.room.resolve() if hasattr(self.room, 'resolve') else self.room  # type: Entity
            if not room:
                rospy.logwarn("Room is None, ignoring room constraints")
            
        seat = self.seat.resolve() if hasattr(self.seat, 'resolve') else self.seat  # type: Entity
        if not seat:
            rospy.logwarn("Seat is None, so cannot find seats there")
            return None
        if not seat.pose:
            rospy.logwarn("Seat does not have a pose")
            return None
        
        seat_point = VectorStamped(seat.pose.frame.p, seat.pose.header.stamp, seat.pose.header.frame_id)
        person_string="person"
        
        # query the people within 1m of the furniture: TODO adapt to the size of the furniture. i.e. couch/sofa
        people: List(Entity) = self.robot.ed.get_entities(etype="person", center_point=seat_point, radius=1.0, ignore_z=True)

        #TODO ignore people not within the specified room. i.e spectators
        return people

    def __repr__(self):
        return "PersonInSeatDesignator({}, {}, {}, {})".format(self.robot, self.seat, self.room, self.name)


class FindEmptySeat(smach.StateMachine):
    """
    Iterate over all seat-type objects and check that their 'on-top-of' volume is empty
    That can be done with an Inspect and then query for any Entities inside that volume.
    If there are none, then the seat is empty
    """

    def __init__(self, robot, seats_to_inspect: dict, room, fit_supporting_entity=False, seat_is_for=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        seats_volumes_des = ds.VariableDesignator(seats_to_inspect)
        seats = SeatsInRoomDesignator(robot, list(seats_to_inspect.keys()), room, "seats_in_room")
        seat_ent_des = ds.VariableDesignator(resolve_type=Entity)
        seat_ent_uuid_des = ds.AttrDesignator(seat_ent_des, 'uuid', resolve_type=str)
        volumes_des = ds.ValueByKeyDesignator(seats_volumes_des, seat_ent_uuid_des, resolve_type=[str],
                                              name="volumes_des")
        volume_des = ds.VariableDesignator(resolve_type=str)

        if seat_is_for:
            ds.check_type(seat_is_for, str)
        else:
            seat_is_for = ds.Designator(' ')

        with self:
            smach.StateMachine.add('SAY_LETS_FIND_SEAT',
                                   Say(robot,
                                       ["Let me find a place for {name} to sit. Please be patient while I check "
                                        "out where there's place to sit"],
                                       name=seat_is_for,
                                       block=False),
                                   transitions={'spoken': 'ITERATE_NEXT_SEAT'})

            smach.StateMachine.add('ITERATE_NEXT_SEAT',
                                   IterateDesignator(seats, seat_ent_des.writeable),
                                   transitions={'next': 'ITERATE_NEXT_VOLUME',
                                                'stop_iteration': 'SAY_NO_EMPTY_SEATS'})

            smach.StateMachine.add('ITERATE_NEXT_VOLUME',
                                   IterateDesignator(volumes_des, volume_des.writeable),
                                   transitions={'next': 'CHECK_SEAT_EMPTY',
                                                'stop_iteration': 'ITERATE_NEXT_SEAT'})

            smach.StateMachine.add('CHECK_SEAT_EMPTY',
                                   CheckVolumeEmpty(robot, seat_ent_des, volume_des, 0.6, None),
                                   transitions={'occupied': 'ITERATE_NEXT_VOLUME',
                                                'empty': 'POINT_AT_EMPTY_SEAT',
                                                'partially_occupied': 'POINT_AT_PARTIALLY_OCCUPIED_SEAT',
                                                'failed': 'ITERATE_NEXT_VOLUME'})

            smach.StateMachine.add('POINT_AT_EMPTY_SEAT',
                                   PointAtReception(robot=robot,
                                                    arm_designator=ds.UnoccupiedArmDesignator(robot, {
                                                        'required_goals': ['point_at']}),
                                                    point_at_designator=seat_ent_des,
                                                    volume=volume_des,
                                                    look_at_designator=seat_ent_des),
                                   transitions={"succeeded": "SAY_SEAT_EMPTY",
                                                "failed": "SAY_SEAT_EMPTY"})

            smach.StateMachine.add('SAY_SEAT_EMPTY',
                                   Say(robot,
                                       ["Please sit on the {seat}, {name}!"],
                                       name=seat_is_for,
                                       seat=ds.AttrDesignator(seat_ent_des, 'uuid', resolve_type=str),
                                       block=True),
                                   transitions={'spoken': 'RESET_SUCCESS'})

            smach.StateMachine.add('POINT_AT_PARTIALLY_OCCUPIED_SEAT',
                                   PointAtReception(robot=robot,
                                                    arm_designator=ds.UnoccupiedArmDesignator(robot, {
                                                        'required_goals': ['point_at']}),
                                                    point_at_designator=seat_ent_des,
                                                    volume=volume_des,
                                                    look_at_designator=seat_ent_des),
                                   transitions={"succeeded": "SAY_SEAT_PARTIALLY_OCCUPIED",
                                                "failed": "SAY_SEAT_PARTIALLY_OCCUPIED"})

            smach.StateMachine.add('SAY_SEAT_PARTIALLY_OCCUPIED',
                                   Say(robot,
                                       ["I think there's some space left here where you can sit {name},"
                                        "Please leave some space for any potential future guests!"],
                                       name=seat_is_for,
                                       block=True),
                                   transitions={'spoken': 'RESET_SUCCESS'})

            smach.StateMachine.add('SAY_NO_EMPTY_SEATS',
                                   Say(robot,
                                       ["Sorry, there are no empty seats. I guess you just have to stand {name}"],
                                       name=seat_is_for,
                                       block=True),
                                   transitions={'spoken': 'RESET_FAIL'})

            smach.StateMachine.add('RESET_FAIL',
                                   ResetArms(robot),
                                   transitions={'done': 'failed'})

            smach.StateMachine.add('RESET_SUCCESS',
                                   ResetArms(robot),
                                   transitions={'done': 'succeeded'})


if __name__ == "__main__":
    import sys
    from robot_skills import get_robot

    if len(sys.argv) < 3:
        print(
            "Please provide robot_name, and seat to inspect as arguments. Eg. 'hero sofa")
        sys.exit(1)

    from robot_smach_states.util.designators import EntityByIdDesignator
    robot_name = sys.argv[1]
    seat = sys.argv[2]

    
    rospy.init_node('test_find_emtpy_seat')
    robot = get_robot(robot_name)

    seat_entity = EntityByIdDesignator(robot, seat) 
    
    people_designator = PersonInSeatDesignator(robot, seat_entity, name="people_in_seat_designator")
    found_people = people_designator.resolve()

    rospy.loginfo(f"Designator {people_designator} resolved to {found_people}")