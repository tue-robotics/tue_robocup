#! /usr/bin/env python
import rospy
from robot_skills.robot import Robot
from robot_smach_states.human_interaction import Say, FindPeopleInRoom
from robot_smach_states.designator_iterator import IterateDesignator
from robot_smach_states.manipulation import PointAt
from robot_smach_states.reset import ResetArms
from robot_smach_states.world_model import CheckVolumeEmpty
import robot_smach_states.util.designators as ds
import smach
from robot_skills.util.entity import Entity
from robot_skills.util.volume import Volume
from robot_skills.classification_result import ClassificationResult

try:
    from typing import List
except ImportError:
    pass


class SeatsInRoomDesignator(ds.Designator):
    def __init__(self, robot, seat_ids, room, name=None, debug=False):
        super(SeatsInRoomDesignator, self).__init__(resolve_type=[Entity], name=name)

        self.robot = robot

        ds.check_type(seat_ids, [str])
        ds.check_type(room, Entity)

        self.room = room
        self.seat_ids = seat_ids
        self.debug = debug

    def _resolve(self):
        if self.debug:
            import ipdb;ipdb.set_trace()
        room = self.room.resolve() if hasattr(self.room, 'resolve') else self.room  # type: Entity
        if not room:
            rospy.logwarn("Room is None, so cannot find seats there")
            return None
        seats = [self.robot.ed.get_entity(seat_id) for seat_id in self.seat_ids]  # type: List[Entity]

        true_seats = [seat for seat in seats if seat is not None]  # get_entity returns None if entity does not exist

        seats_in_room = room.entities_in_volume(true_seats,"in")

        return seats_in_room

    def __repr__(self):
        return "SeatsInRoomDesignator({}, {}, {}, {})".format(self.robot, self.seat_ids, self.room, self.name)


class SeatVolumeNamesDesignator(ds.Designator):
    def __init__(self, robot, seat_entity_designator, name=None, debug=False):
        super(SeatVolumeNamesDesignator, self).__init__(resolve_type=[str], name=name)

        self.robot = robot

        ds.check_type(seat_entity_designator, Entity)
        self.seat_entity_designator = seat_entity_designator

        self.debug = debug

    def _resolve(self):
        if self.debug:
            import ipdb;ipdb.set_trace()

        try:
            seat_entity = self.seat_entity_designator.resolve()  # type: Entity
            seat_volume_names = sorted([k for k in seat_entity.volumes.keys() if k.endswith('seat')])

            return seat_volume_names
        except AttributeError:
            return None

    def __repr__(self):
        return "SeatsInRoomDesignator({}, {}, {}, {})".format(self.robot, self.seat_ids, self.room, self.name)


class DetermineEmptySeat(smach.State):
    """
    Find an Entity with a volume that does *not* have a Person-entity in it
    This is an empty seat.
    """
    def __init__(self, robot, people_des, all_seats_des, empty_seat_des, empty_volume_name_des, debug=False):
        # type: (Robot, ds.Designator, ds.Designator, ds.VariableWriter, ds.VariableWriter) -> None
        super(DetermineEmptySeat, self).__init__(outcomes=["done"])
        self._debug = debug

        ds.check_type(people_des, [Entity])
        self._people_des = people_des

        ds.check_type(all_seats_des, [Entity])
        self._all_seats_des = all_seats_des

        ds.is_writeable(empty_seat_des)
        ds.check_type(empty_seat_des, Entity)
        self._empty_seat_des = empty_seat_des

        ds.is_writeable(empty_volume_name_des)
        ds.check_type(empty_volume_name_des, str)
        self._empty_volume_name_des = empty_volume_name_des

    def execute(self, userdata):
        if self._debug:
            import ipdb; ipdb.set_trace()
        people = self._people_des.resolve()  # type: List[Entity]
        all_seat_entities = self._all_seats_des.resolve()  # type: List[Entity] of class Person

        for seat_entity in all_seat_entities:
            # maybe TODO: do this in a dictionary comprehension?
            seat_volume_names = sorted([k for k in seat_entity.volumes.keys() if k.endswith('seat')])
            for seat_volume_name in seat_volume_names:
                people_in_seat_volume = seat_entity.entities_in_volume(people, volume_id=seat_volume_name)
                if not people_in_seat_volume:
                    self._empty_seat_des.write(seat_entity)
                    self._empty_volume_name_des.write(seat_volume_name)
                    break

        return 'done'

class FindEmptySeat(smach.StateMachine):
    """
    Iterate over all seat-type objects and check that their 'on-top-of' volume is empty
    That can be done with an Inspect and then query for any Entities inside that volume.
    If there are none, then the seat is empty
    """
    def __init__(self, robot, seats_to_inspect, room, seat_is_for=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        people = ds.VariableDesignator(resolve_type=[Entity])

        seats = SeatsInRoomDesignator(robot, seats_to_inspect, room, "seats_in_room")
        seat_ent_des = ds.VariableDesignator(resolve_type=Entity, name='seat_entity_designator')
        if seat_is_for:
            ds.check_type(seat_is_for, str)
        else:
            seat_is_for = ds.Designator('someone')

        seat_volume_des = ds.VariableDesignator(resolve_type=str, name='seat_volume_name_designator')

        with self:
            smach.StateMachine.add('SAY_LETS_FIND_SEAT',
                                   Say(robot,
                                              ["Let me find a place for {name} to sit. Please be patient while I check out where there's place to sit"],
                                              name=seat_is_for,
                                              block=False),
                                   transitions={'spoken': 'FIND_PEOPLE_IN_ROOM'})

            smach.StateMachine.add('FIND_PEOPLE_IN_ROOM',
                                   FindPeopleInRoom(robot, room, people.writeable),
                                   transitions={"found": "DETERMINE_EMPTY_SEAT",
                                                "not_found": "DETERMINE_EMPTY_SEAT"})  # No people found means you can sit anywhere

            smach.StateMachine.add('DETERMINE_EMPTY_SEAT',
                                   DetermineEmptySeat(self.robot, people, seats, seat_ent_des.writeable, seat_volume_des.writeable),
                                   transitions={"done": "POINT_AT_EMPTY_SEAT"})


            smach.StateMachine.add('POINT_AT_EMPTY_SEAT',
                                   PointAt(robot=robot,
                                                  arm_designator=ds.UnoccupiedArmDesignator(robot, {'required_goals':['point_at']}),
                                                  point_at_designator=seat_ent_des,
                                                  look_at_designator=seat_ent_des),
                                   transitions={"succeeded": "SAY_SEAT_EMPTY",
                                                "failed": "SAY_SEAT_EMPTY"})

            smach.StateMachine.add('SAY_SEAT_EMPTY',
                                   Say(robot,
                                              ["Please sit on the {seat}, {name}"],
                                              name=seat_is_for,
                                              seat=ds.AttrDesignator(seat_ent_des, 'id', resolve_type=str),
                                              block=True),
                                   transitions={'spoken': 'RESET_SUCCESS'})

            smach.StateMachine.add('POINT_AT_PARTIALLY_OCCUPIED_SEAT',
                                   PointAt(robot=robot,
                                                  arm_designator=ds.UnoccupiedArmDesignator(robot, {'required_goals':['point_at']}),
                                                  point_at_designator=seat_ent_des,
                                                  look_at_designator=seat_ent_des),
                                   transitions={"succeeded": "SAY_SEAT_PARTIALLY_OCCUPIED",
                                                "failed": "SAY_SEAT_PARTIALLY_OCCUPIED"})

            smach.StateMachine.add('SAY_SEAT_PARTIALLY_OCCUPIED',
                                   Say(robot,
                                              ["I think there's some space left here where you can sit {name}"],
                                              name=seat_is_for,
                                              block=True),
                                   transitions={'spoken': 'RESET_SUCCESS'})

            smach.StateMachine.add('SAY_NO_EMPTY_SEATS',
                                   Say(robot,
                                              ["Sorry, there are no empty seats. I guess you just have to stand, {name}"],
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

    if len(sys.argv) < 4:
        print "Please provide robot_name, room and seats_to_inspect as arguments. Eg. 'hero livingroom dinner_table bar dinnertable",
        sys.exit(1)

    robot_name = sys.argv[1]
    room = sys.argv[2]
    seats_to_inspect = sys.argv[3:]

    rospy.init_node('test_find_emtpy_seat')
    robot = get_robot(robot_name)

    sm = FindEmptySeat(robot,
                       seats_to_inspect=seats_to_inspect,
                       room=ds.EntityByIdDesignator(robot, room))
    sm.execute()
