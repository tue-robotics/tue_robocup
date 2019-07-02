#! /usr/bin/env python
import rospy
import robot_smach_states as states
import robot_smach_states.util.designators as ds
import smach
from robot_skills.util.entity import Entity
from robot_skills.classification_result import ClassificationResult


class CheckVolumeEmpty(smach.StateMachine):
    def __init__(self, robot, entity_des, volume):
        smach.StateMachine.__init__(self, outcomes=['empty', 'occupied', 'failed'])

        seen_entities_des = ds.VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            smach.StateMachine.add('INSPECT',
                                   states.Inspect(robot, entity_des, searchArea=volume, objectIDsDes=seen_entities_des),
                                   transitions = {"done": "CHECK",
                                                  "failed": "failed"})

            @smach.cb_interface(outcomes=['empty', 'occupied'])
            def check_occupied(userdata):
                seen_entities = seen_entities_des.resolve()
                if seen_entities:
                    return 'occupied'
                else:
                    return 'empty'
            smach.StateMachine.add('CHECK', smach.CBState(check_occupied),
                                   transitions={'empty': 'empty',
                                                'occupied': 'occupied'})


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
        seats = [self.robot.ed.get_entity(seat_id) for seat_id in self.seat_ids]  # type: List[Entity]

        true_seats = [seat for seat in seats if seat is not None]  # get_entity returns None if entity does not exist

        seats_in_room = room.entities_in_volume(true_seats,"in")

        return seats_in_room

    def __repr__(self):
        return "SeatsInRoomDesignator({}, {}, {}, {})".format(self.robot, self.seat_ids, self.room, self.name)


class FindEmptySeat(smach.StateMachine):
    """
    Iterate over all seat-type objects and check that their 'on-top-of' volume is empty
    That can be done with an Inspect and then query for any Entities inside that volume.
    If there are none, then the seat is empty
    """
    def __init__(self, robot, seats_to_inspect, room):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        seats = SeatsInRoomDesignator(robot, seats_to_inspect, room, "seats_in_room")
        seat_ent_des = ds.VariableDesignator(resolve_type=Entity)
        with self:
            smach.StateMachine.add('ITERATE_NEXT_SEAT',
                                   states.IterateDesignator(seats, seat_ent_des.writeable),
                                   transitions={'next': 'CHECK_SEAT_EMPTY',
                                                'stop_iteration': 'SAY_NO_EMPTY_SEATS'})

            smach.StateMachine.add('CHECK_SEAT_EMPTY',
                                   CheckVolumeEmpty(robot, seat_ent_des, 'on_top_of'),
                                   transitions={'occupied': 'ITERATE_NEXT_SEAT',
                                                'empty': 'POINT_AT_EMPTY_SEAT',
                                                'failed': 'ITERATE_NEXT_SEAT'})


            smach.StateMachine.add('POINT_AT_EMPTY_SEAT',
                                   states.PointAt(robot=robot,
                                                  arm_designator=ds.UnoccupiedArmDesignator(robot, {'required_goals':['point_at']}),
                                                  point_at_designator=seat_ent_des,
                                                  look_at_designator=seat_ent_des),
                                   transitions={"succeeded": "SAY_SEAT_EMPTY",
                                                "failed": "SAY_SEAT_EMPTY"})

            smach.StateMachine.add('SAY_SEAT_EMPTY',
                                   states.Say(robot, ["Please sit here"], block=True),
                                   transitions={'spoken': 'RESET_SUCCESS'})

            smach.StateMachine.add('SAY_NO_EMPTY_SEATS',
                                   states.Say(robot, ["Sorry, there are no empty seats. I guess you just have to stand"], block=True),
                                   transitions={'spoken': 'RESET_FAIL'})

            smach.StateMachine.add('RESET_FAIL',
                                   states.ResetArmsTorsoHead(robot),
                                   transitions={'done': 'failed'})

            smach.StateMachine.add('RESET_SUCCESS',
                                   states.ResetArmsTorsoHead(robot),
                                   transitions={'done': 'succeeded'})


if __name__ == "__main__":
    rospy.init_node("find_emtpy_seat")
    from robot_smach_states.util.startup import startup

    class Test(smach.StateMachine):
        def __init__(self, robot):
            smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])
            with self:
                smach.StateMachine.add('FIND_EMTPY_SEAT',
                                       FindEmptySeat(robot,
                                                     seats_to_inspect=['hallway_table'],  # couch
                                                     room=ds.EntityByIdDesignator(robot, 'hallway')))


    startup(Test)
