from typing import List

import rospy
import smach

from pykdl_ros import VectorStamped

from ed.entity import Entity
from robot_smach_states.util import designators as ds
from robot_smach_states.human_interaction import FindPeople


class PeopleInSeatDesignator(ds.Designator):
    def __init__(self, robot, seat: Entity, room: Entity = None, name=None):
        super(PeopleInSeatDesignator, self).__init__(resolve_type=[Entity], name=name)

        self.robot = robot

        ds.check_type(seat, Entity)
        if room is not None:
            ds.check_type(room, Entity)

        self.room = room
        self.seat = seat

    def _resolve(self) -> Entity:
        if self.room:
            room = ds.value_or_resolve(self.room)
            if not room:
                rospy.logwarn("Room is None, ignoring room constraints")

        seat = ds.value_or_resolve(self.seat)
        if not seat:
            rospy.logwarn("Seat is None, so cannot find seats there")
            return None
        if not seat.pose:
            rospy.logwarn("Seat does not have a pose")
            return None

        seat_point = VectorStamped(seat.pose.frame.p, seat.pose.header.stamp, seat.pose.header.frame_id)

        # query the people within 1m of the furniture: TODO adapt to the size of the furniture. i.e. couch/sofa
        people: List[Entity] = self.robot.ed.get_entities(
            etype="person", center_point=seat_point, radius=1.0, ignore_z=True
        )

        # TODO ignore people not within the specified room. i.e spectators
        people = [person for person in people if person.pose.frame.p]
        return people

    def __repr__(self):
        return "PersonInSeatDesignator({}, {}, {}, {})".format(self.robot, self.seat, self.room, self.name)


class _CheckPeople(smach.State):
    def __init__(self, people_des, number_of_people_des):
        """
        Constructor

        :param people_des: designator with the detected people
        :param number_of_people_des: designator which will result in the number of people near the entity, or None in case of state fails.
        """
        smach.State.__init__(self, outcomes=["empty", "occupied", "failed"])

        ds.check_resolve_type(people_des, [Entity])
        self.people_des = people_des
        ds.check_resolve_type(number_of_people_des, int)
        ds.is_writeable(number_of_people_des)
        self.number_of_people_des = number_of_people_des

    def execute(self, ud=None):
        people = self.people_des.resolve()
        if people is None or not isinstance(people, list):
            return "failed"

        number_of_people = len(people)
        self.number_of_people_des.write(number_of_people)

        if number_of_people == 0:
            return "empty"
        else:
            return "occupied"


class CheckSeatEmpty(smach.StateMachine):
    def __init__(self, robot, entity_des, number_of_people_des=None):
        """
        Constructor

        :param robot: robot object
        :param entity_des: EdEntityDesignator indicating the (furniture) object to check
        :param number_of_people_des: designator which will result in the number of people near the entity, or None in case of state fails.
        """
        # TODO implement logic for percent vs volume check in state machine rather than in the states themselves
        smach.StateMachine.__init__(self, outcomes=["empty", "occupied", "failed"])

        if number_of_people_des:
            ds.check_resolve_type(number_of_people_des, int)
        else:
            number_of_people_des = ds.VariableDesignator(resolve_type=int)

        people_in_seat_des = PeopleInSeatDesignator(robot, entity_des)

        with self:
            # TODO add state to turn head towards the entity.
            smach.StateMachine.add(
                "INSPECT_SEAT",
                FindPeople(
                    robot=robot,
                    # properties=None,
                    # query_entity_designator=None,
                    # found_people_designator=None,
                    # look_distance=10.0,
                    # speak=False,
                    # strict=True,
                    # nearest=False,
                    attempts=1,
                    # search_timeout=60,
                    look_range=(0.0, 0.0),
                    look_steps=1,
                ),
                transitions={"done": "CHECK_EMPTY", "failed": "failed"},
            )

            smach.StateMachine.add(
                "CHECK_EMPTY",
                _CheckPeople(people_in_seat_des, number_of_people_des.writeable),
                transitions={"empty": "empty", "occupied": "occupied", "failed": "failed"},
            )
