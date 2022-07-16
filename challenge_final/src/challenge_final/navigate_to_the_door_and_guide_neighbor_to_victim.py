import math
import os
from functools import partial
from threading import Event

import rospy
import smach
from smach import StateMachine, State
from smach.state import CBState
from smach.util import cb_interface
from std_msgs.msg import String

import robot_smach_states.util.designators as ds
from robot_skills import get_robot
from robot_smach_states.human_interaction import Say
from robot_smach_states.human_interaction.human_interaction import WaitForPersonInFront
from robot_smach_states.navigation import ForceDrive
from robot_smach_states.navigation import guidance
from robot_smach_states.navigation.navigate_to_waypoint import NavigateToWaypoint
from robot_smach_states.utility import WaitTime


class WaitForStringMsg(State):
    def __init__(self, robot, timeout=30, topic="message_from_ros", data="There is someone at your door"):
        State.__init__(self, outcomes=["received", "timeout"])

        self.robot = robot
        self.timeout = timeout
        self.topic = f"{robot.robot_name}/{topic}"
        self.data = data.lower()

    def cb(self, event: Event, msg: String):
        if msg.data.lower() == self.data:
            event.set()

    def execute(self, userdata=None):
        event = Event()
        rospy.loginfo("Registering bell listener")
        lazy_sub = rospy.Subscriber(self.topic, String, partial(self.cb, event))

        bell_before_timeout = event.wait(self.timeout)

        rospy.loginfo("Unregistering bell listener")
        lazy_sub.unregister()

        if bell_before_timeout:
            rospy.loginfo("Bell has been ringed")
            return "received"

        rospy.loginfo("Doorbell timed-out")
        return "timeout"


class GuideToRoomOrObject(StateMachine):
    def __init__(self, robot, entity_des, operator_distance=1.5, operator_radius=1.5):
        """
        Constructor

        :param robot: robot object
        :param entity_des: designator resolving to a room or a piece of furniture
        :param operator_distance: (float) check for the operator to be within this range of the robot
        :param operator_radius: (float) from the point behind the robot defined by `distance`, the person must be within
            this radius
        """
        StateMachine.__init__(
            self, outcomes=["arrived", "unreachable", "goal_not_defined", "lost_operator", "preempted"]
        )

        self.operator_distance = operator_distance
        self.operator_radius = operator_radius
        self.area_designator = ds.VariableDesignator(resolve_type=str).writeable

        with self:

            @smach.cb_interface(outcomes=["guide"])
            def determine_type(userdata=None):
                entity = entity_des.resolve()
                if not entity:
                    rospy.logwarn("No entity in the designator")
                    return unreachable
                entity_type = entity.etype
                if entity_type == "room":
                    self.area_designator.write("in")
                else:
                    self.area_designator.write("in_front_of")

                return "guide"

            StateMachine.add(
                "DETERMINE_TYPE",
                smach.CBState(determine_type),
                transitions={"guide": "GUIDE"},
            )

            StateMachine.add(
                "GUIDE",
                guidance.GuideToSymbolic(
                    robot,
                    {entity_des: self.area_designator},
                    entity_des,
                    operator_distance=self.operator_distance,
                    operator_radius=self.operator_radius,
                    describe_near_objects=False
                ),
                transitions={
                    "arrived": "arrived",
                    "unreachable": "WAIT_GUIDE_BACKUP",
                    "goal_not_defined": "goal_not_defined",
                    "lost_operator": "GUIDE_NAV_BACKUP",
                    "preempted": "preempted",
                },
            )

            StateMachine.add(
                "WAIT_GUIDE_BACKUP",
                WaitTime(robot, 3.0),
                transitions={"waited": "GUIDE_BACKUP", "preempted": "preempted"},
            )

            StateMachine.add(
                "GUIDE_BACKUP",
                guidance.GuideToSymbolic(
                    robot,
                    {entity_des: self.area_designator},
                    entity_des,
                    operator_distance=self.operator_distance,
                    operator_radius=self.operator_radius,
                    describe_near_objects=False
                ),
                transitions={
                    "arrived": "arrived",
                    "unreachable": "GUIDE_BACKUP_FAILED",
                    "goal_not_defined": "goal_not_defined",
                    "lost_operator": "GUIDE_NAV_BACKUP",
                    "preempted": "preempted",
                },
            )

            StateMachine.add(
                "GUIDE_BACKUP_FAILED",
                ForceDrive(robot, 0.0, 0, 0.5, math.pi / 0.5),
                transitions={"done": "GUIDE_NAV_BACKUP"},
            )

            StateMachine.add(
                "GUIDE_NAV_BACKUP",
                guidance.GuideToSymbolic(
                    robot,
                    {entity_des: self.area_designator},
                    entity_des,
                    operator_distance=-1,
                    operator_radius=self.operator_radius,
                    describe_near_objects=False
                ),
                transitions={
                    "arrived": "arrived",
                    "unreachable": "unreachable",
                    "goal_not_defined": "goal_not_defined",
                    "lost_operator": "lost_operator",
                    "preempted": "preempted",
                },
            )


class NavigateToTheDoorAndGuideNeighborToVictim(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])

        waypoint_door = {'id': 'entry_door', 'radius': 0.2}
        door_waypoint = ds.EntityByIdDesignator(robot, uuid=waypoint_door['id'])
        victim_entity = ds.EntityByIdDesignator(robot, uuid='victim')
        cupboard_entity = ds.EntityByIdDesignator(robot, uuid='cupboard')

        with self:
            StateMachine.add("SAY_WAITING_DOORBELL", Say(robot, "I am waiting for the doorbell. I will stay with you "
                                                                "until your neighbour arrives", block=False),
                             transitions={'spoken': 'WAIT_FOR_BELL'})
            StateMachine.add("WAIT_FOR_BELL", WaitForStringMsg(robot, timeout=30),
                             transitions={'received': 'SAY_DOORBELL_RANG',
                                          'timeout': 'SAY_DOORBELL_RANG'})
            StateMachine.add("SAY_DOORBELL_RANG", Say(robot, "The doorbell rang, I am on my way", block=False),
                             transitions={'spoken': 'NAVIGATE_DOOR'})
            StateMachine.add('NAVIGATE_DOOR', NavigateToWaypoint(robot, door_waypoint, waypoint_door['radius']),
                             transitions={'arrived': 'SAY_PLEASE_COME_IN',
                                          'unreachable': 'SAY_NAVIGATE_TO_DOOR_FALLBACK',
                                          'goal_not_defined': 'preempted'})

            StateMachine.add('SAY_NAVIGATE_TO_DOOR_FALLBACK',
                             Say(robot, "Help, lets try it another way, hope this goes "
                                        "faster because we are in a hurry",
                                 block=False),
                             transitions={'spoken': 'TURN_AROUND'})

            StateMachine.add('TURN_AROUND', ForceDrive(robot, 0, 0, 0.5, (2 * math.pi) / 0.5),
                             transitions={'done': 'NAVIGATE_DOOR'})

            StateMachine.add('SAY_PLEASE_COME_IN',
                             Say(robot, ["Please come in soon, I'm waiting for you"],
                                 block=True, look_at_standing_person=True),
                             transitions={'spoken': 'WAIT_FOR_GUEST'})

            StateMachine.add("WAIT_FOR_GUEST", WaitForPersonInFront(robot, attempts=30, sleep_interval=1),
                             transitions={'success': 'SAY_FOLLOW_ME',
                                          'failed': 'SAY_PLEASE_COME_IN'})

            StateMachine.add('SAY_FOLLOW_ME',
                             Say(robot, ["Please follow me, we will first go to pick up the first aid kit."
                                         "Afterwards we will hurry to go and help Arpit"],
                                 block=False, look_at_standing_person=True),
                             transitions={'spoken': 'ROTATE_TO_OPERATOR'})

            StateMachine.add(
                "ROTATE_TO_OPERATOR",
                ForceDrive(robot, 0.0, 0, 0.5, math.pi / 0.5),
                transitions={"done": "GUIDE_OPERATOR_FAK"},
            )

            StateMachine.add(
                "GUIDE_OPERATOR_FAK",
                GuideToRoomOrObject(robot, cupboard_entity),
                transitions={
                    "arrived": "SAY_ARRIVED_AT_FAK",
                    "unreachable": "SAY_CANNOT_REACH_CUPBOARD",
                    "goal_not_defined": "SAY_CANNOT_REACH_CUPBOARD",
                    "lost_operator": "SAY_LOST_OPERATOR_CUPBOARD",
                    "preempted": "SAY_CANNOT_REACH_CUPBOARD",
                },
            )

            smach.StateMachine.add(
                "SAY_CANNOT_REACH_CUPBOARD",
                Say(robot, ["I am sorry but I cannot reach the cupboard, let's go to Arpit instead, follow me there!"],
                    block=True),
                transitions={"spoken": "GUIDE_OPERATOR_ARPIT"},
            )

            smach.StateMachine.add(
                "SAY_LOST_OPERATOR_CUPBOARD",
                Say(robot, ["Oops I have lost you completely, I will hurry to arpit!"], block=True),
                transitions={"spoken": "NAVIGATE_TO_ARPIT_DIRECTLY"},
            )

            smach.StateMachine.add(
                "NAVIGATE_TO_ARPIT_DIRECTLY",
                NavigateToWaypoint(robot, victim_entity),
                transitions={"arrived": "ARRIVED_AT_ARPIT",
                             "unreachable": "SAY_CANNOT_REACH",
                             "goal_not_defined": "SAY_CANNOT_REACH"},
            )

            StateMachine.add(
                "SAY_ARRIVED_AT_FAK",
                Say(robot, "We have arrived, please take the first aid kit from the cupboard. I will wait here!"),
                transitions={"spoken": "WAIT_AT_CUPBOARD"},
            )

            StateMachine.add(
                "WAIT_AT_CUPBOARD",
                WaitTime(10),
                transitions={"waited": "LETS_SAFE_ARPIT",
                             "preempted": "preempted"}
            )

            StateMachine.add(
                "LETS_SAFE_ARPIT",
                Say(robot, "Hurry we have to safe Arpit!"),
                transitions={"spoken": "ROTATE_TO_OPERATOR_2"},
            )

            StateMachine.add(
                "ROTATE_TO_OPERATOR_2",
                ForceDrive(robot, 0.0, 0, 0.5, math.pi / 0.5),
                transitions={"done": "GUIDE_OPERATOR_ARPIT"},
            )
            StateMachine.add(
                "GUIDE_OPERATOR_ARPIT",
                GuideToRoomOrObject(robot, victim_entity),
                transitions={
                    "arrived": "LOOK_DOWN",
                    "unreachable": "SAY_CANNOT_REACH",
                    "goal_not_defined": "SAY_CANNOT_REACH",
                    "lost_operator": "SAY_LOST_OPERATOR",
                    "preempted": "preempted",
                },
            )

            @cb_interface(outcomes=["done"])
            def _look_down(_):
                robot.head.look_down()
                robot.head.wait_for_motion_done()
                return "done"

            self.add("LOOK_DOWN", CBState(_look_down), transitions={"done": "ARRIVED_AT_ARPIT"})

            smach.StateMachine.add(
                "SAY_CANNOT_REACH",
                Say(robot, ["I am sorry but I cannot get to Arpit, please safe my human!"], block=True),
                transitions={"spoken": "done"},
            )

            smach.StateMachine.add(
                "SAY_LOST_OPERATOR",
                Say(robot, ["Oops I have lost you completely, please get to Arpit and safe my human."], block=True),
                transitions={"spoken": "done"},
            )

            smach.StateMachine.add(
                "ARRIVED_AT_ARPIT",
                Say(robot, ["Arpit I brought some help, please get well soon!"], block=True),
                transitions={"spoken": "done"},
            )


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    NavigateToTheDoorAndGuideNeighborToVictim(robot_instance).execute()
    # sm = WaitForStringMsg(robot_instance, timeout=30)
    # sm.execute()
