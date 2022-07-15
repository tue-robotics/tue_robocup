from functools import partial
import os
import math
from threading import Event

import rospy
from std_msgs.msg import String
import smach
from smach import StateMachine, State
import robot_smach_states.util.designators as ds
from robot_skills import get_robot
from robot_smach_states.human_interaction import Say
from robot_smach_states.human_interaction.human_interaction import WaitForPersonInFront
from robot_smach_states.navigation import ForceDrive
from robot_smach_states.navigation.navigate_to_waypoint import NavigateToWaypoint
from robot_smach_states.navigation import guidance
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
                    "arrived": "SAY_OPERATOR_STAND_IN_FRONT",
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
                    "arrived": "SAY_OPERATOR_STAND_IN_FRONT",
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
                    "arrived": "SAY_OPERATOR_STAND_IN_FRONT",
                    "unreachable": "unreachable",
                    "goal_not_defined": "goal_not_defined",
                    "lost_operator": "unreachable",
                    "preempted": "preempted",
                },
            )

            StateMachine.add(
                "SAY_OPERATOR_STAND_IN_FRONT",
                Say(robot, "We have arrived at the location. Please stand in front of me now and stay there."),
                transitions={"spoken": "HEAD_RESET_STAY_THERE"},
            )

            @smach.cb_interface(outcomes=["done"])
            def head_reset_stay_there(userdata=None):
                robot.head.reset()
                rospy.sleep(2.)
                return "done"

            StateMachine.add(
                "HEAD_RESET_STAY_THERE",
                smach.CBState(head_reset_stay_there),
                transitions={"done": "WAIT_OPERATOR_IN_FRONT"},
            )

            StateMachine.add(
                "WAIT_OPERATOR_IN_FRONT",
                WaitTime(robot, 5.0),
                transitions={"waited": "SAY_ARRIVED", "preempted": "preempted"},
            )

            StateMachine.add(
                "SAY_ARRIVED",
                Say(robot, "Great. I'll go back to the meeting point"),
                transitions={"spoken": "arrived"},
            )


class NavigateToTheDoorAndGuideNeighborToVictim(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])

        waypoint_door = {'id': 'entry_door', 'radius': 0.2}
        door_waypoint = ds.EntityByIdDesignator(robot, uuid=waypoint_door['id'])
        victim_entity = ds.EntityByIdDesignator(robot, uuid='victim')

        with self:
            StateMachine.add("SAY_WAITING_DOORBELL", Say(robot, "I am waiting for the doorbell", block=False),
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
                             Say(robot, ["Please follow me fast to Arpit"],
                                 block=False, look_at_standing_person=True),
                             transitions={'spoken': 'GUIDE_OPERATOR'})

            StateMachine.add(
                "GUIDE_OPERATOR",
                GuideToRoomOrObject(robot, victim_entity),
                transitions={
                    "arrived": "done",
                    "unreachable": "preempted",
                    "goal_not_defined": "preempted",
                    "lost_operator": "preempted",
                    "preempted": "preempted",
                },
            )


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    NavigateToTheDoorAndGuideNeighborToVictim(robot_instance).execute()
    # sm = WaitForStringMsg(robot_instance, timeout=30)
    # sm.execute()
