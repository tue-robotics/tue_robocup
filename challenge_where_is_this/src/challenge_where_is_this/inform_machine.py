# System
import enum
# ROS
import math
import os

import rospkg
import rospy

import robot_smach_states.util.designators as ds
import smach
# TU/e
from ed.entity import Entity
from hmi import HMIResult
from robocup_knowledge import load_knowledge
from robot_skills.simulation import is_sim_mode
from robot_smach_states.human_interaction import (
    GiveDirections,
    HearOptions,
    HearOptionsExtra,
    Say,
    WaitForPersonInFront, human_interaction, ShowImageState,
)
from robot_smach_states.navigation import guidance, NavigateToSymbolic, NavigateToWaypoint, ForceDrive
from robot_smach_states.utility import WaitTime
from challenge_where_is_this.hear_options_extra_picovoice import HearOptionsExtraPicovoice

# Challenge where is this
from .simulation import mock_detect_operator

if is_sim_mode():
    rospy.loginfo("In Sim Mode. Using mock_detect_operator for detecting operator behind the robot")
    guidance._detect_operator_behind_robot = mock_detect_operator

# Load and extract knowledge here so that stuff fails on startup if not defined
knowledge = load_knowledge("challenge_where_is_this")
BACKUP_SCENARIOS = knowledge.backup_scenarios
INFORMATION_POINT_ID = knowledge.information_point_id


class WaitMode(enum.Enum):
    SPEECH = "speech"
    VISUAL = "visual"


# Defines whether speech recognition or (visual) person recognition is used to determine when to proceed
WAIT_MODE = WaitMode.SPEECH


class EntityFromHmiResults(ds.Designator):
    """
    Designator to pick the closest item on top of the table to grab. This is used for testing
    """

    def __init__(self, robot, hmi_result_des, parse=True):
        """
        Constructor

        :param robot: robot object
        :param hmi_result_des:
        """
        super(EntityFromHmiResults, self).__init__(resolve_type=Entity)

        self._robot = robot
        self._hmi_result_des = hmi_result_des
        self.parse = parse

    def _resolve(self):
        """
        Resolves

        :return: entity in the <area_description> of the <surface_designator> that is closest to the robot
        """
        entity_id = self._hmi_result_des.resolve().semantics
        if entity_id is None:
            return None

        entities = self._robot.ed.get_entities(uuid=entity_id)
        if entities:
            return entities[0]
        else:
            return None


class GuideToRoomOrObject(smach.StateMachine):
    def __init__(self, robot, entity_des, operator_distance=1.5, operator_radius=1.5):
        """
        Constructor

        :param robot: robot object
        :param entity_des: designator resolving to a room or a piece of furniture
        :param operator_distance: (float) check for the operator to be within this range of the robot
        :param operator_radius: (float) from the point behind the robot defined by `distance`, the person must be within
            this radius
        """
        smach.StateMachine.__init__(
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

            smach.StateMachine.add(
                "DETERMINE_TYPE",
                smach.CBState(determine_type),
                transitions={"guide": "GUIDE"},
            )

            smach.StateMachine.add(
                "GUIDE",
                guidance.GuideToSymbolic(
                    robot,
                    {entity_des: self.area_designator},
                    entity_des,
                    operator_distance=self.operator_distance,
                    operator_radius=self.operator_radius,
                ),
                transitions={
                    "arrived": "SAY_OPERATOR_STAND_IN_FRONT",
                    "unreachable": "WAIT_GUIDE_BACKUP",
                    "goal_not_defined": "goal_not_defined",
                    "lost_operator": "GUIDE_NAV_BACKUP",
                    "preempted": "preempted",
                },
            )

            smach.StateMachine.add(
                "WAIT_GUIDE_BACKUP",
                WaitTime(robot, 3.0),
                transitions={"waited": "GUIDE_BACKUP", "preempted": "preempted"},
            )

            smach.StateMachine.add(
                "GUIDE_BACKUP",
                guidance.GuideToSymbolic(
                    robot,
                    {entity_des: self.area_designator},
                    entity_des,
                    operator_distance=self.operator_distance,
                    operator_radius=self.operator_radius,
                ),
                transitions={
                    "arrived": "SAY_OPERATOR_STAND_IN_FRONT",
                    "unreachable": "GUIDE_BACKUP_FAILED",
                    "goal_not_defined": "goal_not_defined",
                    "lost_operator": "GUIDE_NAV_BACKUP",
                    "preempted": "preempted",
                },
            )

            smach.StateMachine.add(
                "GUIDE_BACKUP_FAILED",
                ForceDrive(robot, 0.0, 0, 0.5, math.pi / 0.5),
                transitions={"done": "GUIDE_NAV_BACKUP"},
            )

            smach.StateMachine.add(
                "GUIDE_NAV_BACKUP",
                guidance.GuideToSymbolic(
                    robot,
                    {entity_des: self.area_designator},
                    entity_des,
                    operator_distance=-1,
                    operator_radius=self.operator_radius,
                ),
                transitions={
                    "arrived": "SAY_OPERATOR_STAND_IN_FRONT",
                    "unreachable": "unreachable",
                    "goal_not_defined": "goal_not_defined",
                    "lost_operator": "unreachable",
                    "preempted": "preempted",
                },
            )

            smach.StateMachine.add(
                "SAY_OPERATOR_STAND_IN_FRONT",
                Say(robot, "We have arrived at the location. Please stand in front of me now and stay there."),
                transitions={"spoken": "HEAD_RESET_STAY_THERE"},
            )

            @smach.cb_interface(outcomes=["done"])
            def head_reset_stay_there(userdata=None):
                robot.head.reset()
                rospy.sleep(2.)
                return "done"

            smach.StateMachine.add(
                "HEAD_RESET_STAY_THERE",
                smach.CBState(head_reset_stay_there),
                transitions={"done": "WAIT_OPERATOR_IN_FRONT"},
            )

            smach.StateMachine.add(
                "WAIT_OPERATOR_IN_FRONT",
                WaitTime(robot, 5.0),
                transitions={"waited": "SAY_ARRIVED", "preempted": "preempted"},
            )

            smach.StateMachine.add(
                "SAY_ARRIVED",
                Say(robot, "Great. I'll go back to the meeting point"),
                transitions={"spoken": "arrived"},
            )


class InformMachine(smach.StateMachine):
    def __init__(self, robot):
        """
        Constructor

        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            self.spec_des = ds.Designator(knowledge.location_grammar)
            self.answer_des = ds.VariableDesignator(resolve_type=HMIResult)
            self.entity_des = EntityFromHmiResults(robot, self.answer_des)
            self._location_hmi_attempt = 0
            self._max_hmi_attempts = 3  # ToDo: parameterize?

            @smach.cb_interface(outcomes=["reset"])
            def _reset_location_hmi_attempt(userdata=None):
                """Resets the location hmi attempt so that each operator gets three attempts"""
                self._location_hmi_attempt = 0
                return "reset"

            smach.StateMachine.add(
                "START_SAY",
                Say(
                    robot,
                    ["Please stand in front of me."],
                    block=True,
                ),
                transitions={"spoken": "RESET_HMI_ATTEMPT"},
            )

            smach.StateMachine.add(
                "RESET_HMI_ATTEMPT", smach.CBState(_reset_location_hmi_attempt), transitions={"reset": "SHOW_IMAGE_SPEAK"}
            )

            smach.StateMachine.add(
                'SHOW_IMAGE_SPEAK',
                ShowImageState(
                    robot,
                    os.path.join(rospkg.RosPack().get_path('challenge_restaurant'), "images", "speak.jpg"),
                    seconds=10
                ),
                transitions={'succeeded': 'INSTRUCT', 'failed': 'INSTRUCT'}
            )

            smach.StateMachine.add(
                "INSTRUCT",
                Say(
                    robot,
                    [
                        "Please tell me where you would like to go. Talk loudly into my microphone."
                    ],
                    block=True,
                ),
                transitions={"spoken": "LISTEN_FOR_LOCATION"},
            )

            if is_sim_mode():
                # Use state for simulation purposes
                smach.StateMachine.add(
                    "LISTEN_FOR_LOCATION",
                    HearOptionsExtra(robot, self.spec_des, self.answer_des.writeable, 6),
                    transitions={"heard": "ASK_CONFIRMATION", "no_result": "HANDLE_FAILED_HMI"},
                )
            else:
                # PICO voice implementation, can not be simulated
                smach.StateMachine.add(
                    "LISTEN_FOR_LOCATION",
                    HearOptionsExtraPicovoice(robot, 'where_is_this', self.answer_des.writeable, 6),
                    transitions={"heard": "ASK_CONFIRMATION", "no_result": "HANDLE_FAILED_HMI"},
                )


            smach.StateMachine.add(
                "ASK_CONFIRMATION",
                Say(
                    robot,
                    ["I hear that you would like to go to the {place}, is this correct?"],
                    place=ds.AttrDesignator(self.entity_des, "uuid", resolve_type=str),
                ),
                transitions={"spoken": "CONFIRM_LOCATION"},
            )

            smach.StateMachine.add(
                "CONFIRM_LOCATION",
                HearOptions(robot=robot, options=["yes", "no"]),
                transitions={"yes": "INSTRUCT_FOR_WAIT", "no": "INSTRUCT", "no_result": "HANDLE_FAILED_HMI"},
            )

            @smach.cb_interface(outcomes=["retry", "fallback", "failed"])
            def _handle_failed_hmi(userdata=None):
                """Handle failed HMI queries so we can try up to x times"""
                self._location_hmi_attempt += 1  # Increment
                if self._location_hmi_attempt == self._max_hmi_attempts:
                    rospy.logwarn("HMI failed for the {} time, returning 'failed'".format(self._max_hmi_attempts))

                    if not BACKUP_SCENARIOS:
                        rospy.logwarn("No fallback scenario's available anymore")
                        return "failed"

                    backup = BACKUP_SCENARIOS.pop(0)
                    robot.speech.speak("I am sorry but I did not hear you", mood="sad", block=False)
                    robot.speech.speak(backup.sentence, block=False)
                    self.answer_des.writeable.write(HMIResult("", backup.entity_id))
                    return "fallback"

                rospy.loginfo(
                    "HMI failed for the {} time out of {}, retrying".format(
                        self._location_hmi_attempt, self._max_hmi_attempts
                    )
                )

                return "retry"

            smach.StateMachine.add(
                "HANDLE_FAILED_HMI",
                smach.CBState(_handle_failed_hmi),
                transitions={"retry": "INSTRUCT", "fallback": "INSTRUCT_FOR_WAIT", "failed": "failed"},
            )

            smach.StateMachine.add(
                "INSTRUCT_FOR_WAIT",
                Say(
                    robot,
                    [
                        "Let me think how to get to the {entity_id}",
                        "I will now determine the best route to the {entity_id}",
                    ],
                    entity_id=ds.AttrDesignator(self.entity_des, "uuid", resolve_type=str),
                ),
                transitions={"spoken": "INSTRUCT_FOR_WAIT_STEP_BACK"},
            )

            smach.StateMachine.add(
                "INSTRUCT_FOR_WAIT_STEP_BACK",
                Say(robot, "Please move two steps back"),
                transitions={"spoken": "INSTRUCT_FOR_WAIT_STEP_BACK_WAIT"},
            )

            smach.StateMachine.add(
                "INSTRUCT_FOR_WAIT_STEP_BACK_WAIT",
                WaitTime(robot, 3.0),
                transitions={"waited": "STAND_BEHIND_ME", "preempted": "STAND_BEHIND_ME"},
            )

            # smach.StateMachine.add(
            #     "GIVE_DIRECTIONS",
            #     GiveDirections(robot, self.entity_des),
            #     transitions={"succeeded": "STAND_BEHIND_ME", "failed": "failed"},
            # )

            smach.StateMachine.add(
                "STAND_BEHIND_ME",
                Say(robot, ["Please stand behind me"], block=True),
                transitions={"spoken": "STAND_BEHIND_ME_WAIT"},
            )

            smach.StateMachine.add(
                "STAND_BEHIND_ME_WAIT",
                WaitTime(robot, 5.0),
                transitions={"waited": "INSTRUCT_FOLLOW", "preempted": "INSTRUCT_FOLLOW"},
            )

            smach.StateMachine.add(
                "INSTRUCT_FOLLOW",
                Say(robot, ["Please follow me at one meter distance"], block=True),
                transitions={"spoken": "GUIDE_OPERATOR"},
            )

            smach.StateMachine.add(
                "GUIDE_OPERATOR",
                GuideToRoomOrObject(robot, self.entity_des),
                transitions={
                    "arrived": "RETURN_TO_INFORMATION_POINT",
                    "unreachable": "SAY_CANNOT_REACH",
                    "goal_not_defined": "SAY_CANNOT_REACH",
                    "lost_operator": "SAY_LOST_OPERATOR",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_CANNOT_REACH",
                Say(robot, ["I am sorry but I cannot reach the destination."], block=True),
                transitions={"spoken": "RETURN_TO_INFORMATION_POINT"},
            )

            smach.StateMachine.add(
                "SAY_LOST_OPERATOR",
                Say(robot, ["Oops I have lost you completely."], block=True),
                transitions={"spoken": "RETURN_TO_INFORMATION_POINT"},
            )

            smach.StateMachine.add(
                "RETURN_TO_INFORMATION_POINT",
                NavigateToWaypoint(robot, ds.EntityByIdDesignator(robot, INFORMATION_POINT_ID)),
                transitions={
                    "arrived": "succeeded",
                    "unreachable": "RETURN_TO_INFORMATION_POINT_FAILED",
                    "goal_not_defined": "failed"
                },
            )

            smach.StateMachine.add(
                "RETURN_TO_INFORMATION_POINT_FAILED",
                ForceDrive(robot, 0.0, 0, 0.5, math.pi / 0.5),
                transitions={"done": "RETURN_TO_INFORMATION_POINT"},
            )
