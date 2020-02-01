# System
import enum

# ROS
import rospy
import smach

# TU/e
import robot_skills
from robot_skills.simulation import is_sim_mode
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from hmi import HMIResult
from robocup_knowledge import load_knowledge
from robot_smach_states.human_interaction.give_directions import GiveDirections
from robot_smach_states.navigation import guidance

# Challenge where is this
from .simulation import mock_detect_operator

if is_sim_mode():
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
        super(EntityFromHmiResults, self).__init__(resolve_type=robot_skills.util.entity.Entity)

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

        entities = self._robot.ed.get_entities(id=entity_id, parse=self.parse)
        if entities:
            return entities[0]
        else:
            return None


class GuideToRoomOrObject(smach.StateMachine):
    def __init__(self, robot, entity_des, operator_distance=1.5, operator_radius=0.5):
        """
        Constructor

        :param robot: robot object
        :param entity_des: designator resolving to a room or a piece of furniture
        :param operator_distance: (float) check for the operator to be within this range of the robot
        :param operator_radius: (float) from the point behind the robot defined by `distance`, the person must be within this radius
        """
        smach.StateMachine.__init__(
            self, outcomes=["arrived", "unreachable", "goal_not_defined", "lost_operator", "preempted"])

        self.operator_distance = operator_distance
        self.operator_radius = operator_radius

        with self:
            @smach.cb_interface(outcomes=["room", "object"])
            def determine_type(userdata=None):
                entity = entity_des.resolve()
                entity_type = entity.type
                if entity_type == "room":
                    return "room"
                else:
                    return "object"

            smach.StateMachine.add("DETERMINE_TYPE",
                                   smach.CBState(determine_type),
                                   transitions={"room": "GUIDE_TO_ROOM",
                                                "object": "GUIDE_TO_FURNITURE"})

            smach.StateMachine.add("GUIDE_TO_ROOM",
                                   guidance.GuideToSymbolic(robot, {entity_des: "in"}, entity_des,
                                       operator_distance=self.operator_distance, operator_radius=self.operator_radius),
                                   transitions={"arrived": "arrived",
                                                "unreachable": "WAIT_ROOM_BACKUP",
                                                "goal_not_defined": "goal_not_defined",
                                                "lost_operator": "ROOM_NAV_BACKUP",
                                                "preempted": "preempted"})

            smach.StateMachine.add("WAIT_ROOM_BACKUP",
                                   states.WaitTime(robot, 3.0),
                                   transitions={"waited": "GUIDE_TO_ROOM_BACKUP",
                                                "preempted": "preempted"})

            smach.StateMachine.add("GUIDE_TO_ROOM_BACKUP",
                                   guidance.GuideToSymbolic(robot, {entity_des: "in"}, entity_des,
                                       operator_distance=self.operator_distance, operator_radius=self.operator_radius),
                                   transitions={"arrived": "arrived",
                                                "unreachable": "unreachable",
                                                "goal_not_defined": "goal_not_defined",
                                                "lost_operator": "ROOM_NAV_BACKUP",
                                                "preempted": "preempted"})

            smach.StateMachine.add("GUIDE_TO_FURNITURE",
                                   guidance.GuideToSymbolic(robot, {entity_des: "in_front_of"}, entity_des,
                                       operator_distance=self.operator_distance, operator_radius=self.operator_radius),
                                   transitions={"arrived": "arrived",
                                                "unreachable": "WAIT_FURNITURE_BACKUP",  # Something is blocking
                                                "goal_not_defined": "GUIDE_NEAR_FURNITURE",  # in_front_of not defined
                                                "lost_operator": "FURNITURE_NAV_BACKUP",
                                                "preempted": "preempted"})

            smach.StateMachine.add("GUIDE_NEAR_FURNITURE",
                                   guidance.GuideToSymbolic(robot, {entity_des: "near"}, entity_des,
                                       operator_distance=self.operator_distance, operator_radius=self.operator_radius),
                                   transitions={"arrived": "arrived",
                                                "unreachable": "WAIT_FURNITURE_BACKUP",
                                                "goal_not_defined": "goal_not_defined",
                                                "lost_operator": "FURNITURE_NAV_BACKUP",
                                                "preempted": "preempted"})

            smach.StateMachine.add("WAIT_FURNITURE_BACKUP",
                                   states.WaitTime(robot, 3.0),
                                   transitions={"waited": "GUIDE_NEAR_FURNITURE_BACKUP",
                                                "preempted": "preempted"})

            smach.StateMachine.add("GUIDE_NEAR_FURNITURE_BACKUP",
                                   guidance.GuideToSymbolic(robot, {entity_des: "near"}, entity_des,
                                       operator_distance=self.operator_distance, operator_radius=self.operator_radius),
                                   transitions={"arrived": "arrived",
                                                "unreachable": "unreachable",
                                                "goal_not_defined": "goal_not_defined",
                                                "lost_operator": "FURNITURE_NAV_BACKUP",
                                                "preempted": "preempted"})

            smach.StateMachine.add("ROOM_NAV_BACKUP",
                                   states.NavigateToSymbolic(robot, {entity_des: "in"}, entity_des),
                                   transitions={"arrived": "SAY_ARRIVED",
                                                "unreachable": "unreachable",
                                                "goal_not_defined": "goal_not_defined",})

            smach.StateMachine.add("FURNITURE_NAV_BACKUP",
                                   states.NavigateToSymbolic(robot, {entity_des: "near"}, entity_des),
                                   transitions={"arrived": "SAY_ARRIVED",
                                                "unreachable": "unreachable",
                                                "goal_not_defined": "goal_not_defined",})

            smach.StateMachine.add("SAY_ARRIVED",
                                   states.Say(robot, "We have arrived. I'll go back to the meeting point"),
                                   transitions={"spoken": "arrived"})


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
                """ Resets the location hmi attempt so that each operator gets three attempts """
                self._location_hmi_attempt = 0
                return "reset"

            smach.StateMachine.add("RESET_HMI_ATTEMPT",
                                   smach.CBState(_reset_location_hmi_attempt),
                                   transitions={"reset": "ANNOUNCE_ITEM"})

            if WAIT_MODE == WaitMode.SPEECH:
                smach.StateMachine.add("ANNOUNCE_ITEM",
                                       states.Say(robot, "Hello, my name is {}. Please call me by my name. "
                                                         "Talk loudly into my microphone and wait for the ping".
                                                  format(robot.robot_name), block=True),
                                       transitions={"spoken": "WAIT_TO_BE_CALLED"})

                smach.StateMachine.add("WAIT_TO_BE_CALLED",
                                       states.HearOptions(robot, ["{}".format(robot.robot_name)], timeout=10),
                                       transitions={"{}".format(robot.robot_name): "INSTRUCT",
                                                    "no_result": "ANNOUNCE_ITEM"})

            elif WAIT_MODE == WaitMode.VISUAL:
                smach.StateMachine.add("ANNOUNCE_ITEM",
                                       states.Say(robot, "Hello, my name is {}. Please step in front of me.".format(
                                           robot.robot_name), block=True),
                                       transitions={"spoken": "WAIT_TO_BE_CALLED"})

                smach.StateMachine.add("WAIT_TO_BE_CALLED",
                                       states.WaitForPersonInFront(robot, attempts=10, sleep_interval=1.0),
                                       transitions={"success": "INSTRUCT",
                                                    "failed": "SAY_NOT_DETECTED"})

                smach.StateMachine.add("SAY_NOT_DETECTED",
                                       states.Say(robot, "I did not see you but will try to continue anyway.".format(
                                           robot.robot_name), block=True),
                                       transitions={"spoken": "INSTRUCT"})

            smach.StateMachine.add("INSTRUCT",
                                   states.Say(robot,
                                              ["Please tell me where you would like to go. "
                                               "Talk loudly into my microphone and wait for the ping"],
                                              block=True),
                                   transitions={"spoken": "LISTEN_FOR_LOCATION"})

            smach.StateMachine.add("LISTEN_FOR_LOCATION",
                                   states.HearOptionsExtra(robot, self.spec_des, self.answer_des.writeable,
                                                           rospy.Duration(6)),
                                   transitions={"heard": "ASK_CONFIRMATION",
                                                "no_result": "HANDLE_FAILED_HMI"})

            smach.StateMachine.add("ASK_CONFIRMATION",
                                   states.Say(robot, ["I hear that you would like to go to the {place},"
                                                               "is this correct?"],
                                                       place=ds.AttrDesignator(self.entity_des, "id",
                                                                                   resolve_type=str)),
                                   transitions={"spoken": "CONFIRM_LOCATION"})

            smach.StateMachine.add("CONFIRM_LOCATION",
                                   states.HearOptions(robot=robot, options=["yes", "no"]),
                                                      transitions={"yes": "INSTRUCT_FOR_WAIT",
                                                                   "no": "INSTRUCT",
                                                                   "no_result": "HANDLE_FAILED_HMI"})

            @smach.cb_interface(outcomes=["retry", "fallback", "failed"])
            def _handle_failed_hmi(userdata=None):
                """ Handle failed HMI queries so we can try up to x times """
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

                rospy.loginfo("HMI failed for the {} time out of {}, retrying".format(
                    self._location_hmi_attempt, self._max_hmi_attempts))

                return "retry"

            smach.StateMachine.add("HANDLE_FAILED_HMI",
                                   smach.CBState(_handle_failed_hmi),
                                   transitions={"retry": "INSTRUCT",
                                                "fallback": "INSTRUCT_FOR_WAIT",
                                                "failed": "failed"})

            smach.StateMachine.add("INSTRUCT_FOR_WAIT",
                                   states.human_interaction.Say(
                                       robot,
                                       ["Let me think how to get to the {entity_id}",
                                        "I will now determine the best route to the {entity_id}"],
                                       entity_id=ds.AttrDesignator(self.entity_des, "id", resolve_type=str)),
                                   transitions={"spoken": "GIVE_DIRECTIONS"})

            smach.StateMachine.add("GIVE_DIRECTIONS",
                                   GiveDirections(robot, self.entity_des),
                                   transitions={"succeeded": "INSTRUCT_FOLLOW",
                                                "failed": "failed"})

            smach.StateMachine.add("INSTRUCT_FOLLOW",
                                   states.Say(robot,
                                              ["Please follow me"],
                                              block=True),
                                   transitions={"spoken": "GUIDE_OPERATOR"})

            smach.StateMachine.add("GUIDE_OPERATOR",
                                   GuideToRoomOrObject(robot, self.entity_des),
                                   transitions={"arrived": "SUCCESS",
                                                "unreachable": "SAY_CANNOT_REACH",
                                                "goal_not_defined": "SAY_CANNOT_REACH",
                                                "lost_operator": "SAY_LOST_OPERATOR",
                                                "preempted": "failed"})

            smach.StateMachine.add("SUCCESS",
                                   states.Say(robot,
                                              ["We have arrived"],
                                              block=True),
                                   transitions={"spoken": "RETURN_TO_INFORMATION_POINT"})

            smach.StateMachine.add("SAY_CANNOT_REACH",
                                   states.Say(robot,
                                              ["I am sorry but I cannot reach the destination."],
                                              block=True),
                                   transitions={"spoken": "RETURN_TO_INFORMATION_POINT"})

            smach.StateMachine.add("SAY_LOST_OPERATOR",
                                   states.Say(robot,
                                              ["Oops I have lost you completely."],
                                              block=True),
                                   transitions={"spoken": "RETURN_TO_INFORMATION_POINT"})

            smach.StateMachine.add("RETURN_TO_INFORMATION_POINT",
                                   states.NavigateToWaypoint(
                                       robot,
                                       ds.EntityByIdDesignator(robot, INFORMATION_POINT_ID)
                                   ),
                                   transitions={"arrived": "succeeded",
                                                "unreachable": "failed",
                                                "goal_not_defined": "failed"})
