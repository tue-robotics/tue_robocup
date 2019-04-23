import smach
import rospy
import robot_smach_states
import random

from robot_skills.util.kdl_conversions import FrameStamped
from robot_smach_states.util.designators import UnoccupiedArmDesignator, OccupiedArmDesignator, Designator

from PyKDL import Frame


class dropPoseDesignator(Designator):
    def __init__(self, robot, drop_height, name):
        super(dropPoseDesignator, self).__init__(resolve_type=FrameStamped, name=name)

        self._robot = robot
        self._drop_height = drop_height

    def _resolve(self):
        frame = None

        # Query ed
        try:
            frame = self._robot.ed.get_entity(id="trashbin")._pose
        except:
            return None

        frame.p.z(self._drop_height)

        return FrameStamped(frame, "/map")

class DetermineCleanupLocation(smach.State):
    def __init__(self, robot, selected_entity_designator, known_types):
        smach.State.__init__(self, outcomes=["trashbin", "other", "failed"])
        self._robot = robot
        self._known_types = known_types
        self._selected_entity_designator = selected_entity_designator

    def execute(self, userdata):
        selected_entity = self._selected_entity_designator.resolve()

        if not selected_entity:
            rospy.logerr("Could not resolve the selected entity!")
            return "failed"

        rospy.loginfo("The type of the entity is '%s'" % selected_entity.type)

        #if we don't know the entity we thow it in the trash
        if selected_entity.type in self._known_types:
            # TODO determine the location where the object should be placed
            return "other"
        else:
            return "trashbin"

class ArmFree(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["yes", "no"])
        self._robot = robot

    def execute(self, userdata):
        d = UnoccupiedArmDesignator(self._robot.arms, self._robot.rightArm, name="empty_arm_designator2")
        if d.resolve():
            return "yes"
        return "no"

class ArmOccupied(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["yes", "no"])
        self._robot = robot

    def execute(self, userdata):
        d = OccupiedArmDesignator(self._robot.arms, self._robot.rightArm, name="empty_arm_designator2")
        if d.resolve():
            return "yes"
        return "no"

class Speak(smach.State):
    def __init__(self, robot, selected_entity_designator, location_id, segment_area):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        self._selected_entity_designator = selected_entity_designator

        object_description = "%s the %s" % (segment_area, location_id)
        self._sentences = [
            "I will grab the %s " + object_description,
            "Grabbing the %s " + object_description,
            "Cleaning the %s " + object_description,
            "Getting rid of the %s " + object_description,
            "Removing the %s " + object_description
        ]

    def execute(self, userdata):
        e = self._selected_entity_designator.resolve()

        e_type = random.choice(["item", "object", "trash"])

        self._robot.speech.speak(random.choice(self._sentences) % e_type, block=False)

        return "done"


class SelfCleanup(smach.StateMachine):
    def __init__(self, robot, selected_entity_designator, location_id, segment_area, known_types):

        smach.StateMachine.__init__(self, outcomes=['done','failed'])

        trash_place_pose = dropPoseDesignator(robot, 0.6, "drop_pose")

        with self:

            smach.StateMachine.add("SPEAK", Speak(robot, selected_entity_designator, location_id, segment_area),
                                   transitions={"done": "GRAB"})

            smach.StateMachine.add("GRAB",
                                   robot_smach_states.Grab(robot, selected_entity_designator,
                                                           UnoccupiedArmDesignator(robot.arms,
                                                                                   robot.rightArm,
                                                                                   name="empty_arm_designator")),
                                   transitions={"done": "SAY_GRAB_SUCCESS", "failed": "SAY_GRAB_FAILED"})

            smach.StateMachine.add('SAY_GRAB_SUCCESS',
                                   robot_smach_states.Say(robot, ["Now I am going to toss the item in the trashbin",
                                                                  "Let's clean up this object",
                                                                  "Away with this garbage",
                                                                  "Everything will be cleaned"], block=False),
                                   transitions={"spoken": "DETERMINE_PLACE_LOCATION"})

            smach.StateMachine.add('SAY_GRAB_FAILED',
                                   robot_smach_states.Say(robot, ["I could not grab the item.",
                                                                  "I failed to grasp the item",
                                                                  "I cannot reach the item",
                                                                  "Item grab failed"], block=False),
                                   transitions={"spoken": "failed"})

            smach.StateMachine.add('CHECK_ARM_FREE', ArmFree(robot), transitions={"yes": "done", "no": "CHECK_ARM_OCCUPIED"})

            smach.StateMachine.add('CHECK_ARM_OCCUPIED', ArmOccupied(robot), transitions={"yes": "DETERMINE_PLACE_LOCATION", "no": "done"})

            smach.StateMachine.add('DETERMINE_PLACE_LOCATION', DetermineCleanupLocation(robot, selected_entity_designator, known_types),
                                   transitions={"trashbin" : "NAVIGATE_TO_TRASH", "other" : "PLACE_TO_STORE", "failed" : "NAVIGATE_TO_TRASH"})

            smach.StateMachine.add('NAVIGATE_TO_TRASH',
                                   robot_smach_states.NavigateToPlace(robot,
                                                            trash_place_pose,
                                                            OccupiedArmDesignator(robot.arms,
                                                                                  robot.rightArm,
                                                                                  name="occupied_arm_designator")),
                                   transitions={"arrived": "PLACE_IN_TRASH", "unreachable": "SAY_PLACE_FAILED", "goal_not_defined" : "SAY_PLACE_FAILED"})

            smach.StateMachine.add('PLACE_IN_TRASH',
                                   robot_smach_states.Place(robot, 
                                                            selected_entity_designator, 
                                                            trash_place_pose,
                                                            OccupiedArmDesignator(robot.arms,
                                                                                  robot.rightArm,
                                                                                  name="occupied_arm_designator")),
                                   transitions={"done": "SAY_PLACE_SUCCESS", "failed": "SAY_PLACE_FAILED"})

            smach.StateMachine.add('PLACE_TO_STORE',
                                    robot_smach_states.Place(robot,
                                                             selected_entity_designator,
                                                             "dinner_table",
                                                             OccupiedArmDesignator(robot.arms,
                                                                                   robot.rightArm,
                                                                                   name="occupied_arm_designator"),
                                                             "on_top_of"),
                                    transitions = {"done": "SAY_PLACE_SUCCESS", "failed": "SAY_PLACE_FAILED"})

            smach.StateMachine.add('SAY_PLACE_SUCCESS',
                                   robot_smach_states.Say(robot, ["Bye bye!",
                                                                  "Yeah!",
                                                                  "Successfully disposed the item",
                                                                  "Another score for AMIGO"], block=False),
                                   transitions={"spoken": "CHECK_ARM_OCCUPIED"})

            smach.StateMachine.add('SAY_PLACE_FAILED',
                                   robot_smach_states.Say(robot, ["I could not cleanup the item.",
                                                                  "I cannot put the item in the trashbin",
                                                                  "Item cleanup failed"], block=False),
                                   transitions={"spoken": "CHECK_ARM_OCCUPIED"})
