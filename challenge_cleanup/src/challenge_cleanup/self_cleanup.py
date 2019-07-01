import smach
import rospy
import robot_smach_states
import random
import hmi

from robot_skills.util.kdl_conversions import FrameStamped
from robot_skills.util.entity import Entity
from robot_smach_states.util.designators import UnoccupiedArmDesignator, OccupiedArmDesignator, Designator, EntityByIdDesignator

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_cleanup')

from PyKDL import Frame

#ToDo: Location for trash depends on the room chosen: trash_bin (living) or trash_can (kitchen). Not handled yet.

class dropPoseDesignator(Designator):
    def __init__(self, robot, drop_height, name):
        super(dropPoseDesignator, self).__init__(resolve_type=FrameStamped, name=name)

        self._robot = robot
        self._drop_height = drop_height

    def _resolve(self):
        frame = None

        # Query ed
        #ToDo: Make this happen for the bin in the chosen room...

        try:
            frame = self._robot.ed.get_entity(id="trash_bin")._pose
        except:
            return None

        frame.p.z(self._drop_height)

        return FrameStamped(frame, "/map")

class storePlaceDesignator(Designator):
    def __init__(self, robot, name, selected_entity_designator):
        super(storePlaceDesignator, self).__init__(resolve_type=Entity, name=name)

        self._robot = robot
        self._selected_entity_designator = selected_entity_designator

    def _resolve(self):
        e = self._selected_entity_designator.resolve()

        item_category = challenge_knowledge.common.get_object_category(e.type)
        if item_category is not None:
            location, area_name = challenge_knowledge.common.get_object_category_location(item_category)
            entities = self._robot.ed.get_entities(id=location)
            if entities:
                return entities[0]
            else:
                return None
        else:
            rospy.logerr("Could not resolve the selected entity!")
            return None

class storeAreaDesignator(Designator):
    def __init__(self, robot, name, selected_entity_designator):
        super(storeAreaDesignator, self).__init__(resolve_type=str, name=name)

        self._robot = robot
        self._selected_entity_designator = selected_entity_designator

    def _resolve(self):
        e = self._selected_entity_designator.resolve()

        #return "on_top_of"
        item_category = challenge_knowledge.common.get_object_category(e.type)
        if item_category is not None:
            location, area_name = challenge_knowledge.common.get_object_category_location(item_category)
            return area_name
        else:
            rospy.logerr("Could not resolve the selected entity!")
            return None

class DetermineCleanupLocation(smach.State):

    #ToDo: Change this to let the operator decide where the object must be brought to

    def __init__(self, robot, selected_entity_designator):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._robot = robot
        self._selected_entity_designator = selected_entity_designator
        self._max_tries = 5

    def _confirm(self):
        cgrammar = """
            C[P] -> A[P]
            A['yes'] -> yes
            A['no'] -> no
            """
        try:
            speech_result = self._robot.hmi.query(description="Is this correct?", grammar="T[True] -> yes;"
                                                                                          "T[False] -> no", target="T")
        except hmi.TimeoutException:
            return False

        return speech_result.semantics

    def execute(self, userdata=None):
        selected_entity = self._selected_entity_designator.resolve()

        if not selected_entity:
            rospy.logerr("Could not resolve the selected entity!")
            return "failed"

        rospy.loginfo("The type of the entity is '%s'" % selected_entity.type)


        nr_tries = 0
        while nr_tries < self._max_tries and not rospy.is_shutdown():
            nr_tries += 1
            rospy.loginfo('nr_tries: %d', nr_tries)

            # self._robot.speech.speak("Where to should I bring this object?")
            count = 0


            try:
                speech_result = self._robot.hmi.query(description="Where to should I bring this object?",
                                                          grammar=challenge_knowledge.location_grammar, target="O")
                break
            except hmi.TimeoutException:
                if count < 5:
                    self._robot.speech.speak(random.choice(["I'm sorry, can you repeat",
                                                            "Please repeat your order, I didn't hear you",
                                                            "I didn't get your order, can you repeat it",
                                                            "Please speak up, as I didn't hear your order"]))
                else:
                    self._robot.speech.speak("I am sorry but I cannot understand you. I will quit now", block=False)
                    self._robot.head.cancel_goal()
                    return "failed"

            try:
                # Now: confirm
                if "beverage" in speech_result.semantics:
                    self._robot.speech.speak("I understood that you would like {}, "
                                            "is this correct?".format(speech_result.semantics['beverage']))
                elif "food1" in speech_result.semantics and "food2" in speech_result.semantics:
                    self._robot.speech.speak("I understood that you would like {} and {}, "
                                            "is this correct?".format(speech_result.semantics['food1'],
                                                                    speech_result.semantics['food2']))
            except:
                continue

            if self._confirm():
                # DO NOT ASSIGN self._orders OR OTHER STATES WILL NOT HAVE THE CORRECT REFERENCE
                for k, v in speech_result.semantics.iteritems():
                    self._orders[k] = v
                self._robot.head.cancel_goal()
                self._robot.speech.speak("Ok, I will get your order", block=False)
                return "succeeded"

        self._robot.speech.speak("I am sorry but I cannot understand you. I will quit now", block=False)
        self._robot.head.cancel_goal()
        return "failed"



        # if challenge_knowledge.common.is_known_object(selected_entity.type):
        #     return "other"
        # else:
        #     return "trashbin"

class ArmFree(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["yes", "no"])
        self._robot = robot

    def execute(self, userdata):
        d = UnoccupiedArmDesignator(self._robot, {}, name="empty_arm_designator2")
        if d.resolve():
            return "yes"
        return "no"

class ArmOccupied(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["yes", "no"])
        self._robot = robot

    def execute(self, userdata):
        d = OccupiedArmDesignator(self._robot, {}, name="empty_arm_designator2")
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
    """
    Grab the current object and:
    - Bring it to the predefined destination, or
    - Drop it in the designated waste bin
    """
    def __init__(self, robot, selected_entity_designator, location_id, segment_area):

        smach.StateMachine.__init__(self, outcomes=['done','failed'])

        trash_place_pose = dropPoseDesignator(robot, 0.6, "drop_pose")
        trash_designator = EntityByIdDesignator(robot, "trash_bin")
        item_store_entity = storePlaceDesignator(robot,
                                                 "store_entity",
                                                 selected_entity_designator)
        item_store_area = storePlaceDesignator(robot,
                                               "store_area",
                                               selected_entity_designator)

        with self:

            smach.StateMachine.add("SPEAK", Speak(robot, selected_entity_designator, location_id, segment_area),
                                   transitions={"done": "GRAB"})

            smach.StateMachine.add("GRAB",
                                   robot_smach_states.Grab(robot, selected_entity_designator,
                                                           UnoccupiedArmDesignator(robot,
                                                                                   {},
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

            smach.StateMachine.add('DETERMINE_PLACE_LOCATION', DetermineCleanupLocation(robot, selected_entity_designator),
                                   transitions={"succeede" : "INSPECT_TRASH", "failed" : "NAVIGATE_TO_TRASH"})

            # smach.StateMachine.add('NAVIGATE_TO_TRASH',
            #                        robot_smach_states.NavigateToPlace(robot,
            #                                                 trash_place_pose,
            #                                                 OccupiedArmDesignator(robot,
            #                                                                       {},
            #                                                                       name="occupied_arm_designator")),
            #                        transitions={"arrived": "PLACE_IN_TRASH", "unreachable": "SAY_PLACE_FAILED", "goal_not_defined" : "SAY_PLACE_FAILED"})
            #
            # smach.StateMachine.add('INSPECT_TRASH',
            #                        robot_smach_states.Inspect(robot,
            #                                                   trash_designator),
            #                        transitions={"done": "PLACE_IN_TRASH",
            #                                     "failed": "SAY_PLACE_FAILED"})
            #
            # smach.StateMachine.add('PLACE_IN_TRASH',
            #                        robot_smach_states.Place(robot,
            #                                                 selected_entity_designator,
            #                                                 trash_place_pose,
            #                                                 OccupiedArmDesignator(robot,
            #                                                                       {},
            #                                                                       name="occupied_arm_designator")),
            #                        transitions={"done": "SAY_PLACE_SUCCESS", "failed": "SAY_PLACE_FAILED"})
            #
            # smach.StateMachine.add('PLACE_TO_STORE',
            #                         robot_smach_states.Place(robot,
            #                                                  selected_entity_designator,
            #                                                  item_store_entity,
            #                                                  OccupiedArmDesignator(robot,
            #                                                                        {},
            #                                                                        name="occupied_arm_designator"),
            #                                                  "on_top_of"),
            #                         transitions = {"done": "SAY_PLACE_SUCCESS", "failed": "SAY_PLACE_FAILED"})
            #
            # smach.StateMachine.add('SAY_PLACE_SUCCESS',
            #                        robot_smach_states.Say(robot, ["Bye bye!",
            #                                                       "Yeah!",
            #                                                       "Successfully disposed the item",
            #                                                       "Another score for HERO"], block=False),
            #                        transitions={"spoken": "CHECK_ARM_OCCUPIED"})
            #
            # smach.StateMachine.add('SAY_PLACE_FAILED',
            #                        robot_smach_states.Say(robot, ["I could not cleanup the item.",
            #                                                       "I cannot put the item in the trashbin",
            #                                                       "Item cleanup failed"], block=False),
            #                        transitions={"spoken": "CHECK_ARM_OCCUPIED"})
