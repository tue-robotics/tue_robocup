# ROS
import smach
import rospy

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds

from robot_skills.classification_result import ClassificationResult
from robocup_knowledge import knowledge_loader

# Challenge serving drinks
from .serve_one_drink import ServeOneDrink
from .sd_states import AskAvailability

# Knowledge
challenge_knowledge = knowledge_loader.load_knowledge("challenge_serving_drinks")
common_knowledge = knowledge_loader.load_knowledge("common")


class CheckInspect(smach.State):
    def __init__(self, designator, *resolve_types):
        super(CheckInspect, self).__init__(outcomes=["true", "false"])
        ds.check_type(designator, *resolve_types)
        self.designator = designator

    def execute(self, userdata=None):
        val = self.designator.resolve() if hasattr(self.designator, "resolve") else self.designator
        if val:
            return "true"
        else:
            return "false"


class UpdateUnavailableDrinkList(smach.State):
    def __init__(self, unavailable_drink_list_designator, drink_to_add_designator):
        super(UpdateUnavailableDrinkList, self).__init__(outcomes=["done", "failed"])
        ds.is_writeable(unavailable_drink_list_designator)
        ds.check_type(unavailable_drink_list_designator, [str])
        ds.check_type(drink_to_add_designator, str)

        self.unavailable_drink_list_designator = unavailable_drink_list_designator
        self.drink_to_add_designator = drink_to_add_designator

    def execute(self, userdata=None):
        drink_to_add = self.drink_to_add_designator.resolve() if hasattr(self.drink_to_add_designator, "resolve") else self.drink_to_add_designator

        if not drink_to_add:
            return "failed"
        else:
            unavailable_drink_list = self.unavailable_drink_list_designator.resolve() if hasattr(self.unavailable_drink_list_designator, "resolve") else self.unavailable_drink_list_designator
            if drink_to_add not in unavailable_drink_list:
                unavailable_drink_list.append(drink_to_add)
                self.unavailable_drink_list_designator.write(unavailable_drink_list)

            return "done"


class IdentifyUnavailableDrinkFromRecognitions(smach.State):
    def __init__(self, objects, classification_list_designator, unavailable_drink_designator, max_unavailable_drinks):
        # TODO: Change unavailable_drink_designator resolve type to list to make the implementation more generalized as
        # this state becomes a bug when max_unavailable_drinks > 1
        super(IdentifyUnavailableDrinkFromRecognitions, self).__init__(outcomes=["done", "failed"])
        ds.is_writeable(unavailable_drink_designator)
        ds.check_type(unavailable_drink_designator, str)
        ds.check_type(classification_list_designator, [ClassificationResult])

        self._drinks_list = [obj["name"] for obj in objects if obj["category"] == "drink"]
        self._classification_list_designator = classification_list_designator
        self._unavailable_drink_designator = unavailable_drink_designator
        self._max_unavailable_drinks = max_unavailable_drinks

    def execute(self, userdata=None):
        classification_list = self._classification_list_designator.resolve() if hasattr(self._classification_list_designator, "resolve") else self._classification_list_designator
        classification_list = set([classification.type for classification in classification_list])

        if not classification_list:
            rospy.loginfo("Classification list empty")
            return "failed"

        unavailable_drinks_count = 0
        unavailable_drink = []
        for drink in self._drinks_list:
            if drink not in classification_list:
                # TODO: Convert unavailable_drink to list because this becomes a bug if max_unavailable_drinks > 1
                unavailable_drink.append(drink)
                rospy.loginfo("Found unavailable drink: {}".format(drink))
                unavailable_drinks_count += 1

        if unavailable_drinks_count == self._max_unavailable_drinks + 1:
            for d in unavailable_drink:
                if d is not "tea_bag":
                    self._unavailable_drink_designator.write(d)
            return "done"
        elif (unavailable_drinks_count == self._max_unavailable_drinks) and (unavailable_drink[0] == "tea_bag"):
            self._unavailable_drink_designator.write("tea bag")
            return "done"
        else:
            # Even if no unavailable drink is found, return failed
            rospy.loginfo("Unavailable drinks count: {}".format(unavailable_drinks_count))
            return "failed"


class ServingDrinks(smach.StateMachine):
    """
    State machine for 'Serving Drinks' challenge.
    """

    def __init__(self, robot):
        # type: (Robot) -> str
        """
        Initialization method

        :param robot: robot api object
        """

        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        # Designators
        bar_designator = ds.EdEntityDesignator(robot=robot, id=challenge_knowledge.bar_id, name='bar_des')
        room_designator = ds.EdEntityDesignator(robot=robot, id=challenge_knowledge.room_id, name='room_des')

        objects_list_des = ds.VariableDesignator(resolve_type=[ClassificationResult], name='objects_list_des')
        unav_drink_des = ds.VariableDesignator(resolve_type=str, name='unav_drink_str_des')

        hacky_arm_des = ds.VariableDesignator(initial_value=robot.get_arm(), name='hacky_arm')

        with self:
            smach.StateMachine.add("INITIALIZE",
                                   states.Initialize(robot=robot),
                                   transitions={"initialized": "INITIAL_POSE",
                                                "abort": "aborted"})

            smach.StateMachine.add("INITIAL_POSE",
                                   states.SetInitialPose(robot,
                                                         challenge_knowledge.starting_point),
                                   transitions={"done": "INSPECT_BAR",
                                                "preempted": "aborted",
                                                "error": "INSPECT_BAR"})

            # Inspect bar and store the list of available drinks
            smach.StateMachine.add("INSPECT_BAR",
                                   states.Inspect(robot=robot,
                                                  entityDes=bar_designator,
                                                  navigation_area="in_front_of",
                                                  objectIDsDes=objects_list_des),
                                   transitions={"done": "INSPECT_FALLBACK", #TODO: Change to CHECK_INSPECT_RESULT after RWC2019
                                                "failed": "INSPECT_FALLBACK"})

            smach.StateMachine.add("CHECK_INSPECT_RESULT",
                                   CheckInspect(objects_list_des,
                                                [ClassificationResult]),
                                   transitions={"true": "IDENTIFY_UNAVAILABLE_DRINK",
                                                "false": "INSPECT_FALLBACK"})

            smach.StateMachine.add("IDENTIFY_UNAVAILABLE_DRINK",
                                   IdentifyUnavailableDrinkFromRecognitions(objects=common_knowledge.objects,
                                                                            classification_list_designator=objects_list_des,
                                                                            unavailable_drink_designator=unav_drink_des.writeable,
                                                                            max_unavailable_drinks=challenge_knowledge.MAX_UNAVAILABLE_DRINKS),
                                   transitions={"done": "NAVIGATE_TO_ROOM",
                                                "failed": "INSPECT_FALLBACK"})

            # Inspect fallback - ask the bartender which drink is unavailable and store the unavailable drink
            smach.StateMachine.add("INSPECT_FALLBACK",
                                   AskAvailability(robot=robot,
                                                   unavailable_drink_designator=unav_drink_des.writeable,
                                                   objects=common_knowledge.objects),
                                   transitions={"succeeded": "RESET_ROBOT",
                                                "failed": "RESET_ROBOT"})

            smach.StateMachine.add("RESET_ROBOT",
                                   states.ArmToJointConfig(robot=robot,
                                                           arm_designator=hacky_arm_des,
                                                           configuration="reset"),
                                   transitions={'succeeded': "NAVIGATE_TO_ROOM",
                                                'failed': "NAVIGATE_TO_ROOM"})

            # Navigate to the predefined room
            smach.StateMachine.add("NAVIGATE_TO_ROOM",
                                   states.NavigateToRoom(robot=robot, entity_designator_room=room_designator),
                                   transitions={"arrived": "SAY_HI",
                                                "unreachable": "SAY_HI",
                                                "goal_not_defined": "aborted"})

            smach.StateMachine.add("SAY_HI",
                                   states.Say(robot, "Hi, I am {}. I'll be your waiter today".format(robot.robot_name)),
                                   transitions={"spoken": "SERVE_DRINK_1"})

            # Explicitly add a new state for each drink, i.e., don't use a range iterator to make sure a new state
            # is constructed every time
            for idx in range(1, challenge_knowledge.NR_DRINKS + 1):
                next_state = "SERVE_DRINK_{}".format(idx + 1) if idx < challenge_knowledge.NR_DRINKS else "SAY_DONE"

                smach.StateMachine.add("SERVE_DRINK_{}".format(idx),
                                       ServeOneDrink(robot=robot,
                                                     bar_designator=bar_designator,
                                                     room_id=challenge_knowledge.room_id,
                                                     room_designator=room_designator,
                                                     objects_list_des=objects_list_des,
                                                     unav_drink_des=unav_drink_des,
                                                     name_options=common_knowledge.names,
                                                     objects=common_knowledge.objects),
                                       transitions={"succeeded": next_state,
                                                    "failed": next_state,
                                                    "aborted": next_state})

            smach.StateMachine.add("SAY_DONE",
                                   states.Say(robot, "My job here is done. Enjoy your day and see you next time"),
                                   transitions={"spoken": "succeeded"})
