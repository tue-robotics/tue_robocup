import random

# ROS
import rospy
import smach

# TU/e
import robot_skills
import robot_smach_states as states
from robot_smach_states.human_interaction.give_directions import GiveDirections
from robot_smach_states.navigation.guidance import GuideToSymbolic
import robot_smach_states.util.designators as ds

from robocup_knowledge import load_knowledge

from hmi import HMIResult


knowledge = load_knowledge("challenge_where_is_this")


class EntityFromHmiResults(ds.Designator):
    """ Designator to pick the closest item on top of the table to grab. This is used for testing

    """
    def __init__(self, robot, hmi_result_des, parse=True):
        """ Constructor

        :param robot: robot object
        :param hmi_result_des:
        """
        super(EntityFromHmiResults, self).__init__(resolve_type=robot_skills.util.entity.Entity)

        self._robot = robot
        self._hmi_result_des = hmi_result_des
        self.parse = parse

    def _resolve(self):
        """ Resolves

        :return: entity in the <area_description> of the <surface_designator> that is closest to the robot
        """
        sem = self._hmi_result_des.resolve().semantics
        entity_id = sem["target-location"]["id"]

        entities = self._robot.ed.get_entities(id=entity_id, parse=self.parse)
        if entities:
            return entities[0]
        else:
            return None


class SayWaitDes(smach.StateMachine):
    def __init__(self, robot, entity_des):
        """ Constructor
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded"])

        with self:
            self.text_des = ds.VariableDesignator(resolve_type=str).writeable

            @smach.cb_interface(outcomes=["succeeded"])
            def get_text(userdata=None):
                entity = entity_des.resolve()
                entity_id = None
                if entity:
                    entity_id = entity.id
                if entity_id:
                    text = ["Let me think how to get to the {}".format(entity_id),
                            "I will now determine the best route to the {}".format(entity_id)]
                else:
                    text = ["Let me think how to get to there", "I will now determine the best route"]

                self.text_des.write(random.choice(text))
                return "succeeded"

            smach.StateMachine.add("GET_TEXT",
                                   smach.CBState(get_text),
                                   transitions={'succeeded': 'SAY_TEXT'})

            smach.StateMachine.add("SAY_TEXT",
                                   states.Say(robot, self.text_des, block=True),
                                   transitions={'spoken': 'succeeded'})


class GuideToRoomOrObject(smach.StateMachine):
    def __init__(self, robot, entity_des):
        """ Constructor
        :param robot: robot object
        :param entity_des: designator resolving to a room or a piece of furniture
        """
        smach.StateMachine.__init__(
            self, outcomes=['arrived', 'unreachable', 'goal_not_defined', 'lost_operator', 'preempted'])

        with self:
            @smach.cb_interface(outcomes=["room", "object"])
            def determine_type(userdata=None):
                entity = entity_des.resolve()
                entity_type = entity.type
                if entity_type == 'room':
                    return 'room'
                else:
                    return 'object'

            smach.StateMachine.add('DETERMINE_TYPE',
                                   smach.CBState(determine_type),
                                   transitions={'room': 'GUIDE_TO_ROOM',
                                                'object': 'GUIDE_TO_FURNITURE'})

            smach.StateMachine.add('GUIDE_TO_ROOM',
                                   GuideToSymbolic(robot, {entity_des: 'in'}, entity_des),
                                   transitions={'arrived': 'arrived',
                                                'unreachable': 'unreachable',
                                                'goal_not_defined': 'goal_not_defined',
                                                'lost_operator': 'lost_operator',
                                                'preempted': 'preempted'})

            smach.StateMachine.add('GUIDE_TO_FURNITURE',
                                   GuideToSymbolic(robot, {entity_des: 'in_front_of'}, entity_des),
                                   transitions={'arrived': 'arrived',
                                                'unreachable': 'unreachable',
                                                'goal_not_defined': 'goal_not_defined',
                                                'lost_operator': 'lost_operator',
                                                'preempted': 'preempted'})


class InformMachine(smach.StateMachine):
    def __init__(self, robot):
        """ Constructor
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            self.spec_des = ds.Designator(knowledge.location_grammar)
            self.answer_des = ds.VariableDesignator(resolve_type=HMIResult)
            self.entity_des = EntityFromHmiResults(robot, self.answer_des)

            smach.StateMachine.add("ANNOUNCE_ITEM",
                                   states.Say(robot, "Hello, my name is {}. Please call me by my name. Talk loudly into my microphone and wait for the ping".
                                              format(robot.robot_name), block=True),
                                   transitions={'spoken': 'WAIT_TO_BE_CALLED'})

            smach.StateMachine.add('WAIT_TO_BE_CALLED',
                                   states.HearOptions(robot, ["{}".format(robot.robot_name)], rospy.Duration(10)),
                                   transitions={'{}'.format(robot.robot_name): "INSTRUCT",
                                                'no_result': 'ANNOUNCE_ITEM'})

            smach.StateMachine.add("INSTRUCT",
                                   states.Say(robot,
                                              ["Please tell me where you would like to go. Talk loudly into my microphone and wait for the ping",
                                               "Where do you want to go? Talk loudly into my microphone and wait for the ping"]
                                              , block=True),
                                   transitions={'spoken': 'LISTEN_FOR_LOCATION'})

            smach.StateMachine.add('LISTEN_FOR_LOCATION',
                                   states.HearOptionsExtra(robot, self.spec_des, self.answer_des.writeable, rospy.Duration(15)),
                                   transitions={'heard': "INSTRUCT_FOR_WAIT",
                                                'no_result': 'failed'})

            smach.StateMachine.add("INSTRUCT_FOR_WAIT",
                                   SayWaitDes(robot, self.entity_des),
                                   transitions={'succeeded': 'GIVE_DIRECTIONS'})

            smach.StateMachine.add('GIVE_DIRECTIONS',
                                   GiveDirections(robot, self.entity_des),
                                   transitions={'succeeded': "GUIDE_OPERATOR",
                                                'failed': 'failed'})

            smach.StateMachine.add('GUIDE_OPERATOR',
                                   GuideToRoomOrObject(robot, self.entity_des),
                                   transitions={'arrived': 'SUCCESS',
                                                'unreachable': 'FAILURE',
                                                'goal_not_defined': 'FAILURE',
                                                'lost_operator': 'FAILURE',
                                                'preempted': 'failed'})

            smach.StateMachine.add("SUCCESS",
                                   states.Say(robot,
                                              ["We have arrived"]
                                              , block=True),
                                   transitions={'spoken': 'succeeded'})

            smach.StateMachine.add("FAILURE",
                                   states.Say(robot,
                                              ["I have failed you."]
                                              , block=True),
                                   transitions={'spoken': 'failed'})
