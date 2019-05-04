# ROS
import rospy
import smach

# TU/e
import robot_skills
import robot_smach_states as states
from robot_smach_states.human_interaction.give_directions import GiveDirections
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
                                   states.Say(robot, "Hello, my name is {}. Please call me by my name for guiding".
                                              format(robot.robot_name), block=False),
                                   transitions={'spoken': 'WAIT_TO_BE_CALLED'})

            smach.StateMachine.add('WAIT_TO_BE_CALLED',
                                   states.HearOptions(robot, ["{}".format(robot.robot_name)], rospy.Duration(20)),
                                   transitions={'{}'.format(robot.robot_name): "INSTRUCT",
                                                'no_result': 'ANNOUNCE_ITEM'})

            smach.StateMachine.add("INSTRUCT",
                                   states.Say(robot,
                                              ["Please tell me where you would like to go",
                                               "Where do you want to go?"]
                                              , block=False),
                                   transitions={'spoken': 'LISTEN_FOR_LOCATION'})

            smach.StateMachine.add('LISTEN_FOR_LOCATION',
                                   states.HearOptionsExtra(robot, self.spec_des, self.answer_des.writeable, rospy.Duration(20)),
                                   transitions={'heard': "GIVE_DIRECTIONS",
                                                'no_result': 'failed'})

            # @smach.cb_interface(outcomes=["succeeded"])
            # def update_ds(userdata=None):
            #     sem = self.answer_des.resolve().semantics
            #     print(sem["target-location"]["id"])
            #     self.entity_des = ds.EntityByIdDesignator(robot, id=sem["target-location"]["id"])
            #
            #     return "succeeded"
            #
            # smach.StateMachine.add("UPDATE_DS",
            #                        smach.CBState(update_ds),
            #                        transitions={'succeeded': 'GIVE_DIRECTIONS'})

            smach.StateMachine.add('GIVE_DIRECTIONS',
                                   GiveDirections(robot, self.entity_des),
                                   transitions={'succeeded': "succeeded",
                                                'failed': 'failed'})
