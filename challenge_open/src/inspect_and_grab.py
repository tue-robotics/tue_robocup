import rospy
from robot_skills.util.robot_constructor import robot_constructor
from robot_skills.classification_result import ClassificationResult
from robot_skills.util.entity import Entity

import smach
import robot_smach_states as states
from robot_smach_states.util.designators import UnoccupiedArmDesignator, VariableDesignator, EdEntityDesignator, Designator

class SelectEasiestGraspEntity(smach.State):
    def __init__(self, robot, classification_result_designator, grasp_entity_designator):
        smach.State.__init__(self, outcomes=['selected', 'failed'])
        self._robot = robot
        self._classification_result_designator = classification_result_designator
        self._grasp_entity_designator = grasp_entity_designator

    def execute(self, userdata=None):
        classification_results = self._classification_result_designator.resolve()
        smallest_distance = 100
        closest_entity = None
        for res in classification_results:
            e = self._robot.ed.get_entity(id=res.id)
            pose = e.pose.projectToFrame(self._robot.robot_name+"/base_link",
                                         self._robot.tf_listener)
            vs = pose.extractVectorStamped()
            distance = vs.vector.Norm()
            if distance < smallest_distance:
                smallest_distance = distance
                closest_entity = e

        if closest_entity:
            self._grasp_entity_designator.write(closest_entity)
            return 'selected'
        else:
            return 'failed'

class InspectAndGrab(smach.StateMachine):
    """ Smach state machine to inspect a given entity and grab something from its on_top_of area
    """
    def __init__(self, robot, supporting_entity_designator, arm_designator):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'inspect_failed', 'grasp_failed'])
        self._robot = robot
        self._supporting_entity_designator = supporting_entity_designator
        self._arm_designator = arm_designator

        self._classification_result_designator = VariableDesignator([], resolve_type=[ClassificationResult])

        self._grasp_entity_designator = VariableDesignator(resolve_type=Entity)

        with self:
            smach.StateMachine.add('INSPECT',
                                   states.Inspect(robot=self._robot,
                                                  entityDes=self._supporting_entity_designator,
                                                  objectIDsDes=self._classification_result_designator,
                                                  searchArea="on_top_of",
                                                  navigation_area="in_front_of"),
                                   transitions={'done': 'SELECT_EASIEST_GRASP_ENTITY',
                                                'failed': 'inspect_failed'}
                                   )

            smach.StateMachine.add('SELECT_EASIEST_GRASP_ENTITY',
                                   SelectEasiestGraspEntity(
                                       robot=self._robot,
                                       classification_result_designator=self._classification_result_designator,
                                       grasp_entity_designator=self._grasp_entity_designator.writeable
                                       ),
                                   transitions={'selected': 'GRAB',
                                                'failed':   'inspect_failed'}
                                   )

            smach.StateMachine.add('GRAB',
                                   states.Grab(robot=self._robot,
                                               item=self._grasp_entity_designator,
                                               arm=self._arm_designator
                                               ),
                                   transitions={'done': 'succeeded',
                                                'failed': 'grasp_failed'})

def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])

    des = EdEntityDesignator(robot=robot, id='left_planks')
    arm_des = UnoccupiedArmDesignator(robot.arms, robot.leftArm)

    with sm:
        # Start challenge via StartChallengeRobust, skipped atm
        smach.StateMachine.add("INSPECT_AND_GRAB",
                               InspectAndGrab(robot, supporting_entity_designator=des,
                                              arm_designator=arm_des),
                               transitions={"succeeded": "Done",
                                            "inspect_failed": "Aborted",
                                            "grasp_failed": "Aborted"})

    return sm

if __name__ == '__main__':
    rospy.init_node('test_inspect_and_grab')

    amigo = robot_constructor('amigo')
    sm = setup_statemachine(amigo)

    sm.execute()
