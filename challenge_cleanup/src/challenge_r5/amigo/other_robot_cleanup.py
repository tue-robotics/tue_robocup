import smach
import robot_smach_states
import sys
from std_msgs.msg import String
import rospy

from robot_smach_states.util.designators import EntityByIdDesignator
from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('r5cop_demo')

other_robot_name = "x-80sv"

class ContactOtherRobot(smach.State):
    def __init__(self, robot, selected_entity_designator):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self._robot = robot
        self._pub = rospy.Publisher("/%s/trigger" % other_robot_name, String, queue_size=1)
        self._selected_entity_designator = selected_entity_designator

    def execute(self, userdata):

        e = self._selected_entity_designator.resolve()

        if not e:
            rospy.logerr("For some reason, the entity could not be resolved, this should not happen!")
            self._robot.speech.speak("I failed to resolve the entity")
            return "failed"

        self._pub.publish(String(data=e.id))

        self._robot.speech.speak("I have contacted %s, waiting for continue trigger" % other_robot_name, block=False)

        return "done"


class OtherRobotCleanup(smach.StateMachine):
    def __init__(self, robot, selected_entity_designator, location_id, segment_area):

        smach.StateMachine.__init__(self, outcomes=['done'])

        sentences = ["%s, please clean this object %s the %s" % (other_robot_name, segment_area, location_id),
                     "%s, can you clean the trash %s the %s?" % (other_robot_name, segment_area, location_id),
                     "Can another robot clean the garbage %s the %s?" % (segment_area, location_id)]
        with self:

            smach.StateMachine.add('SAY_OTHER_ROBOT_CLEANUP',
                                   robot_smach_states.Say(robot, sentences, block=False),
                                   transitions={"spoken": "NAVIGATE_TO_INITIAL"})

            smach.StateMachine.add( "NAVIGATE_TO_INITIAL",
                                    robot_smach_states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=challenge_knowledge.starting_point,), radius=0.05),
                                    transitions={   'arrived'           : 'CONTACT_OTHER_ROBOT',
                                                    'unreachable'       : 'CONTACT_OTHER_ROBOT',
                                                    'goal_not_defined'  : 'CONTACT_OTHER_ROBOT'})

            smach.StateMachine.add('CONTACT_OTHER_ROBOT',
                                   ContactOtherRobot(robot, selected_entity_designator),
                                   transitions={"done": "WAIT_FOR_TRIGGER", "failed": "SAY_FAILED"})

            smach.StateMachine.add('WAIT_FOR_TRIGGER',
                                   robot_smach_states.WaitForTrigger(robot, ["continue", "gpsr"], "/amigo/trigger"),
                                   transitions={"continue": "SAY_DONE", "gpsr": "SAY_FAILED", "preempted" : "SAY_FAILED"})

            smach.StateMachine.add('SAY_DONE',
                                   robot_smach_states.Say(robot, ["Thanks for cleaning", "Thank you", "You are the best"], block=True),
                                   transitions={"spoken": "done"})

            smach.StateMachine.add('SAY_FAILED',
                                   robot_smach_states.Say(robot, ["Too bad then, we will just leave that trash there"], block=True),
                                   transitions={"spoken": "done"})
