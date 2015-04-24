import smach, rospy, sys
from robot_smach_states.util.startup import startup
import robot_smach_states as states

import threading
import time

class FollowOperatorAndStoreWaypoints(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])

        self._robot = robot
        self._speech_recognition_thread = threading.Thread(target=self._speech_recognition_thread_function, args=())
        self._speech_recognition_result = None

    def _speech_recognition_thread_function(self):
        for i in range(0,10):
            print "test"
            time.sleep(1)

        self._speech_recognition_result = "banana"
 
    def execute(self, userdata):
        while True:

            # Check speech result
            if self._speech_recognition_result:
                print "speech result: " + self._speech_recognition_result

                self._speech_recognition_result = None

            # (Re)Start the speech recognition thread if not running 
            if not self._speech_recognition_thread.isAlive():
                self._speech_recognition_thread.start()

            time.sleep(0.1)

        self._speech_recognition_thread.join()


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['done', 'aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'FOLLOW_OPERATOR_AND_STORE_WAYPOINTS',
                                                'abort':'aborted'})
        smach.StateMachine.add('FOLLOW_OPERATOR_AND_STORE_WAYPOINTS', FollowOperatorAndStoreWaypoints(robot), transitions={ 'done' :'done'})
    
    return sm


# TEST
if __name__ == '__main__':
    rospy.init_node('follow_operator')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[FOLLOW_OPERATOR] Please provide robot name as argument."
        exit(1)

    startup(setup_statemachine, robot_name=robot_name)
