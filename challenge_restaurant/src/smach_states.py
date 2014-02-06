#!/usr/bin/env python
import roslib; roslib.load_manifest('amigo_demo')
import rospy
import os
import random

from robot_skills.amigo import Amigo
from robot_skills.reasoner import Conjunction, Compound

import smach
import robot_smach_states as states

from std_msgs.msg import String
import challenge_restaurant.srv

from math import sin


#This node publishes integer values to the 'key_commands' topic

mesg = """
This node receives values from the 'key_commands' topic.

The commands will by default for the left arm.
Toggle to right and back again using TAB.
That key even has handy arrows on it!

spacebar to quit.
"""


def grab_item(robot):
    rospy.loginfo("Starting to grab an item")
    
    import object_msgs.msg

    #Copied from RDO finale
    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=['Succeeded','Failed','Aborted'])

    with sm:
        query_grabpoint = Conjunction(  Compound("current_object", "ObjectID"),
                                        Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))
        #query_grabpoint = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z"))
        smach.StateMachine.add('GRAB',
                        states.GrabMachine(robot.leftArm, robot, query_grabpoint),
                        transitions={   'succeeded':'CARRYING_POSE',
                                        'failed':'REPORT_FAILED' })

        smach.StateMachine.add('CARRYING_POSE',
                               states.Carrying_pose(robot.leftArm, robot),
                               transitions={'succeeded':'SAY_SUCCEEDED',
                                            'failed':'Failed'})


        smach.StateMachine.add('REPORT_FAILED',
                               states.Say_generated(robot,
                                                    lambda userdata: "I am sorry I could not get your object.".format(userdata.operator_name),
                                                    input_keys=['operator_name']),
                               transitions={'spoken':'Failed'})

        smach.StateMachine.add('SAY_SUCCEEDED',
                               states.Say_generated(robot,
                                                    lambda userdata: "Where would you like this?",
                                                    input_keys=['operator_name']),
                               transitions={'spoken':'Succeeded'})

    rospy.loginfo("State machine set up, start execution...")
    #import pdb; pdb.set_trace()
    result = sm.execute()
    rospy.loginfo("State machine executed. Result: {0}".format(result))

    return result

def lookat_objects_roi(robot):
    rospy.loginfo("Starting to look at an ROI")
    
    rospy.loginfo("Test (smach_states): robot.spindle.upper_limit = {0}".format(robot.spindle.upper_limit))

    robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
    robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))

    robot.reasoner.assertz(Compound("challenge", "smach_state_server"))

    sm = smach.StateMachine(outcomes=['Succeeded','Failed','Aborted'])

    with sm:
        query_lookat = Compound("point_of_interest", "large_table_1", Compound("point_3d", "X", "Y", "Z"))

        #Make sure the object we're dealing with isn't already disposed (i.e. handled for cleanup)
        #After cleaning the object up/disposing it, 
        #MARK_DISPOSED asserts disposed(current_objectID)
        query_object = Conjunction(
                            Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")),
                            Compound("not", Compound("disposed", "ObjectID")))

        smach.StateMachine.add("LOOK_AT_OBJECT_ROI",
                                states.LookForObjectsAtROI(robot, query_lookat, query_object, maxdist=0.9, modules=["object_recognition"], waittime=2.5),
                                transitions={   'looking'           : 'LOOK_AT_OBJECT_ROI',
                                                'object_found'      : 'Succeeded',
                                                'no_object_found'   : 'Failed',
                                                'abort'             : 'Aborted'})

    rospy.loginfo("State machine set up, start execution...")
    #import pdb; pdb.set_trace()
    result = sm.execute()
    rospy.loginfo("State machine executed. Result: {0}".format(result))

    return result


def process_request(request):
    response = challenge_restaurant.srv.SmachStatesResponse()
    if request.state == "grasp":
        response.outcome = grab_item(robot)
    elif request.state == "lookatroi":
        response.outcome = lookat_objects_roi(robot)
    else:
        rospy.logerr("The requested state is not (yet) implemented")
        response.outcome = "Aborted"

    rospy.loginfo("Response = {0}".format(response))
    return response

def main():
    
    server_ = rospy.Service('smach_states', challenge_restaurant.srv.SmachStates, process_request)

    rospy.init_node('robot_smach_states')

    global robot
    #~ if len(sys.argv) > 1:
        #~ from test_tools.build_amigo import build_amigo
        #~ robot = build_amigo(fake=['base','arms','perception','head', 'worldmodel'])
    #~ else:
    robot = Amigo() #dontInclude=['perception'] is needed for grabbing items
    #import pdb; pdb.set_trace()
    rospy.spin()

if __name__ == "__main__":

    main()
