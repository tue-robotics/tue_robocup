#!/usr/bin/env python
import roslib; roslib.load_manifest('amigo_demo')
import rospy
import os
import random

from robot_skills.amigo import Amigo
from robot_skills.reasoner import Conjunction, Compound
from std_msgs.msg import String

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
    import smach
    import robot_smach_states as states
    import object_msgs.msg

    #Copied from RDO finale
    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    #sm.userdata.target = object_msgs.msg.ExecutionTarget() #drink is a global var
    #sm.userdata.known_object_list = rospy.get_param("known_object_list")
    #sm.userdata.desired_objects = [obj for obj in sm.userdata.known_object_list]# if obj["class_label"]=="soup_pack"] #drink is a global var
    sm.userdata.operator_name = "operator"
    #sm.userdata.rate = 10 #first used in LOOK_FOR_DRINK. TODO:refactor this parameter out
    #sm.userdata.command = "" #First used in LOOK_FOR_DRINK
    #sm.userdata.dropoff_location = object_msgs.msg.ExecutionTarget(name=sm.userdata.target.category,class_label="location",ID=-2) #Also used in LOOK_FOR_DRINK.
    rospy.loginfo("Userdata set")

    robot.leftArm.reset_arm()
    robot.rightArm.reset_arm()
    robot.reasoner.reset()
    robot.reasoner.detach_all_from_gripper("/grippoint_left")
    robot.reasoner.detach_all_from_gripper("/grippoint_right")

    with sm:
        smach.StateMachine.add('ANNOUNCE_LOOK_FOR_DRINK',
                                states.Say(robot, "I wonder what objects I can see here!", block=False),
                                transitions={'spoken':'LOOK'})

        
        #smach.StateMachine.add('LOOK_FOR_DRINK', 
        #                               states.Look_for_objects(robot),
        #                               transitions={'looking':'LOOK_FOR_DRINK',
        #                                            'object_found':'GRAB',
        #                                            'no_object_found':'SAY_NO_DRINK',
        #                                            'abort':'Aborted'})

        
        query_lookat = Conjunction( Compound("current_exploration_target", "Target"),
                                    Compound("point_of_interest", "Target", Compound("point_3d", "X", "Y", "Z")))

        
        rospy.logerr("TODO Loy/Janno/Sjoerd: make query use current_object")
        query_object = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z"))

        smach.StateMachine.add('LOOK',
                                states.LookForObjectsAtPoint(robot, query_object, robot.base.point(0.75,0,0.65,frame_id="/base_link",stamped=True)),
                                transitions={   'looking':'LOOK',
                                                'object_found':'GRAB',
                                                'no_object_found':'SAY_NO_DRINK',
                                                'abort':'Aborted'})


        #smach.StateMachine.add('GRAB',
        #                       states.GrabMachine(robot.leftArm,robot),
        #                       transitions={'succeeded':'CARRYING_POSE',
        #                                    'failed':'REPORT_FAILED'})

        query_grabpoint = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z"))
        smach.StateMachine.add('GRAB',
                        states.GrabMachine(robot.leftArm, robot, query_grabpoint),
                        transitions={   'succeeded':'CARRYING_POSE',
                                        'failed':'REPORT_FAILED' })

        smach.StateMachine.add('CARRYING_POSE',
                               states.Carrying_pose(robot.leftArm, robot),
                               transitions={'succeeded':'SAY_SUCCEEDED',
                                            'failed':'Done'})

        smach.StateMachine.add('SAY_NO_DRINK',
                               states.Say_generated(robot,
                                                    lambda userdata: "I can't find the drink I am looking for. You will have to get one yourself.".format(userdata.operator_name),
                                                    input_keys=['operator_name']),
                               transitions={'spoken':'Done'})

        smach.StateMachine.add('REPORT_FAILED',
                               states.Say_generated(robot,
                                                    lambda userdata: "I am sorry I could not get your drink.".format(userdata.operator_name),
                                                    input_keys=['operator_name']),
                               transitions={'spoken':'Done'})

        smach.StateMachine.add('SAY_SUCCEEDED',
                               states.Say_generated(robot,
                                                    lambda userdata: "Where would you like this?".format(userdata.operator_name),
                                                    input_keys=['operator_name']),
                               transitions={'spoken':'Done'})

    rospy.loginfo("State machine set up, start execution...")
    #import pdb; pdb.set_trace()
    result = sm.execute()
    rospy.loginfo("State machine executed. Result: {0}".format(result))
# sequences = dict()
# #An action consists of a pose and a waittime: (POSE, waittime)
# sequences["WAVE"] = [("GOAL",0)]


def main():
    keyboard_cmd_listener = rospy.Subscriber("key_commands",String,callback_keyboard_cmd)
    speech_listener = rospy.Subscriber("/speech/output",String,callback_speech)
    rospy.init_node('robot_smach_states')

    print mesg

    print_keys()

    global robot
    #~ if len(sys.argv) > 1:
        #~ from test_tools.build_amigo import build_amigo
        #~ robot = build_amigo(fake=['base','arms','perception','head', 'worldmodel'])
    #~ else:
    robot = Amigo() #dontInclude=['perception'] is needed for grabbing items
    robot.lights.set_color(0.5, 1, 0)
    #import pdb; pdb.set_trace()
    rospy.spin()

if __name__ == "__main__":

    main()
