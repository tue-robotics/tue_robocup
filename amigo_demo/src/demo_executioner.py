#!/usr/bin/env python
import roslib; roslib.load_manifest('amigo_demo')
import rospy
import os
import random

import decorators as dec

from robot_skills.amigo import Amigo
from robot_skills.reasoner import Conjunction, Compound
import robot_skills.util.msg_constructors as msgs
from std_msgs.msg import String

from math import sin

from transporter import Transporter
from drive_to_person import DriveToClosestPerson

#This node publishes integer values to the 'key_commands' topic

mesg = """
This node receives values from the 'key_commands' topic.

The commands will by default for the left arm.
Toggle to right and back again using TAB.
That key even has handy arrows on it!

spacebar to quit.
"""

#In this script, poses, sentences and keys are decoupled.
#When you press a key, it is first checked if there is an action for that key, if it is not special in any way.
#Then, the associated pose and sentence is looked up, which are then performed, or spoken.
#Next: sequences of actions, both speech and poses.

# [q1 = shoulder: sidewards, q2 = shoulder: front, q3 = upper arm rotation , q4 = elbow, q5 = lower arm rotation, q6 = gripper up down, q7 = gripper left right ]
poses = dict()
poses["INIT"]                   = ("",  [ 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 ])
poses["ANGLE"]                  = ("",  [ 0.0000 , 0.0000 , 0.0000 , 1.5708 , 0.0000 , 0.0000 , 0.0000 ])
poses["DRIVE"]                  = ("",  [ 0.0000 , -0.1292 , 0.0000 , 0.7100 , 0.0000 , 0.0000 , 0.0000 ])
poses["HANDSHAKEUP"]            = ("4", [ 0.0000 , 0.0000 , 0.0230 , 1.9800 , 1.3900 , 0.2038 , 0.5100 ])
poses["HANDSHAKEDOWN"]          = ("5", [ 0.0000 , -0.0598 , 0.0000 , 1.2380 , 1.6220 , 0.0508 , -0.3400 ])
poses["PICTURE"]                = ("6", [ -1.5700 , -0.4208 , -0.1500 , 0.9000 , 1.8300 , -0.0792 , -0.5500 ])
poses["TUC"]                    = ("",  [ -0.3080 , 0.2312 , 1.6400 , 1.6220 , -0.2350 , -0.0562 , 0.6500 ])
poses["EGYPTIAN1"]              = ("",  [ 0.0000 , 1.0000 , 0.0000 , 1.4000 , 0.0000 , -0.8292 , 0.0000 ])
poses["EGYPTIAN2"]              = ("",  [ 0.0000 , -2.5708 , 0.0000 , 1.0000 , 0.0000 , -0.8292 , 0.0000 ])
poses["GOAL"]                   = ("",  [ 0.0000 , 1.5692 , 0.0000 , 1.5700 , 0.0000 , 0.0008 , 0.0000 ])
poses["NORMAL"]                 = ("u", [ -0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800 ])
poses["HANDSHAKEUP_NEW"]        = ("a", [ -0.3650 , 0.9436 , 0.4707 , 1.0420 , 0.7136 , -0.1692 , 0.2180 ])
poses["lEFT_HANDSHAKEDOWN_NEW"] = ("",  [ -0.2750 , 0.8678 , 0.6077 , 0.8030 , 0.7176 , -0.2852 , 0.1450 ])
poses["SALUT1"]                 = ("d", [ -1.2240 , 0.9271 , -1.0283 , 2.2860 , -0.2499 , -0.1762 , -0.0300 ])
poses["SALUT2"]                 = ("f", [ -1.2810 , 0.9842 , -0.3800 , 1.6190 , -0.0060 , -0.1762 , -0.0300 ])
poses["BICEPS_UP"]              = ("g", [ -0.1170 , 0.1960 , 0.2717 , 1.8920 , -1.2647 , -0.1022 , 0.2000 ])
poses["BICEPS_DOWN"]            = ("h", [ -0.1170 , 0.1960 , 0.2717 , 0.4120 , -1.2647 , -0.1022 , 0.2900 ])
poses["HIP"]                    = ("",  [ -0.8000 , -0.1358 , 1.3590 , 1.6980 , 0.0000 , -0.1072 , -0.0550 ])
poses["SHOULDER_TRAINING1"]     = ("j", [ -0.6930 , 1.3882 , 0.3170 , 0.5800 , 0.5550 , 0.0888 , -0.0200 ])
poses["SHOULDER_TRAINING2"]     = ("k", [ -1.3700 , -0.0138 , -0.1565 , 0.4830 , 0.1030 , 0.2408 , 0.1470 ])
poses["SHOULDER_TRAINING3"]     = ("l", [ -0.0600 , 0.0302 , -0.1560 , 0.4870 , 0.5210 , 0.3158 , 0.1000 ])
poses["SQUEEZE"]                = ("x", [ 0.0680 , 0.4437 , 0.3035 , 0.7000 , 1.3170 , -0.1642 , -0.3000 ])
poses["FRONT_GRASP_OLD"]        = ("",  [ 0.0770 , 1.4192 , 1.2236 , 0.2970 , -1.2040 , 0.1948 , 0.1570 ])
poses["FRONT_GRASP"]     	    = ("c", [ -0.3053 , 1.07292 , 0.366618 , 0.6427728 , -0.617385 , 0.04477456 , -0.07338288 ])
poses["FRONT_GRASP_CLOSE"]      = ("C", [ -0.22288 , -0.09266 , 0.172507 , 1.42479 , -0.145511 , 0.285575 , -0.001989])
poses["FLOOR_GRASP1"]           = ("v", [ -0.0480 , -0.0431 , -1.7130 , 0.4010 , 0.2620 , 0.5938 , -0.0090 ])
poses["FLOOR_GRASP2"]           = ("b", [ -0.2230 , -0.0108 , -0.0078 , 0.0600 , 0.0000 , -0.0572 , -0.1190 ])
poses["HAT_ON"]                 = ("",  [ -0.8500 , 0.8992 , -0.3430 , 2.2000 , -0.4400 , 0.4398 , 0.0020 ])
poses["HAT_WAVE"]               = ("",  [ -0.7320 , 0.7791 , -0.5920 , 1.7550 , -0.0900 , 0.4408 , 0.0020 ])
poses["SAFE"]                   = ("i", [ -0.5230 , 0.4672 , -0.4560 , 0.5860 , -0.3470 , 0.1308 , 0.1500 ])
poses["RIGHT_ANGLE"]            = ("",  [ 0.0000 , 0.0000 , 0.0000 , 1.5708 , 0.0000 , 0.0000 , 0.0000 ])
poses["RIGHT_DRIVE"]            = ("",  [ 0.0000 , 0.2292 , 0.0000 , 0.7100 , 0.0000 , 0.0000 , 0.0000 ])
poses["RIGHT_HANDSHAKEUP"]      = ("",  [ 0.0000 , -1.0000 , -0.1000 , 1.0000 , -1.8300 , 0.0508 , -0.2300 ])
poses["RIGHT_HANDSHAKEDOWN"]    = ("",  [ 0.0000 , -1.0000 , -0.1000 , 0.4300 , -1.8300 , 0.0508 , 0.3400 ])
poses["RIGHT_PICTURE"]          = ("",  [ -1.4600 , -0.4208 , 0.1500 , 0.9000 , -1.8300 , -0.0792 , 0.5500 ])
poses["RIGHT_TUC"]              = ("",  [ -0.4000 , -0.0608 , 0.0000 , 0.6000 , 0.0100 , 0.0008 , 0.0000 ])
poses["RIGHT_EGYPTIAN1"]        = ("",  [ 0.0000 , 1.1292 , 0.0000 , 1.0000 , 0.0000 , -1.2292 , 0.0000 ])
poses["RIGHT_EGYPTIAN2"]        = ("",  [ 0.0000 , -1.0708 , 0.0000 , 1.4000 , 0.0000 , -0.8292 , 0.0000 ])
poses["RIGHT_GRASP1"]           = ("",  [0.0, 0.73425897, 0.17566431, 0.53222897, -0.14837615, 0.25673428, 0.00852748])
poses["RIGHT_PRE_GRASP1"]       = ("",  [-0.2042001, -0.36908028, 0.14210841, 1.8928793, -0.10121332, 0.08952796, 0.05080516])
poses["RIGHT_PRE_GRASP2"]       = ("",  [-0.02744625, 0.11145711, 0.15037341, 2.214187105, -0.108680065, 0.08952796, 0.05084748])
poses["PICK_UP"]                = ("",  [-0.00595091, 0.22645549, 0.29562252, 1.706210675, 0.019163495, -0.26780096, 0.0008464])
poses["HOME"]                   = ("",  [ 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 , 0.0000 ])
poses["TIP1"]                   = ("",  [ 0.0, 0.46537017, 0.04879656, 1.138768735, 0.053594125, 0.13434484, 0.06629428])
poses["TIP2"]                   = ("",  [-0.31459877, 0.49708959, 0.4254822, 1.216120685, 1.720162405, -0.0662308, 0.03440616])
poses["FLOWERS_LEFT"]           = ("",  [-0.06670424, -0.30989772, 0.21649341, 1.781292305, 1.349394605, 0.22025444, -0.44696268])
poses["FLOWERS_RIGHT"]          = ("",  [-0.06, 0.27889894, 0.00657894, 1.48349305, 0.52218894, 0.53084092, -0.28932068])
poses["HI5_pre"]				= ("",  [-0.0, 1.577, 0.0, 1.577, -0.0, 0.000, 0])
poses["HI5_post"]				= ("",  [0.0, 1.57, 0.000, 1.07928, -0.00, 0.00, 0.00])

poses_transposed = {key:(name, joints) for name, (key, joints) in poses.iteritems() if key}

#TODO: Make 2 the language to use an argument
dictionary = dict()
language = "nl"
voices = {"nl":"david", "en":"kyle"}

sentences = dict()
sentences["HELLO"] = "Hallo!"#"Hello"
sentences["INTRODUCE"] = "Mijn naam is Amigo"#"My name is AMIGO"
sentences["TUE"] = "Ik ben de servicerobot van de TU Eindhoven"#"I am the service robot of th TU Eindhoven"
sentences["PICTURE"] = "Wil je op de foto met mij?"#"Do you want to take a picture with me?"
sentences["NO_BITE"] = "Niet bang zijn, ik bijt niet. Dat kan ik niet eens!"#"Don't be afraid, I wont bite you! How could I ever?"
sentences["HI"] = "Hoi!"#"Hi"
sentences["GREET"] = "Hoi. ik heet Amigo."#"Hello, My name is Amigo."
sentences["YOURNAME?"] = "Hoe heet jij?"#"Hello, My name is Amigo."
sentences["APPLAUSE"] = "Applaus alstublieft!"#"Applause please!"
sentences["AUB"] = "Alsjeblieft!"#"Applause please!"
sentences["OUCH"] = "Au! Dat doet pijn."#"Ouch! That hurts!"
sentences["YES"] = "Ja"#"Yes!"
#sentences[""] = ""

sent_keymap = dict()
sent_keymap['d'] = "HI"
sent_keymap['6'] = "PICTURE"
sent_keymap['$'] = "GREET"
sent_keymap['H'] = "HELLO"
#sent_keymap['B'] = "NO_BITE"
sent_keymap['I'] = "INTRODUCE"
sent_keymap['Y'] = "YOURNAME?"
sent_keymap['A'] = "APPLAUSE"
sent_keymap['Z'] = "AUB"
sent_keymap['!'] = "OUCH"
sent_keymap['Y'] = "YES"
sent_keymap['p'] = "PASS"

#TODO: Looking at finger is not yet tested!!
@dec.register_robot_key("q", "right grasp")
def grasp(robot):
    print "joints = RIGHT_GRASP1"
    robot.rightArm.send_gripper_goal_close()
    robot.rightArm.send_joint_goal(*poses["RIGHT_PRE_GRASP1"][1])

    #One option is to let Amigo look to his hands directly using a point relative to it:
    robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/finger1_right"))

    #rospy.sleep(rospy.Duration(4.0))

    robot.rightArm.send_joint_goal(*poses["RIGHT_PRE_GRASP2"][1])
    #rospy.sleep(rospy.Duration(0.5))

    robot.rightArm.send_joint_goal(*poses["RIGHT_GRASP1"][1])

    #Look at finger again
    robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/finger1_right"))

@dec.register_robot_key("e")
def pickup(robot):
    print "joints = PICK_UP"
    robot.rightArm.send_gripper_goal_open()
    #rospy.sleep(rospy.Duration(2.0))

    robot.rightArm.send_joint_goal(*poses["PICK_UP"][1])
    robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/finger1_right"))

    #rospy.sleep(rospy.Duration(2.0))
    robot.rightArm.send_joint_goal(*poses["RIGHT_PRE_GRASP1"][1])
    robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/finger1_right"))

    #rospy.sleep(rospy.Duration(2.0))

    robot.head.reset_position()

@dec.register_robot_key("t")
def give(robot):
    print "joints = GIVE"
    robot.rightArm.send_joint_goal(*poses["RIGHT_PRE_GRASP1"][1])
    #rospy.sleep(rospy.Duration(4.0))
    robot.rightArm.send_joint_goal(*poses["RIGHT_PRE_GRASP2"][1])
    #rospy.sleep(rospy.Duration(0.5))
    robot.rightArm.send_joint_goal(*poses["RIGHT_GRASP1"][1])
    #rospy.sleep(rospy.Duration(4.0))
    robot.rightArm.send_gripper_goal_close()

@dec.register_robot_key("y")
def home(robot):
    print "joints = HOME"
    robot.head.reset_position()
    robot.rightArm.send_joint_goal() #defaults to 0, 0....
    robot.rightArm.send_gripper_goal_close()

@dec.register_robot_key("7")
def tip(robot):
    print "joints = TIPPING"
    robot.rightArm.send_joint_goal(*poses["TIP1"][1])
    robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/finger1_right"))
    #rospy.sleep(rospy.Duration(4.0))
    robot.head.send_goal(msgs.PointStamped(0,0,0, frame_id="/finger1_right"))
    #robot.rightArm.send_joint_goal(*poses["TIP2"][1])
    rospy.sleep(rospy.Duration(1.0))

@dec.register_robot_key("w")
def wave2(robot):
    joints = poses["GOAL"][1]

    robot.leftArm.send_joint_goal(*joints)
    robot.rightArm.send_joint_goal(*joints)

    #rospy.loginfo("double wave, abort with esc")

    start_time = rospy.Time.now()

    while ((rospy.Time.now() - start_time) < rospy.Duration(10.0)) and not actions_canceled:
        joints[2] = 0.5*sin(0.25*3.14 * (rospy.Time.now().to_sec() - start_time.to_sec()) )
        robot.leftArm.send_joint_goal(*joints)
        joints[2] = -joints[2] # q3 of left and right arm should be mirrorred
        robot.rightArm.send_joint_goal(*joints)
        #rospy.sleep(rospy.Duration(0.05))
    rospy.loginfo("Waving done")
    robot.rightArm.send_joint_goal(*poses["DRIVE"][1])
    robot.leftArm.send_joint_goal(*poses["DRIVE"][1])

@dec.register_robot_arm_key("n")
def wave1(robot, arm):
    joints = poses["GOAL"][1]
    arm.send_joint_goal(*joints)

    start_time = rospy.Time.now()

    while ((rospy.Time.now() - start_time) < rospy.Duration(10.0)) and not actions_canceled:
        joints[2] = 0.5*sin(0.25*3.14 * (rospy.Time.now().to_sec() - start_time.to_sec()) )
        arm.send_joint_goal(*joints)
    rospy.loginfo("Waving done")
    arm.send_joint_goal(*poses["DRIVE"][1])

@dec.register_robot_key("m")
def wave_lights(robot):
    rospy.loginfo("GOAL, LET'S CHEER")
    os.system("mpg123 -q /home/amigo/Music/Toeter1.mp3 &")

    joints = poses["GOAL"][1]

    robot.leftArm.send_joint_goal(*joints)
    robot.rightArm.send_joint_goal(*joints)

    #rospy.loginfo("double wave, abort with esc")

    start_time = rospy.Time.now()

    while ((rospy.Time.now() - start_time) < rospy.Duration(10.0)) and not actions_canceled:
        joints[2] = 0.5*sin(0.25*3.14 * (rospy.Time.now().to_sec() - start_time.to_sec()) )
        robot.leftArm.send_joint_goal(*joints)
        joints[2] = -joints[2] # q3 of left and right arm should be mirrorred
        robot.rightArm.send_joint_goal(*joints)
        #rospy.sleep(rospy.Duration(0.05))
        robot.lights.set_color( random.random(),
                                random.random(),
                                random.random())
    rospy.loginfo("Waving done")

    robot.rightArm.send_joint_goal(*poses["DRIVE"][1])
    robot.leftArm.send_joint_goal(*poses["DRIVE"][1])
    robot.lights.set_color(0, 0, 1)

@dec.register_robot_key("Q")
def ask_drinks(robot):
    import smach
    import robot_smach_states as states

    sm = smach.StateMachine(outcomes=['Done','Aborted'])
    with sm:
        smach.StateMachine.add("ASK_QUESTION",
                                states.Timedout_QuestionMachine(robot=robot,
                                                    default_option='nothing',
                                                    sentence="What do you want to drink?",
                                                    options=['coke', 'fanta', 'sprite']),
                                transitions={'answered':'RECITE_ORDER',
                                             'not_answered':'RECITE_ORDER'},
                                remapping={'answer':'sentence'})

        smach.StateMachine.add("RECITE_ORDER",
                               states.Say_generated(robot,
                                                    lambda ud: "You want {0}? I am new in this environment, but I will try to get {0} for you.".format(ud.sentence),
                                                    ["sentence"]),
                                transitions={'spoken':'Done'})
    outcome = sm.execute()
    global drink
    drink = sm.userdata.sentence

@dec.register_robot_arm_key("G")
def grab_item(robot, selectedArm):
    rospy.loginfo("Starting to grab an item")
    import smach
    import robot_smach_states as states

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
    robot.reasoner.detach_all_from_gripper("/amigo/grippoint_left")
    robot.reasoner.detach_all_from_gripper("/amigo/grippoint_right")

    with sm:
        smach.StateMachine.add('ANNOUNCE_LOOK_FOR_DRINK',
                                states.Say(robot, "I wonder what objects I can see here!", block=False),
                                transitions={'spoken':'LOOK'})
        
        query_lookat = Conjunction( Compound("current_exploration_target", "Target"),
                                    Compound("point_of_interest", "Target", Compound("point_3d", "X", "Y", "Z")))

        
        rospy.logerr("TODO Loy/Janno/Sjoerd: make query use current_object")
        query_object = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z"))

        smach.StateMachine.add('LOOK',
                                states.LookForObjectsAtPoint(robot, query_object, msgs.PointStamped(0.75, 0, 0.65, frame_id="/amigo/base_link")),
                                transitions={   'looking':'LOOK',
                                                'object_found':'GRAB',
                                                'no_object_found':'SAY_NO_DRINK',
                                                'abort':'Aborted'})

        query_grabpoint = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z"))
        smach.StateMachine.add('GRAB',
                        states.GrabMachine(selectedArm, robot, query_grabpoint),
                        transitions={   'succeeded':'CARRYING_POSE',
                                        'failed':'REPORT_FAILED' })

        smach.StateMachine.add('CARRYING_POSE',
                               states.Carrying_pose(selectedArm, robot),
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

@dec.register_robot_key("B")
def grab_absolute_position(robot):

    rospy.logwarn("Temporarily storing the real worldmodel...")
    orig_WM = robot.worldmodel

    from test_tools.mocked_parts.worldmodel import MockedWorldModel
    robot.worldmodel = MockedWorldModel(robot.tf_listener)

    rospy.loginfo("Starting to grab an item")
    import smach
    import robot_smach_states as states
    import object_msgs.msg

    #Copied from RDO finale
    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    sm.userdata.target = object_msgs.msg.ExecutionTarget() #drink is a global var
    sm.userdata.known_object_list = rospy.get_param("known_object_list")
    sm.userdata.desired_objects = [obj for obj in sm.userdata.known_object_list]# if obj["class_label"]=="soup_pack"] #drink is a global var
    sm.userdata.operator_name = "my operator"
    sm.userdata.rate = 10 #first used in LOOK_FOR_DRINK. TODO:refactor this parameter out
    sm.userdata.command = "" #First used in LOOK_FOR_DRINK
    sm.userdata.dropoff_location = object_msgs.msg.ExecutionTarget(name=sm.userdata.target.category,class_label="location",ID=-2) #Also used in LOOK_FOR_DRINK.
    rospy.loginfo("Userdata set")

    robot.worldmodel.reset()

    with sm:

        @smach.cb_interface(outcomes=['accepted'], output_keys=['target'])
        def read_abs_loc(userdata):
            userdata.target=object_msgs.msg.ExecutionTarget(ID=666)

            input = raw_input("Coordinates of object? e.g. 1.2,3.4,5.6: \n")
            parts = input.split(",")
            coords = tuple([float(part) for part in parts])

            import geometry_msgs.msg as gm
            import object_msgs.msg as om

            dummy = om.SeenObject(class_label="drink",
                                    pose=gm.Pose(position=gm.Point(*coords)),
                                    ID=666)

            robot.worldmodel.insert_object(dummy)
            return 'accepted'


        smach.StateMachine.add('INSERT_ITEM',
                               smach.CBState(read_abs_loc),
                               transitions={'accepted':'GRAB'})
        smach.StateMachine.add('GRAB',
                               states.GrabMachine(robot.leftArm,robot),
                               transitions={'succeeded':'CARRYING_POSE',
                                            'failed':'REPORT_FAILED'})

        smach.StateMachine.add('CARRYING_POSE',
                               states.Carrying_pose(robot.leftArm, robot),
                               transitions={'succeeded':'Done',
                                            'failed':'Done'})

        smach.StateMachine.add('REPORT_FAILED',
                               states.Say_generated(robot,
                                                    lambda userdata: "I am sorry I could not get your object.".format(userdata.operator_name),
                                                    input_keys=['operator_name']),
                               transitions={'spoken':'Done'})

    rospy.loginfo("State machine set up, start execution...")
    #import pdb; pdb.set_trace()
    result = sm.execute()

    rospy.logwarn("Restoring original WM")
    robot.worldmodel = orig_WM

    rospy.loginfo("State machine executed. Result: {0}".format(result))

@dec.register_robot_key("O")
def amigo_introduction_english(robot):

    import smach
    import robot_smach_states as states

    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=['Done'])

    with sm:

        smach.StateMachine.add('AMIGO_INTRODUCTION',
                               states.AmigoIntroductionEnglish(robot),
                               transitions={'finished_introduction':'Done'})

    rospy.loginfo("State machine set up, start execution...")
    #import pdb; pdb.set_trace()
    result = sm.execute()

    rospy.loginfo("State machine executed. Result: {0}".format(result))

@dec.register_robot_key("o")
def amigo_introduction_dutch(robot):

    import smach
    import robot_smach_states as states

    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=['Done'])

    with sm:

        smach.StateMachine.add('AMIGO_INTRODUCTION',
                               states.AmigoIntroductionDutch(robot),
                               transitions={'finished_introduction':'Done'})

    rospy.loginfo("State machine set up, start execution...")
    #import pdb; pdb.set_trace()
    result = sm.execute()

    rospy.loginfo("State machine executed. Result: {0}".format(result))

@dec.register_robot_key("`")
def prepare_welcome_maxima(robot):
    import smach
    import robot_smach_states as states
    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=["Done"])
    with sm:
        smach.StateMachine.add('PREPARE_WELCOME_MAXIMA',
            states.PrepareWelcomeMaxima(robot),
            transitions={'done':'Done'})
    rospy.loginfo("State machine set up, start execution...")
    result = sm.execute()
    rospy.loginfo("State machine executed. Result: {0}".format(result))

@dec.register_robot_key("1")
def welcome_maxima1(robot):
    import smach
    import robot_smach_states as states
    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=["Done"])
    with sm:
        smach.StateMachine.add('WELCOME_MAXIMA',
            states.WelcomeMaxima1(robot),
            transitions={'done':'Done'})
    rospy.loginfo("State machine set up, start execution...")
    result = sm.execute()
    rospy.loginfo("State machine executed. Result: {0}".format(result))

@dec.register_robot_key("2")    
def welcome_maxima2(robot):
    import smach
    import robot_smach_states as states
    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=["Done"])
    with sm:
        smach.StateMachine.add('WELCOME_MAXIMA',
            states.WelcomeMaxima2(robot),
            transitions={'done':'Done'})
    rospy.loginfo("State machine set up, start execution...")
    result = sm.execute()
    rospy.loginfo("State machine executed. Result: {0}".format(result))

@dec.register_robot_key("3")    
def welcome_maxima3(robot):
    import smach
    import robot_smach_states as states
    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=["Done"])
    with sm:
        smach.StateMachine.add('WELCOME_MAXIMA',
            states.WelcomeMaxima3(robot),
            transitions={'done':'Done'})
    rospy.loginfo("State machine set up, start execution...")
    result = sm.execute()
    rospy.loginfo("State machine executed. Result: {0}".format(result))

@dec.register_robot_key("S")
def grab_demo(robot):
    import smach
    import robot_smach_states as states
    
    robot.leftArm.reset_arm()
    robot.rightArm.reset_arm()
    robot.reasoner.reset()
    robot.reasoner.detach_all_from_gripper("/amigo/grippoint_left")
    robot.reasoner.detach_all_from_gripper("/amigo/grippoint_right")

    r = robot.reasoner
    r.query(r.load_database("tue_knowledge", 'prolog/locations.pl'))
    r.assertz(r.challenge("clean_up"))

    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=['Done', "Failed", "Aborted"])

    search_query = Compound("point_of_interest", "Poi", Compound("point_3d", "X", "Y", "Z"))
    object_query = Compound("position", Compound("point_3d", "X", "Y", "Z")) #I don't care a bout the type for now.

    with sm:
        smach.StateMachine.add('GET_OBJECT',
                               states.GetObject(robot, robot.leftArm, search_query, object_query),
                               transitions={'Done':'DROPOFF_OBJECT',
                                            'Failed':'Failed',
                                            'Aborted':'Aborted',
                                            'Timeout':'Failed'})

        query_dropoff_loc = Compound("point_of_interest", "trashbin1", Compound("point_3d", "X", "Y", "Z"))
        smach.StateMachine.add("DROPOFF_OBJECT",
                                states.Gripper_to_query_position(robot, robot.leftArm, query_dropoff_loc),
                                transitions={   'succeeded':'DROP_OBJECT',
                                                'failed':'Failed',
                                                'target_lost':'Failed'})
        smach.StateMachine.add( 'DROP_OBJECT', states.SetGripper(robot, robot.leftArm, gripperstate=0), #open
                                transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                'failed'   :'CLOSE_AFTER_DROP'})
        smach.StateMachine.add( 'CLOSE_AFTER_DROP', states.SetGripper(robot, robot.leftArm, gripperstate=1), #close
                                transitions={   'succeeded':'RESET_ARM',
                                                'failed'   :'RESET_ARM'})
        smach.StateMachine.add('RESET_ARM',
                                states.ArmToPose(robot, robot.leftArm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), #Copied from demo_executioner NORMAL
                                transitions={   'done':'GET_OBJECT',
                                                'failed':'Failed'})

    rospy.loginfo("State machine set up, start execution...")
    #import pdb; pdb.set_trace()
    import smach_ros
    introserver = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT_PRIMARY')
    introserver.start()
    result = sm.execute()
    introserver.stop()

    r.query(r.retractall(r.visited("X")))
    r.query(r.retractall(r.unreachable("X")))

    rospy.loginfo("State machine executed. Result: {0}".format(result))

@dec.register_robot_key("D", "Drop off an object at a predefined location. Can be edited via python! See the dropoff_demo-function")
def dropoff_demo(robot, selectedArm, query_dropoff_loc=None):
    import smach
    import robot_smach_states as states

    r = robot.reasoner
    r.query(r.load_database("tue_knowledge", 'prolog/locations.pl'))
    r.query(r.load_database("tue_knowledge", 'prolog/objects.pl'))
    r.assertz(r.challenge("clean_up"))

    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=['Done', "Failed", "Aborted"])

    if not query_dropoff_loc:
        query_dropoff_loc = Compound("dropoff_point",   "hallway_table", Compound("point_3d", "X", "Y", "Z"))
        # query_dropoff_loc = Conjunction(
        #                             Compound("is", "X", 2.0),
        #                             Compound("is", "Y", 2.2),
        #                             Compound("is", "Z", 0.8))
    rospy.loginfo("Dropping to {0}".format(query_dropoff_loc))

    with sm:
        smach.StateMachine.add("DROPOFF_OBJECT",
                        #PlaceObject(side, robot, placement_query, dropoff_height_offset=0.1):
                        #states.Gripper_to_query_position(robot, robot.leftArm, query_dropoff_loc),
                        states.DropObject(selectedArm, robot, query_dropoff_loc),
                        transitions={   'succeeded':'Done',
                                        'failed':'Failed',
                                        'target_lost':'Failed'})

    rospy.loginfo("State machine set up, start execution...")
    #import pdb; pdb.set_trace()
    import smach_ros
    introserver = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT_PRIMARY')
    introserver.start()
    result = sm.execute()
    introserver.stop()

    # if not via_reasoner:
    #     r.query(r.retractall(Compound("demo_dropoff_point",   Compound("point_3d", "X", "Y", "Z"))))

    rospy.loginfo("State machine executed. Result: {0}".format(result))

@dec.register_robot_key(":")
def learn_face(robot):
    import smach
    import robot_smach_states as states
    from speech_interpreter.srv import AskUser
    
    name = "mr. X"

    try:
        ask_user_service_get_learn_person_name = rospy.ServiceProxy('interpreter/ask_user', AskUser)
        response = ask_user_service_get_learn_person_name("name", 3 , rospy.Duration(60))  # This means that within 3 tries and within 60 seconds an answer is received.
        
        for x in range(0,len(response.keys)):
                if response.keys[x] == "answer":
                    response_answer = response.values[x]

        name = response_answer

        if name == "no_answer" or name == "wrong_answer":
            name = "mr. X"
    except Exception, e:
        rospy.logerr(e)

    robot.speech.speak("Hi {0}".format(name))

    rospy.loginfo("Setting up state machine")
    sm = smach.StateMachine(outcomes=['Done', "Failed", "Aborted"])

    with sm:
        smach.StateMachine.add('LEARN_FACE',
                                states.Learn_Person(robot, name),
                                transitions={   'face_learned':'Done',
                                                'learn_failed':'Failed'})

    rospy.loginfo("State machine set up, start execution...")
    #import pdb; pdb.set_trace()
    import smach_ros
    introserver = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT_PRIMARY')
    introserver.start()
    result = sm.execute()
    introserver.stop()

    rospy.loginfo("State machine result: {0}".format(result))

@dec.register_robot_key(";")
def recognize_face(robot):
    #import smach

    robot.speech.speak("Let me see who I can find here...")
    robot.head.set_pan_tilt(tilt=-0.2)
    robot.spindle.reset()

    robot.perception.toggle(["face_segmentation"])
    rospy.sleep(5.0)
    robot.perception.toggle([])

    person_result = robot.reasoner.query(
                                        Conjunction(  
                                            Compound( "property_expected", "ObjectID", "class_label", "face"),
                                            Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))))
    if not person_result:
        robot.speech.speak("No one here. Face segmentation did not find a person here")
        return False

    robot.speech.speak("Hi there, human. Please look into my eyes, so I can recognize you.")  
    
    robot.perception.toggle(["face_recognition", "face_segmentation"])
    rospy.sleep(5.0)
    robot.perception.toggle([])
    person_result = robot.reasoner.query(
                                        Conjunction(  
                                            Compound( "property_expected", "ObjectID", "class_label", "face"),
                                            Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                            Compound( "property", "ObjectID", "name", Compound("discrete", "DomainSize", "NamePMF"))))

        # get the name PMF, which has the following structure: [p(0.4, exact(will)), p(0.3, exact(john)), ...]
    if len(person_result) > 0:
        name_pmf = person_result[0]["NamePMF"]

        if len(person_result) > 1:
            rospy.logwarn("Multiple faces detected, only checking the first one!")

        name=None
        name_prob=0
        for name_possibility in name_pmf:
            print name_possibility
            prob = float(name_possibility[0])
            if prob > 0.175 and prob > name_prob:
                name = str(name_possibility[1][0])
                #print "Updated most probable name to " + str(name)
                name_prob = prob

        if not name:
            robot.speech.speak("I don't know who you are.")
            #return "looking"        

        if name:
            robot.speech.speak("Hello " + str(name))        
    else:
        rospy.logwarn("No person names received from world model")    
        #return "found"

    #return "not_found"


    # rospy.loginfo("Setting up state machine")
    # sm = smach.StateMachine(outcomes=['Done', "Failed", "Aborted"])

    # with sm:
    #     smach.StateMachine.add('RECOGNIZE_FACE',
    #                             recognize_face(robot),
    #                             transitions={   'looking':'Failed',
    #                                             'found':'Done',
    #                                             'not_found':'Failed'})

    # rospy.loginfo("State machine set up, start execution...")
    # #import pdb; pdb.set_trace()
    # import smach_ros
    # introserver = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT_PRIMARY')
    # introserver.start()
    # result = sm.execute()
    # introserver.stop()

    # rospy.loginfo("State machine result: {0}".format(result))

dec.register_robot_key("T", "Transporter demo. Place AMIGO in from of the special box and amigo will bring objects from one spot to another")(dec.run_statemachine(Transporter))
dec.register_robot_key("P", "go to a Person. Uses the laser scanner to go to a person and say something on arrival")(dec.run_statemachine(DriveToClosestPerson))

@dec.register_robot_key(chr(27))
def cancel_actions(robot):
    global actions_canceled
    actions_canceled = True

    robot.leftArm.cancel_goal()
    robot.rightArm.cancel_goal()
    robot.base.cancel_goal()
    robot.head.cancel_goal()

@dec.register_key("L")
def next_language():
    global language

    languages = list(dictionary.keys())
    languages = sorted(languages)
    current_lang_index = languages.index(language)

    next_lang_index = current_lang_index + 1
    next_lang = languages[next_lang_index%len(languages)] #Use remainder to stay in bounds
    language = next_lang
    rospy.loginfo("Current language: {0} out of {1}".format(language.upper(), languages))

    rospy.set_param("/language", language)

@dec.register_key('\t')
def toggle_arm():
    global arm_selection
    arm_selection = not arm_selection
    if arm_selection == True:
        print "Commanding RIGHT arm, press TAB to toggle"
    else:
        print "Commanding LEFT arm, press TAB to toggle"

@dec.register_robot_key("F", "Hold flowers")
def hold_flowers(robot):
    robot.rightArm.send_joint_goal(*poses["FLOWERS_RIGHT"][1])
    robot.leftArm.send_joint_goal(*poses["FLOWERS_LEFT"][1])

@dec.register_robot_arm_key("<", "OPEN gripper")
def open_gripper(robot, arm):
    arm.send_gripper_goal_open()

@dec.register_robot_arm_key(">", "CLOSE gripper")
def close_gripper(robot, arm):
    arm.send_gripper_goal_close()

dec.register_key(" ", "exit")(exit)
dec.register_robot_key("-", "Head down")(lambda robot: robot.head.look_down())
dec.register_robot_key("+", "Head straight")(lambda robot: robot.head.reset_position())

############################## callback speech ###################################
def callback_speech(data):
    speech_command = data.data
    global key
    rospy.loginfo("Received '{0}' through speech".format(speech_command))
    if speech_command == "introduceyourself":
        key = "O"
    elif speech_command == "grasp":
        key = "G"
    elif speech_command == "wave":
        key = "n"
    elif speech_command == "relax":
        key = "u"
    elif speech_command == "picture":
        key = "6"
    process_key(key)
    return True

############################## callback keyboard_cmd ###################################
def callback_keyboard_cmd(data):
    global key
    key = data.data
    process_key(key)
    return True

def process_key(key):
    #Rtoggled can be 0 or 1, like a boolean.. Its purpose is to determine if a certain move is for the right arm or not (then left arm)
    #arm_selection = True #True: Left, False: Right
    global arm_selection
    arms = {False:robot.leftArm, True:robot.rightArm}

    if rospy.has_param("/language"):
        global language
        language = rospy.get_param("/language")

    if key in dec.mapping:
        print dec.namings[key]
        dec.mapping[key]()

    if key in dec.robot_mapping:
        print dec.namings[key]
        dec.robot_mapping[key](robot)

    if key in dec.robot_arm_mapping:
        print dec.namings[key]
        dec.robot_arm_mapping[key](robot, arms[arm_selection])

    try:
        if key in poses_transposed:
            global actions_canceled
            actions_canceled = False
            pose_name, joints = poses_transposed[key]
            print "{0} : {1}".format(key, pose_name)
            arms[arm_selection].send_joint_goal(*joints)

        #~ else:
            #~ rospy.logwarn("Unbound key {0} ({1})".format(key))

        if sent_keymap.has_key(key) and dictionary[language].has_key(sent_keymap[key]):
            #we look up the language->sentencename map.
            #Then, we look up what key maps to which sentencename.
            #Finally we maps the sentencename to the actual sentence
            selected_dict = dictionary[language]
            sentencename = sent_keymap[key]
            sentence_to_say = selected_dict[sentencename]

            if isinstance(sentence_to_say, list):
                sentence_to_say = random.choice(sentence_to_say)

            rospy.logdebug(str(sentence_to_say)+str(language)+str(voices[language]))

            robot.speech.speak( sentence_to_say,
                                language,
                                voices[language])

    except Exception, e:
        import pdb; pdb.set_trace()
        print e
        raw_input("Press enter to quit the crashed teleop-panel. Please take note of the error to have it fixed.")
        exit()

robot = None
arm_selection = True
drink = "tea_pack"
actions_canceled = False

@dec.register_key("/", "print keymap (so, this) in this window")
def print_keys(separate=False):
    commands = ["{0} = {1}".format(k, name) for name, (k, v) in poses.iteritems() if k]
    command_txt = '\n'.join(commands)
    if not separate:
        print command_txt

    mappings = ["{0} = {1}".format(k, v) for k, v in dec.namings.iteritems()]
    special_txt = '\n'.join(mappings)
    if not separate:
        print "Sequences: "
        print special_txt

    speech = ["{0} = {1} - {2}".format(k, dictionary['nl'].get(v, "---"), dictionary['en'].get(v, "---")) for k, v in sent_keymap.iteritems()]
    speech_txt = '\n'.join(speech)
    if not separate:
        print "Speech: "
        print speech_txt

    all_keys = "`1234567890-=qwertyuiop[]asdfghjkl;'\zxcvbnm,./" + "~!@#$%^&*()_+QWERTYUIOP{}ASDFGHJKL:|ZXCVBNM<>?"
    used_keys = [k for (k, v) in poses.values() if k] + dec.namings.keys() + sent_keymap.keys()
    unused_keys = set(all_keys) - set(used_keys)
    print "Unused keys: {0}".format("".join(sorted(unused_keys)))

    keymap_file = file('keymap.txt', 'w+')
    keymap_file.write(command_txt+'\n')
    keymap_file.write(special_txt+'\n')
    keymap_file.write(speech_txt)
    keymap_file.close()

    if not separate:
        print "\033[1m" + "PRESS ? to display the keymap in a separate window!" + "\033[0;0m" #From http://stackoverflow.com/questions/8924173/python-print-bold-text

    if separate:
        os.system("geany -i -m -s keymap.txt &") #-i: new instance, -m: Don't show message window at startup, -s: Don't load the previous session's files

dec.register_key("?", "print keymap (so, this) in a separate window")(lambda: print_keys(separate=True))

def main():
    keyboard_cmd_listener = rospy.Subscriber("key_commands",String,callback_keyboard_cmd)
    speech_listener = rospy.Subscriber("/speech/output",String,callback_speech)
    rospy.init_node('amigo_demo_executioner')

    global dictionary
    if rospy.has_param("/nl"):
        dictionary['nl'] = rospy.get_param("/nl")
    else:
        dictionary['nl'] = sentences
    if rospy.has_param("/en"):
        dictionary['en'] = rospy.get_param("/en")
    if rospy.has_param("/language"):
        global language
        language = rospy.get_param("/language")


    print mesg

    print_keys()

    global robot
    #~ if len(sys.argv) > 1:
        #~ from test_tools.build_amigo import build_amigo
        #~ robot = build_amigo(fake=['base','arms','perception','head', 'worldmodel'])
    #~ else:
    robot = Amigo() #dontInclude=['perception'] is needed for grabbing items
    robot.lights.set_color(0, 0, 1)
    #import pdb; pdb.set_trace()
    rospy.spin()

if __name__ == "__main__":

    main()
