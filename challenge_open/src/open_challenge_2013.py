#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_open')
import rospy
import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from util.startup import startup #gives you speech-exception reading-out-loud and smach_viewer server

from robot_skills.reasoner import Conjunction, Compound

def setup_statemachine(robot):

    #retract old facts
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    robot.reasoner.query(Compound("retractall", Compound("goal", "X")))
    robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
    robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
    
    robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
    robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
    #Assert the current challenge.
    robot.reasoner.assertz(Compound("challenge", "open_challenge"))

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:
                              
        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'INIT_POSE',
                                                'abort':'Aborted'})
        

        smach.StateMachine.add('INIT_POSE',
                                states.Set_initial_pose(robot, "initial"),
                                transitions={   'done':'SAY_START',
                                                'preempted':'Aborted',
                                                'error':'Aborted'})

        smach.StateMachine.add("SAY_START", 
                                states.Say(robot, ["Hello everyone"]),
                                transitions={   'spoken':'QUESTION'})

        smach.StateMachine.add('QUESTION', 
                                    states.Timedout_QuestionMachine(
                                            robot=robot,
                                            default_option = "gotothelivingroom", 
                                            sentence = "Where do you want me to go", 
                                            options = { "gotothelivingroom":Compound("goal","living_room"),
                                                        "gotothekitchen":Compound("goal", "kitchen"),
                                                        "gotothelobby":Compound("goal", "lobby"),
                                                        "gotothebedroom":Compound("goal", "bedroom")}),
                                    transitions={   'answered':'MOVE_TO_INTRO_POINT',
                                                    'not_answered':'QUESTION'})
                                
        # ToDO: include some HRI

        ''' Navigate to room where the IKEA expedit cabinet is located
        during the ride AMIGO will be bullied to illustrate the move base stuff
        - An object blocks the path: show it does not immediately execute its alternative plan
        - A person blocks the path: tell him/her to move (also show in rviz)
        - An object blocks the path permanently: now the robot executes its alternative plan anyway
        Furthermore, the 3D map can be shown'''
        query_intro_point = Compound("base_pose", "meeting_point", Compound("pose_2d", "X", "Y", "Phi"))
        smach.StateMachine.add('MOVE_TO_INTRO_POINT',
                                states.NavigateGeneric(robot, goal_query=query_intro_point, look_at_path_distance=1.5),
                                transitions={   "arrived":"SAY_WBC",
                                                "unreachable":'SAY_WBC',
                                                "preempted":'SAY_WBC',
                                                "goal_not_defined":'SAY_WBC'})

        smach.StateMachine.add("SAY_WBC", 
                                states.Say(robot, ["Next to my navigation skills, I would like to show my new whole body controller"]),
                                transitions={   'spoken':'HIGH_LEFT_ARM'})

        smach.StateMachine.add("HIGH_LEFT_ARM",
                                states.ArmToUserPose(robot.leftArm, x=0.3, y=0.3, z=0.8, roll=0.0, pitch=0.0, yaw=0.0,
                                    time_out=20, pre_grasp=False, frame_id="/base_link"),
                                transitions={   "succeeded":"SAY_TWO_POSITIONS",
                                                "failed":"SAY_TWO_POSITIONS"})

        smach.StateMachine.add("SAY_TWO_POSITIONS", 
                                states.Say(robot, ["This means i can try to do two things at once"]),
                                transitions={   'spoken':'LOW_RIGHT_ARM'})

        smach.StateMachine.add("LOW_RIGHT_ARM",
                                states.ArmToUserPose(robot.rightArm, x=0.3, y=-0.3, z=0.4, roll=0.0, pitch=0.0, yaw=0.0,
                                    time_out=20, pre_grasp=False, frame_id="/base_link"),
                                transitions={   "succeeded":"SAY_HELPFUL",
                                                "failed":"SAY_HELPFUL"})

        smach.StateMachine.add("SAY_HELPFUL", 
                                states.Say(robot, ["This is helpful in many ways"]),
                                transitions={   'spoken':'RESET_LEFT_ARM'})

        smach.StateMachine.add("RESET_LEFT_ARM",
                                states.ArmToUserPose(robot.leftArm, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0,
                                    time_out=20, pre_grasp=False, frame_id="reset"),
                                transitions={   "succeeded":"RESET_RIGHT_ARM",
                                                "failed":"RESET_RIGHT_ARM"})

        smach.StateMachine.add("RESET_RIGHT_ARM",
                                states.ArmToUserPose(robot.rightArm, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0,
                                    time_out=20, pre_grasp=False, frame_id="reset"),
                                transitions={   "succeeded":"MOVE_TO_CABINET",
                                                "failed":"MOVE_TO_CABINET"})


        query_expedit_point = Compound("base_pose", "cabinet_expedit_1", Compound("pose_2d", "X", "Y", "Phi"))
        smach.StateMachine.add('MOVE_TO_CABINET',
                                states.NavigateGeneric(robot, goal_query=query_expedit_point, look_at_path_distance=1.5),
                                transitions={   "arrived":"SAY_LOOK_FOR_OBJECTS",
                                                "unreachable":'SAY_ABORT',
                                                "preempted":'SAY_ABORT',
                                                "goal_not_defined":'SAY_ABORT'})

        ''' At the cabinet, the robot grasp an object using the whole-body controller
        Then it opens a drawer with its other hand and throws the object in the drawer
        while holding the drawer '''
        smach.StateMachine.add("SAY_LOOK_FOR_OBJECTS", 
                                states.Say(robot, ["Lets see what I can find here."]),
                                transitions={   'spoken':'LOOK'})

        # ToDo: Replace query
        #query_lookat = Conjunction( Compound("current_exploration_target", "Target"),
        #                            Compound("region_of_interest", "Target", Compound("point_3d", "X", "Y", "Z")))
        query_lookat = Compound("region_of_interest", "cabinet_expedit_1", Compound("point_3d", "X", "Y", "Z"))
        query_object = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z"))

        smach.StateMachine.add('LOOK',
                                states.LookForObjectsAtROI(robot, query_lookat, query_object),
                                transitions={   'looking':'LOOK',
                                                'object_found':'SAY_FOUND_SOMETHING',
                                                'no_object_found':'LOOK',
                                                'abort':'SAY_ABORT'})
                                                
        smach.StateMachine.add('SAY_FOUND_SOMETHING',
                                states.Say(robot, ["I have found something"]),
                                transitions={ 'spoken':'GRAB' })

        query_grabpoint = Conjunction(  Compound("current_object", "ObjectID"),
                                        Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))
        smach.StateMachine.add('GRAB',
                                states.GrabMachine(robot.leftArm, robot, query_grabpoint),
                                transitions={   'succeeded':'SAY_DISPOSE',
                                                'failed':'SAY_PITY' })

        smach.StateMachine.add('SAY_DISPOSE',
                                states.Say(robot, ["Where can i dispose this"]),
                                transitions={ 'spoken':'LOOK_FOR_HANDLE' })

        ''' Erik: "TODO: In case of failure grasping: make state in which AMIGO asks for help: put coke can in hand manually."'''
        smach.StateMachine.add('SAY_PITY',      
                                states.Say(robot, ["That is a pity. I was not able to get the object. I will try to show some other skills anyway"]),
                                transitions={ 'spoken':'LOOK_FOR_HANDLE' })

        # ToDo: make a decent query (for both)
        query_lookat = Compound("region_of_interest", "cabinet_expedit_1", Compound("point_3d", "X", "Y", "Z"))
        query_object = Compound("position", "ObjectID", Compound("point", "X", "Y", "Z"))
        smach.StateMachine.add('LOOK_FOR_HANDLE',
                                states.LookForObjectsAtROI(robot, query_lookat, query_object),
                                transitions={   'looking':'LOOK',
                                                'object_found':'SAY_FOUND_HANDLE',
                                                'no_object_found':'LOOK',
                                                'abort':'SAY_ABORT'})

        smach.StateMachine.add('SAY_FOUND_HANDLE',
                                states.Say(robot, ["Lets put it in this drawer"]),  
                                transitions={ 'spoken':'REPOSITION' })

        smach.StateMachine.add('REPOSITION',
                                states.NavigateGeneric(robot, goal_query=query_expedit_point, look_at_path_distance=1.5),
                                transitions={   "arrived":"CARRYING_POSE_RIGHT",
                                                "unreachable":'SAY_ABORT',
                                                "preempted":'SAY_ABORT',
                                                "goal_not_defined":'SAY_ABORT'})

        smach.StateMachine.add('CARRYING_POSE_RIGHT',
                                states.Carrying_pose(robot.rightArm, robot),
                                transitions={   'succeeded':    'OPEN_RIGHT_GRIPPER',
                                                'failed':       'OPEN_RIGHT_GRIPPER'})

        smach.StateMachine.add('OPEN_RIGHT_GRIPPER',
                                states.SetGripper(robot, robot.rightArm, gripperstate=states.ArmState.OPEN),
                                transitions={ 'state_set':"SAY_FAKE_OPEN"})

        # ToDo: do we need to reposition AMIGO?
        # ToDo: define decent query
        # ToDo: this probably does not work: we need a roll around a certain axis
        query_handle = Conjunction(  Compound("current_object", "ObjectID"),
                                        Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))
        smach.StateMachine.add('GRASP_HANDLE',
                                states.Gripper_to_query_position(robot, robot.leftArm, query_handle),
                                transitions={   'succeeded':    'CLOSE_RIGHT_GRIPPER',
                                                'failed':       'SAY_ABORT', 
                                                'target_lost':  'SAY_ABORT'})

        smach.StateMachine.add('SAY_FAKE_OPEN',
                                states.Say(robot, ["I cannot really see the handle, so i'll fake it till i make it"]),
                                transitions={ 'spoken':'FAKE_GRASP_HANDLE' })

        smach.StateMachine.add('FAKE_GRASP_HANDLE',
                                states.ArmToUserPose(robot.leftArm, 5.1, 1.8, 0.6, 0, 0, 0, frame_id="/map"),
                                transitions={   'succeeded':    'CLOSE_RIGHT_GRIPPER',
                                                'failed':       'CLOSE_RIGHT_GRIPPER'})        

        smach.StateMachine.add('CLOSE_RIGHT_GRIPPER',
                                states.SetGripper(robot, robot.rightArm, gripperstate=states.ArmState.CLOSE),
                                transitions={ 'state_set':"SAY_ABORT"})

        smach.StateMachine.add('OPEN_DRAWER',
                                states.ArmToUserPose(robot.rightArm, -0.15, 0, 0, frame_id="/grippoint_right", delta=True),
                                transitions={   'succeeded':    'OPEN_LEFT_GRIPPER_FOR_DROP',
                                                'failed':       'OPEN_LEFT_GRIPPER_FOR_DROP'})

        # ToDo: move left arm above drawer: how do we do this?

        smach.StateMachine.add('OPEN_LEFT_GRIPPER_FOR_DROP',
                                states.SetGripper(robot, robot.leftArm, gripperstate=states.ArmState.OPEN, drop_from_frame=True),
                                transitions={ 'state_set':"CLOSE_LEFT_GRIPPER_AFTER_DROP"})

        smach.StateMachine.add('CLOSE_LEFT_GRIPPER_AFTER_DROP',
                                states.SetGripper(robot, robot.leftArm, gripperstate=states.ArmState.CLOSE),
                                transitions={ 'state_set':"CLEAR_LEFT_ARM"})

        smach.StateMachine.add('CLEAR_LEFT_ARM',
                                states.ArmToUserPose(robot.leftArm, 0.0, 0.2, 0, frame_id="/base_link", delta=True),
                                transitions={   'succeeded':    'CLOSE_DRAWER',
                                                'failed':       'CLOSE_DRAWER'})

        smach.StateMachine.add('CLOSE_DRAWER',
                                states.ArmToUserPose(robot.rightArm, 0.15, 0, 0, frame_id="/grippoint_right", delta=True),
                                transitions={   'succeeded':    'OPEN_LEFT_GRIPPER_FOR_DROP',
                                                'failed':       'OPEN_LEFT_GRIPPER_FOR_DROP'})

        smach.StateMachine.add("RESET_LEFT_ARM_AFTER_DROP",
                                states.ArmToUserPose(robot.leftArm, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0,
                                    time_out=20, pre_grasp=False, frame_id="reset"),
                                transitions={   "succeeded":"SAY_ABORT",
                                                "failed":"SAY_ABORT"})

        smach.StateMachine.add('OPEN_RIGHT_GRIPPER_FOR_RELEASE',
                                states.SetGripper(robot, robot.rightArm, gripperstate=states.ArmState.OPEN, drop_from_frame=True),
                                transitions={ 'state_set':"RETRACT_RIGHT_ARM"})

        smach.StateMachine.add('RETRACT_RIGHT_ARM',
                                states.ArmToUserPose(robot.rightArm, -0.15, 0, 0, frame_id="/grippoint_left", delta=True),
                                transitions={   'succeeded':    'CLOSE_RIGHT_GRIPPER_AFTER_RELEASE',
                                                'failed':       'CLOSE_RIGHT_GRIPPER_AFTER_RELEASE'})

        smach.StateMachine.add('CLOSE_RIGHT_GRIPPER_AFTER_RELEASE',
                                states.SetGripper(robot, robot.rightArm, gripperstate=states.ArmState.CLOSE),
                                transitions={ 'state_set':"RESET_RIGHT_ARM_AFTER_RELEASE"})

        smach.StateMachine.add("RESET_RIGHT_ARM_AFTER_RELEASE",
                                states.ArmToUserPose(robot.rightArm, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0,
                                    time_out=20, pre_grasp=False, frame_id="reset"),
                                transitions={   "succeeded":"SAY_FINISH",
                                                "failed":"SAY_FINISH"})

        ''' What else can we do, e.g., Human Robot Interaction '''
           
        ''' What is the fallback scenario if nothing works? '''

        smach.StateMachine.add("SAY_ABORT", 
                                states.Say(robot, ["I am not feeling very well today, so i am sorry but i will leave now"]),
                                transitions={   'spoken':'EXIT'})

        smach.StateMachine.add("SAY_FINISH", 
                                states.Say(robot, ["My work here is done, see you next time"]),
                                transitions={   'spoken':'EXIT'})

        # Move out again
        query_exit_point = Compound("base_pose", "initial", Compound("pose_2d", "X", "Y", "Phi"))
        smach.StateMachine.add('EXIT',
                                states.NavigateGeneric(robot, goal_query=query_exit_point, look_at_path_distance=1.5),
                                transitions={   "arrived":"Done",
                                                "unreachable":'SAY_STUCK',
                                                "preempted":'Aborted',
                                                "goal_not_defined":'Aborted'})

        smach.StateMachine.add("SAY_STUCK", 
                                states.Say(robot, ["I am stuck, i better send an e-mail so that my boss can help me"]),
                                transitions={   'spoken':'Done'})


    return sm

if __name__ == "__main__":
    rospy.init_node('open_challenge_executive', log_level=rospy.DEBUG)
    startup(setup_statemachine)
