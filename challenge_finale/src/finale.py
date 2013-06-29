#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_finale')
import rospy

import smach

import robot_smach_states as states

from robot_skills.reasoner  import Conjunction, Compound, Sequence
from robot_skills.arms import State as ArmState
from robot_smach_states.util.startup import startup

from speech_interpreter.srv import GetInfo

import geometry_msgs.msg

grasp_arm = "left"
#grasp_arm = "right"

##########################################
############## What to run: ##############
##########################################
# CHECK the README!

class StupidHumanDropoff(smach.StateMachine):
    def __init__(self, arm, robot, dropoff_query):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed', 'target_lost'])

        with self:
            smach.StateMachine.add( "DROPOFF_OBJECT",
                                    states.PrepareOrientation(arm, robot, dropoff_query),
                                    transitions={   'orientation_succeeded':'ASK_TAKE_FROM_HAND',
                                                    'orientation_failed':'ASK_TAKE_FROM_HAND',
                                                    'abort':'failed',
                                                    'target_lost':'target_lost'})

            smach.StateMachine.add("ASK_TAKE_FROM_HAND", 
                                    states.Say(robot, ["Please take this from my hand, I'm not confident that I can place to object safely"]),
                                    transitions={   'spoken':'HANDOVER_TO_HUMAN_1'})

            smach.StateMachine.add("HANDOVER_TO_HUMAN_1", 
                                    states.Say(robot, [ "Be careful, I will open my gripper now"]),
                                    transitions={   'spoken':'OPEN_GRIPPER_HANDOVER'})

            smach.StateMachine.add('OPEN_GRIPPER_HANDOVER', 
                                    states.SetGripper(robot, arm, gripperstate=ArmState.OPEN),
                                    transitions={'succeeded'    :   'SAY_PLACE_INSTRUCTION',
                                                 'failed'       :   'SAY_PLACE_INSTRUCTION'})

            def generate_object_sentence(*args,**kwargs):
                try:
                    answers = robot.reasoner.query(dropoff_query)
                    _type = answers[0]["ObjectType"]
                    dropoff = answers[0]["Disposal_type"]
                    return "Please put the {0} on the {1}".format(_type, dropoff).replace("_", " ")
                except Exception, e:
                    rospy.logerr(e)
                    return "Please put the object on the surface in front of me"
            smach.StateMachine.add("SAY_PLACE_INSTRUCTION", 
                                    states.Say_generated(robot, generate_object_sentence),
                                    transitions={   'spoken':'CLOSE_GRIPPER_HANDOVER'})

            smach.StateMachine.add('CLOSE_GRIPPER_HANDOVER', 
                                    states.SetGripper(robot, arm, gripperstate=ArmState.CLOSE),
                                    transitions={'succeeded'    :   'RESET_ARM',
                                                 'failed'       :   'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM', 
                                    states.ArmToJointPos(robot, arm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), #Copied from demo_executioner NORMAL
                                    transitions={   'done'      :'RESET_TORSO',
                                                  'failed'      :'RESET_TORSO'    })

            smach.StateMachine.add('RESET_TORSO',
                                    states.ResetTorso(robot),
                                    transitions={'succeeded'    :'succeeded',
                                                 'failed'       :'failed'})

class ScanTables(smach.State):
    def __init__(self, robot, timeout_duration):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot
        self.timeout_duration = timeout_duration

    def execute(self, gl):

        rospy.loginfo("Trying to detect objects on tables")

        answers = self.robot.reasoner.query(Compound('region_of_interest', 
            'large_table_1', Compound('point_3d', 'X', 'Y', 'Z'), Compound('point_3d', 'Length_x', 'Length_y', 'Length_z')))
        
        ''' Remember current spindle position '''      
        spindle_pos = self.robot.spindle.get_position()


        if answers:
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            target_point = geometry_msgs.msg.PointStamped()
            target_point.header.frame_id = "/map"
            target_point.header.stamp = rospy.Time()
            target_point.point.x = float(answer["X"])
            target_point.point.y = float(answer["Y"])
            target_point.point.z = float(answer["Z"])

            ''' If height is feasible for LRF, use this. Else: use head and tabletop/clustering '''
            if self.robot.spindle.send_laser_goal(float(answer["Z"]), timeout=self.timeout_duration):
                self.robot.speech.speak("I will scan the tables for objects", block=False)
                self.robot.perception.toggle_perception_2d(target_point, answer["Length_x"], answer["Length_y"], answer["Length_z"])
                rospy.logwarn("Here we should keep track of the uncertainty, how can we do that? Now we simply use a sleep")
                rospy.logwarn("Waiting for 2.0 seconds for laser update")
                rospy.sleep(rospy.Duration(2.0))
            else:
                rospy.logerr("Can't scan on spindle height, either the spindle timeout exceeded or ROI too low. Will have to move to prior location")
            
            ''' Reset head and stop all perception stuff '''
            self.robot.perception.toggle([])
            self.robot.spindle.send_goal(spindle_pos, waittime=self.timeout_duration)
        else:
            rospy.logerr("No table location found...")

        return 'succeeded'

class DetermineGoal(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"], input_keys=['object_locations'], output_keys=['object_locations'])
        self.robot = robot
        self.preempted = False

    def execute(self, userdata):

        query = Conjunction(
                 Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")), 
                 Compound( "not", Compound("property_expected", "ObjectID", "class_label", "Class")))

        answers = self.robot.reasoner.query(query)

        self.robot.speech.speak("I have found {0} possible object locations".format(len(answers)))

        (position, orientation) = self.robot.base.get_location()
        counter = 0
        location_list = []
        for answer in answers:
            location_list.append({'X' : answer['X'], 'Y' : answer['Y'], 'Z': answer['Z']})
            location_list = sorted(location_list, key=lambda p: (position.y - float(p['Y']))**2 + (position.x - float(p['X']))**2)
        for point in location_list:
            self.robot.reasoner.assertz(Compound("goal_location", ("a" + str(counter)), Compound("point_3d", point["X"], point["Y"], point["Z"])))
            counter += 1
        
        return "done"

class MoveToTable(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done", "failed_navigate", "no_tables_left"])
        self.robot = robot

        with self:
            smach.StateMachine.add("GET_LOCATION", 
                GetNextLocation(self.robot),
                transitions={'done':'NAVIGATE_TO', 'no_locations':'no_tables_left'})

            smach.StateMachine.add("NAVIGATE_TO", states.NavigateGeneric(robot, 
                lookat_query=Compound("base_grasp_point", "ObjectID", Compound("point_3d", "X", "Y", "Z"))), 
                transitions={'unreachable' : 'failed_navigate', 'preempted' : 'NAVIGATE_TO', 
                'arrived' : 'done', 'goal_not_defined' : 'failed_navigate'})

class Finale(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])
        self.robot = robot
        if grasp_arm == "right": 
            arm = robot.rightArm
        else:            
            arm = robot.leftArm

        #retract old facts
        robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
        robot.reasoner.query(Compound("retractall", Compound("goal", "X")))
        robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
        robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))
        robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
        robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
        robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
        robot.reasoner.query(Compound("retractall", Compound("disposed", "X")))
        
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
	
	    #robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/cleanup_test.pl'))
        #Assert the current challenge.
        robot.reasoner.assertz(Compound("challenge", "finale"))

        # query_unkown_object = Conjunction( Compound("goal", Compound("clean_up", "Room")),
        #                                                 Compound("exploration_target", "Room", "Target"),
        #                                                 Compound("not", Compound("explored", "Target")),
        #                                                 Compound("waypoint", "Target", Compound("pose_2d", "X", "Y", "Phi"))
                                                       # )
        query_unkown_object = Conjunction(
                 Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")), 
                 Compound( "not", Compound("property_expected", "ObjectID", "class_label", "Class")),
                 Compound( "not", Compound("explored", "ObjectID")))

        query_exploration_target = Conjunction( Compound("current_exploration_target", "ObjectID"), 
                                                Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        query_lookat = Conjunction( Compound("current_exploration_target", "Target"),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        #Make sure the object we're dealing with isn't already disposed (i.e. handled for cleanup)
        #After cleaning the object up/disposing it, 
        #MARK_DISPOSED asserts disposed(current_objectID)
        query_object = Conjunction(
                            Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")),
                            Compound("not", Compound("disposed", "ObjectID")))

        query_grabpoint = Conjunction(  Compound("current_object", "ObjectID"),
                                        Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))

        query_current_object_class = Conjunction(
                                Compound("current_object",      "Obj_to_Dispose"), #Of the current object
                                Compound("instance_of",         "Obj_to_Dispose",   Compound("exact", "ObjectType")))

        query_dropoff_loc = Compound("dropoff_point",  "trash_bin", Compound("point_3d", "X", "Y", "Z"))        

        meeting_point = Conjunction(    Compound("waypoint", Compound("meeting_point", "Waypoint"), Compound("pose_2d", "X", "Y", "Phi")),
                                        Compound("not", Compound("unreachable", Compound("meeting_point", "Waypoint"))))

        with self:
            smach.StateMachine.add('INITIALIZE',
                            states.Initialize(robot),
                            transitions={'initialized':'INIT_POSE',
                                         'abort':'Aborted'})

            smach.StateMachine.add('INIT_POSE',
                            states.Set_initial_pose(robot, "custom_initial"),
                            transitions={   'done':'SAY_START',
                                            'preempted':'Aborted',
                                            'error':'Aborted'})

            smach.StateMachine.add("SAY_START", 
                                    states.Say(robot, ["Lets start with the final demonstration, I'm very excited!"], block=False),
                                    transitions={   'spoken':'MOVE_TO_SCAN_POS'})

            smach.StateMachine.add("MOVE_TO_SCAN_POS", 
                        states.NavigateGeneric(robot, goal_pose_2d=(2.06, -4.433, -1.13)),
                        transitions={   'unreachable'       : 'SCAN_TABLES', 
                                        'preempted'         : 'SCAN_TABLES', 
                                        'arrived'           : 'SCAN_TABLES', 
                                        'goal_not_defined'  : 'SCAN_TABLES'})

            # After this state: objects might be in the world model
            smach.StateMachine.add("SCAN_TABLES", 
                                ScanTables(robot, 10.0),
                                transitions={   'succeeded':'DETERMINE_EXPLORATION_TARGET'})

            @smach.cb_interface(outcomes=['found_exploration_target', 'done'], 
                                input_keys=[], 
                                output_keys=[])
            def determine_exploration_target(userdata):            
                # Ask the reaoner for an exploration target that is:
                #  - in the room that needs cleaning up
                #  - not yet explored
                answers = robot.reasoner.query(query_unkown_object)
                for answer in answers:
                    rospy.loginfo("Answers for {0}: {1} \n\n".format(query_unkown_object, answer))
                # First time: 
                # [   {'Y': 1.351, 'X': 4.952, 'Phi': 1.57, 'Room': living_room, 'Target': cabinet_expedit_1}, 
                #     {'Y': -1.598, 'X': 6.058, 'Phi': 3.113, 'Room': living_room, 'Target': bed_1}]
                # ----
                # 2nd:
                # [{'Y': -1.598, 'X': 6.058, 'Phi': 3.113, 'Room': living_room, 'Target': bed_1}]
                #
                #import ipdb; ipdb.set_trace()
                if not answers:
                    # no more exporation targets found
                    return 'done'
                else:         
                    # TODO: pick target based on some metric
                    def calc_dist((xA,yA), (xB,yB)):
                        import math
                        dist = math.sqrt(abs(xA-xB)**2 + abs(yA-yB)**2)
                        return dist
                    
                    loc = self.robot.base.location[0]
                    

                    robot_xy = (loc.x, loc.y)
                    closest_QA = min(answers, key=lambda ans: calc_dist(robot_xy, (float(ans["X"]), float(ans["Y"]))))
                    target = closest_QA["ObjectID"]

                    rospy.loginfo("Minimum distance: {0}".format(closest_QA))

                    #rospy.loginfo("Available targets: {0}".format(answers))
                    rospy.loginfo("Selected target: {0}".format(target))
                    #target = answers[0]["Target"]

                    # remove current target
                    robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))

                    # add new target
                    robot.reasoner.assertz(Compound("current_exploration_target", target))
                    answers = robot.reasoner.query(query_exploration_target)

                    # Not so nice, but works for now: (TODO: add the fact if the target is actually explored)
                    robot.reasoner.assertz(Compound("explored", target))
                    
                    #robot.speech.speak("Lets go look at the object with ID {0}".format(target))
                    robot.speech.speak("Let's go look at an object I found!".format(target), block=False)

                    return 'found_exploration_target'
            
            smach.StateMachine.add('DETERMINE_EXPLORATION_TARGET', smach.CBState(determine_exploration_target),
                                    transitions={   'found_exploration_target':'DRIVE_TO_EXPLORATION_TARGET',
                                                    'done':'SAY_ALL_EXPLORED'})

            ################################################################
            smach.StateMachine.add( 'DRIVE_TO_EXPLORATION_TARGET',
                                    states.PrepareOrientation(arm, robot, grabpoint_query=query_exploration_target),
                                    transitions={   "orientation_succeeded":"SAY_LOOK_FOR_OBJECTS",
                                                    "orientation_failed":'SAY_GOAL_UNREACHABLE',
                                                    "abort":'Aborted',
                                                    "target_lost":'DETERMINE_EXPLORATION_TARGET'})

            def generate_unreachable_sentence(*args,**kwargs):
                try:
                    answers = robot.reasoner.query(query_exploration_target)
                    name = answers[0]["Target"] #Should only have 1 answer
                    return "The object is unreachable, where else can I go?".format(name)
                except Exception, e:
                    rospy.logerr(e)
                    return "Something went terribly wrong, I don't know where to go and it's unreachable too"
            smach.StateMachine.add('SAY_GOAL_UNREACHABLE',
                                    states.Say_generated(robot, sentence_creator=generate_unreachable_sentence),
                                    transitions={ 'spoken':'DETERMINE_EXPLORATION_TARGET' })

            smach.StateMachine.add("SAY_LOOK_FOR_OBJECTS", 
                                    states.Say(robot, ["Lets see what I can find here."]),
                                    transitions={   'spoken':'LOOK'})

            smach.StateMachine.add('LOOK',
                                    states.LookForObjectsAtROI(robot, query_lookat, query_object),
                                    transitions={   'looking':'LOOK',
                                                    'object_found':'SAY_FOUND_SOMETHING',
                                                    'no_object_found':'SAY_FOUND_NOTHING',
                                                    'abort':'Aborted'})

            smach.StateMachine.add('SAY_FOUND_NOTHING',
                                    states.Say(robot, ["I didn't find anything to clean up here", "No objects to clean here", "There are no objects to clean here"]),
                                    transitions={ 'spoken':'DETERMINE_EXPLORATION_TARGET' })

            def generate_object_sentence(*args,**kwargs):
                try:
                    answers = robot.reasoner.query(query_dropoff_loc)
                    _type = answers[0]["ObjectType"]
                    return "I have found a {0}. I'll dump it in the trash bin".format(_type)
                except Exception, e:
                    rospy.logerr(e)
                    try:
                        type_only = robot.reasoner.query(query_current_object_class)[0]["ObjectType"]
                        return "I found a {0}.".format(type_only)
                    except Exception, e:
                        rospy.logerr(e)
                        pass
                    return "I have found something, but I'm not sure what it is. I'll toss in in the trash bin"
            smach.StateMachine.add('SAY_FOUND_SOMETHING',
                                    states.Say_generated(robot, sentence_creator=generate_object_sentence),
                                    transitions={ 'spoken':'GRAB' })

            smach.StateMachine.add('GRAB',
                                    states.GrabMachine(arm, robot, query_grabpoint),
                                    transitions={   'succeeded':'DROPOFF_OBJECT',
                                                    'failed':'HUMAN_HANDOVER' })
            
            smach.StateMachine.add('HUMAN_HANDOVER',
                                    states.Human_handover(arm,robot),
                                    transitions={   'succeeded':'RESET_HEAD',
                                                    'failed':'DETERMINE_EXPLORATION_TARGET'})
        
            @smach.cb_interface(outcomes=["done"])
            def reset_head(*args, **kwargs):
                robot.head.reset_position()
                return "done"   
            smach.StateMachine.add( "RESET_HEAD", 
                        smach.CBState(reset_head),
                        transitions={"done":"DROPOFF_OBJECT"})

            smach.StateMachine.add("DROPOFF_OBJECT",
                                    states.DropObject(arm, robot, query_dropoff_loc),
                                    transitions={   'succeeded':'MARK_DISPOSED',
                                                    'failed':'MARK_DISPOSED',
                                                    'target_lost':'GOTO_HUMAN_DROPOFF'})

            # smach.StateMachine.add("DROPOFF_OBJECT",
            #                         StupidHumanDropoff(arm, robot, query_dropoff_loc),
            #                         transitions={   'succeeded':'MARK_DISPOSED',
            #                                         'failed':'MARK_DISPOSED',
            #                                         'target_lost':'GOTO_HUMAN_DROPOFF'})

            smach.StateMachine.add( 'GOTO_HUMAN_DROPOFF', states.NavigateGeneric(robot, goal_query=meeting_point),
                                    transitions={   "arrived":"SAY_PLEASE_TAKE",
                                                    "unreachable":'SAY_PLEASE_TAKE', #Maybe this should not be "FINISHED?"
                                                    "preempted":'SAY_PLEASE_TAKE',
                                                    "goal_not_defined":'SAY_PLEASE_TAKE'})

            smach.StateMachine.add("SAY_PLEASE_TAKE", 
                                    states.Say(robot, "Please take this thing from my hand. I don't know where to put it"),
                                    transitions={   'spoken':'HANDOVER_TO_HUMAN'})

            smach.StateMachine.add("HANDOVER_TO_HUMAN",
                                    states.HandoverToHuman(arm, robot),
                                    transitions={   'succeeded':'MARK_DISPOSED',
                                                    'failed':'MARK_DISPOSED'})

            #Mark the current_object as disposed
            @smach.cb_interface(outcomes=['done'])
            def deactivate_current_object(userdata):
                try:
                    #robot.speech.speak("I need some debugging in cleanup, please think with me here.")
                    #import ipdb; ipdb.set_trace()
                    objectID = robot.reasoner.query(Compound("current_object", "Disposed_ObjectID"))[0]["Disposed_ObjectID"]
                    disposed = Compound("disposed", objectID)
                    robot.reasoner.assertz(disposed)
                except:
                    pass #Just continue
                return 'done'
            smach.StateMachine.add('MARK_DISPOSED', smach.CBState(deactivate_current_object),
                                    transitions={'done':'DETERMINE_EXPLORATION_TARGET'})

            smach.StateMachine.add("SAY_ALL_EXPLORED", 
                                    states.Say(robot, [ "I searched at all object locations, so cleaning is done.", 
                                                        "All object locations I found with my laser are explored, there is nothing I can find anymore", 
                                                        "All object locations I found with my laser are explored, there are no locations to search anymore"]),
                                    transitions={   'spoken':'RETURN'})
                    
            smach.StateMachine.add( 'RETURN', states.NavigateGeneric(robot, goal_name="exitB"),
                                    transitions={   "arrived":"SAY_DONE",
                                                    "unreachable":'SAY_DONE', #Maybe this should not be "FINISHED?"
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'Aborted'})

            smach.StateMachine.add("SAY_DONE", 
                                    states.Say(robot, ["I cleaned up everything I could find, so my work here is done. Have a nice day!", "I'm done, everything I could find is cleaned up."]),
                                    transitions={   'spoken':'FINISH'})

            smach.StateMachine.add( 'FINISH', states.Finish(robot),
                                    transitions={'stop':'Done'})

if __name__ == "__main__":
    rospy.init_node('finale_exec')
    
    startup(Finale)
