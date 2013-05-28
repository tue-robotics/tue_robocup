#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cleanup')
import rospy

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_skills.reasoner  import Conjunction, Compound
from robot_smach_states.util.startup import startup

from speech_interpreter.srv import GetInfo

##########################################
############## What to run: ##############
##########################################
# - astart
# - amiddle
# - roslaunch create_speech_files speech.launch   (in tue_test_lab the launch file is: speech_tue_test_lab.launch)
# - !! Wait for speech.launch to finish before !!
#   !!   launching speech interpreter          !!
#   roslaunch speech_interpreter start.launch     (in tue_test_lab the launch file is: speech_tue_test_lab.launch)
# - rosrun challenge_cleanup clean_up.py

class Ask_cleanup(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["done"])

        self.robot = robot
        self.get_cleanup_service = rospy.ServiceProxy('interpreter/get_info_user', GetInfo)

    def execute(self, userdata):
        self.robot.head.look_up()

        self.response = self.get_cleanup_service("room_cleanup", 4 , 60)  # This means that within 4 tries and within 60 seconds an answer is received. 
        room = "livingroom"
        if self.response.answer == "no_answer" or self.response.answer == "wrong_answer":
            room = "kitchen"
        elif self.response.answer == "livingroom":
            room = "living_room"
        elif self.response.answer == "diningroom":
            room = "dining_room"
        elif self.response.answer == "kitchen":
            room = "kitchen"
        elif self.response.answer == "bedroom":
            room = "bedroom"
        else:
            self.robot.speech.speak("I'll clean the livingroom, humans always tend to make a mess of that.")
            room = "living_room"

        self.robot.reasoner.query(Compound("assertz", Compound("goal", Compound("clean_up", room))))
            
        return "done"

class Cleanup(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

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
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'magdeburg2013_knowledge.pl'))
	
	    #robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/cleanup_test.pl'))
        #Assert the current challenge.
        robot.reasoner.assertz(Compound("challenge", "clean_up"))


        query_meeting_point = Compound("waypoint", 
                                Compound("meeting_point", "Waypoint"), 
                                Compound("pose_2d", "X", "Y", "Phi"))

        query_exploration_target_in_room = Conjunction( Compound("goal", Compound("clean_up", "Room")),
                                                        Compound("exploration_target", "Room", "Target"),
                                                        Compound("not", Compound("explored", "Target")),
                                                        Compound("base_pose", "Target", Compound("pose_2d", "X", "Y", "Phi"))
                                                       )
        query_room = Conjunction(   Compound("goal", Compound("clean_up", "Room")), 
                                    Compound("waypoint", "Room", Compound("pose_2d", "X", "Y", "Phi"))) 

        query_exploration_target = Conjunction( Compound("current_exploration_target", "Target"),
                                                Compound("base_pose", "Target", Compound("pose_2d", "X", "Y", "Phi")))

        query_lookat = Conjunction( Compound("current_exploration_target", "Target"),
                                    Compound("point_of_interest", "Target", Compound("point_3d", "X", "Y", "Z")))

        #Make sure the object we're dealing with isn't already disposed (i.e. handled for cleanup)
        #After cleaning the object up/disposing it, 
        #MARK_DISPOSED asserts disposed(current_objectID)
        query_object = Conjunction(
                            Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")),
                            Compound("not", Compound("disposed", "ObjectID")))

        query_grabpoint = Conjunction(  Compound("current_object", "ObjectID"),
                                            Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))

        query_current_object_class = Conjunction(
                                Compound("current_object",      "Obj_to_Dispose"), #Of the current object
                                Compound("instance_of",         "Obj_to_Dispose",   Compound("exact", "ObjectType")))

        query_dropoff_loc = Conjunction(
                                Compound("current_object",      "Obj_to_Dispose"), #Of the current object
                                Compound("instance_of",                "Obj_to_Dispose",   Compound("exact", "ObjectType")), #Gets its type
                                Compound("storage_class",       "ObjectType",       "Disposal_type"), #Find AT what sort of thing it should be disposed, e.g. a trashbin
                                Compound("instance_of",                "Dispose_to_object",  "Disposal_type"), #Find objects of that are of type trashbin
                                Compound("point_of_interest",  "Dispose_to_object", Compound("point_3d", "X", "Y", "Z"))) #Get locations of those things

        query_dropoff_loc_backup = Conjunction( Compound("instance_of", "Dispose_to_object",  "trashbin"), #Find objects of that are of type trashbin
                                                    Compound("point_of_interest",  "Dispose_to_object",  Compound("point_3d", "X", "Y", "Z"))) #Get locations of those things

        meeting_point = Conjunction(    Compound("waypoint", Compound("meeting_point", "Waypoint"), Compound("pose_2d", "X", "Y", "Phi")),
                                        Compound("not", Compound("unreachable", Compound("meeting_point", "Waypoint"))))

        with self:

            smach.StateMachine.add( "START_CHALLENGE",
                                    states.StartChallengeRobust(robot, "initial"), 
                                    transitions={   "Done":"ASK_CLEANUP", 
                                                    "Aborted":"Aborted", 
                                                    "Failed":"CANNOT_GOTO_MEETINGPOINT"})

            smach.StateMachine.add("CANNOT_GOTO_MEETINGPOINT", 
                                    states.Say(robot, [ "I can't find a way to the meeting point. Please teach me the correct position and clear the path to it", 
                                                        "I couldn't even get to my first waypoint. May I try again?", 
                                                        "This ended before I could get started, because my first waypoint is unreachable."]),
                                    transitions={   'spoken':'Aborted'})

            smach.StateMachine.add("ASK_CLEANUP",
                                Ask_cleanup(robot),
                                transitions={'done':'DETERMINE_EXPLORATION_TARGET'})
            
            ################################################################
            #                  DETERMINE_EXPLORATION_TARGET
            ################################################################

            @smach.cb_interface(outcomes=['found_exploration_target', 'done'], 
                                input_keys=[], 
                                output_keys=[])
            def determine_exploration_target(userdata):            
                # Ask the reaoner for an exploration target that is:
                #  - in the room that needs cleaning up
                #  - not yet explored
                answers = robot.reasoner.query(query_exploration_target_in_room)
                rospy.loginfo("Answers for {0}: {1}".format(query_exploration_target_in_room, answers))
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
                    
                    loc = robot.base.location[0]
                    robot_xy = (loc.x, loc.y)
                    closest_QA = min(answers, key=lambda ans: calc_dist(robot_xy, (float(ans["X"]), float(ans["Y"]))))
                    target = closest_QA["Target"]
                    rospy.loginfo("Available targets: {0}".format(answers))
                    rospy.loginfo("Selected target: {0}".format(target))
                    #target = answers[0]["Target"]

                    # remove current target
                    robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))

                    # add new target
                    robot.reasoner.assertz(Compound("current_exploration_target", target))

                    # Not so nice, but works for now: (TODO: add the fact if the target is actually explored)
                    robot.reasoner.assertz(Compound("explored", target))                

                    return 'found_exploration_target'
            
            smach.StateMachine.add('DETERMINE_EXPLORATION_TARGET', smach.CBState(determine_exploration_target),
                                    transitions={   'found_exploration_target':'DRIVE_TO_EXPLORATION_TARGET',
                                                    'done':'SAY_ALL_EXPLORED'})

            ################################################################
            smach.StateMachine.add( 'DRIVE_TO_EXPLORATION_TARGET',
                                    states.NavigateGeneric(robot, goal_query=query_exploration_target),
                                    transitions={   "arrived":"SAY_LOOK_FOR_OBJECTS",
                                                    "unreachable":'DETERMINE_EXPLORATION_TARGET',
                                                    "preempted":'Aborted',
                                                    "goal_not_defined":'DETERMINE_EXPLORATION_TARGET'})

            smach.StateMachine.add("SAY_LOOK_FOR_OBJECTS", 
                                    states.Say(robot, ["Lets see what I can find here."]),
                                    transitions={   'spoken':'LOOK'})

            #query_dropoff_loc = Compound("point_of_interest", "trashbin1", Compound("point_3d", "X", "Y", "Z"))
            # 
            # Test this by: 
            # console 1: $ rosrun tue_reasoner_core reasoner
            # console 2: $ roslaunch wire_core start.launch
            # console 3: $ amigo-console
            # r.query(Compound("consult", '~/ros/fuerte/tue/trunk/tue_reasoner/tue_knowledge/prolog/cleanup_test.pl'))
            # r.query(Compound("consult", '~/ros/fuerte/tue/trunk/tue_reasoner/tue_knowledge/prolog/locations.pl'))
            # r.query(Compound("consult", '~/ros/fuerte/tue/trunk/tue_reasoner/tue_knowledge/prolog/objects.pl'))
            # r.assertz(Compound("challenge", "clean_up"))
            # r.assertz(Compound("environment", "tue_test_lab"))
            # r.query(r.dispose("X", "Y", "Z"))
            # This finally returns a list of (all the same) XYZ-coords.
            # If you enter query_dropoff_loc below into the amigo-console, 
            #   you can verify that it returns the same coords, but with more variables of course.

            smach.StateMachine.add('LOOK',
                                    states.LookForObjectsAtROI(robot, query_lookat, query_object),
                                    transitions={   'looking':'LOOK',
                                                    'object_found':'SAY_FOUND_SOMETHING',
                                                    'no_object_found':'DETERMINE_EXPLORATION_TARGET',
                                                    'abort':'Aborted'})
            def generate_object_sentence(*args,**kwargs):
                try:
                    answers = robot.reasoner.query(query_dropoff_loc)
                    _type = answers[0]["ObjectType"]
                    dropoff = answers[0]["Disposal_type"]
                    return "I have found a {0}. I'll' dispose it to a {1}".format(_type, dropoff)
                except Exception, e:
                    rospy.logerr(e)
                    try:
                        type_only = robot.reasoner.query(query_current_object_class)[0]["ObjectType"]
                        return "I found something called {0}.".format(type_only)
                    except Exception, e:
                        rospy.logerr(e)
                        pass
                    return "I have found something, but I'm not sure what it is."
            smach.StateMachine.add('SAY_FOUND_SOMETHING',
                                    states.Say_generated(robot, sentence_creator=generate_object_sentence),
                                    transitions={ 'spoken':'GRAB' })

            smach.StateMachine.add('GRAB',
                                    states.GrabMachine(robot.leftArm, robot, query_grabpoint),
                                    transitions={   'succeeded':'DROPOFF_OBJECT',
                                                    'failed':'HUMAN_HANDOVER' })
            
            smach.StateMachine.add('HUMAN_HANDOVER',
                                    states.Human_handover(robot.leftArm,robot),
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
                                    states.Gripper_to_query_position(robot, robot.leftArm, query_dropoff_loc),
                                    transitions={   'succeeded':'DROP_OBJECT',
                                                    'failed':'DROP_OBJECT',
                                                    'target_lost':'DONT_KNOW_DROP'})
            
            smach.StateMachine.add("DONT_KNOW_DROP", 
                                    states.Say(robot, "Now that I fetched this, I'm not sure where to put it. I'll just give it to a human, they'll know what to do!"),
                                    transitions={   'spoken':'GOTO_HUMAN_DROPOFF'}) #TODO: Dont abort, do something smart!

            # smach.StateMachine.add("DROPOFF_OBJECT_BACKUP",
            #                         states.Gripper_to_query_position(robot, robot.leftArm, query_dropoff_loc_backup),
            #                         transitions={   'succeeded':'DROP_OBJECT',
            #                                         'failed':'DROP_OBJECT',
            #                                         'target_lost':'DONT_KNOW_DROP_BACKUP'})

            # smach.StateMachine.add("DONT_KNOW_DROP_BACKUP", 
            #                         states.Say(robot, "Now that I fetched this, I don't know where to put it. Silly me!"),
            #                         transitions={   'spoken':'GOTO_HUMAN_DROPOFF'})

            smach.StateMachine.add( 'GOTO_HUMAN_DROPOFF', states.NavigateGeneric(robot, goal_query=meeting_point),
                                    transitions={   "arrived":"SAY_PLEASE_TAKE",
                                                    "unreachable":'SAY_PLEASE_TAKE', #Maybe this should not be "FINISHED?"
                                                    "preempted":'SAY_PLEASE_TAKE',
                                                    "goal_not_defined":'SAY_PLEASE_TAKE'})

            smach.StateMachine.add("SAY_PLEASE_TAKE", 
                                    states.Say(robot, "Please take this thing from my hand. I don't know where to put it"),
                                    transitions={   'spoken':'DETERMINE_EXPLORATION_TARGET'})


            # smach.StateMachine.add( 'DRIVE_TO_DROPOFF',
            #                         states.Navigate_to_queryoutcome(robot, query_dropoff_loc, X="X", Y="Y", Phi="Phi"),
            #                         transitions={   "arrived":"PLACE_OBJECT",
            #                                         "unreachable":'RETURN',
            #                                         "preempted":'Aborted',
            #                                         "goal_not_defined":'Aborted'})                
            
            # smach.StateMachine.add( 'PLACE_OBJECT', states.Place_Object(robot.leftArm,robot),
            #                         transitions={   'object_placed':'CARR_POS2'})
            smach.StateMachine.add( 'DROP_OBJECT', states.SetGripper(robot, robot.leftArm, gripperstate=0, drop_from_frame="/grippoint_left"), #open
                                    transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                    'failed'   :'CLOSE_AFTER_DROP'})
            smach.StateMachine.add( 'CLOSE_AFTER_DROP', states.SetGripper(robot, robot.leftArm, gripperstate=1), #close
                                    transitions={   'succeeded':'RESET_ARM',
                                                    'failed'   :'RESET_ARM'})
            smach.StateMachine.add('RESET_ARM', 
                                    states.ArmToPose(robot, robot.leftArm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), #Copied from demo_executioner NORMAL
                                    transitions={   'done':'MARK_DISPOSED',
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
                                    states.Say(robot, [ "I searched at all locations I know of, so cleaning is done.", 
                                                        "All locations I know of are explored, there is nothing I can find anymore", 
                                                        "All locations I know of are explored, there are no locations to search anymore"]),
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
    rospy.init_node('clean_up_exec')
    
    startup(Cleanup)
