#! /usr/bin/env python
import roslib; 
import rospy
import smach

from robot_smach_states.state import State

import utility
import human_interaction
from sensor_msgs.msg import LaserScan

from threading import Event

import robot_skills.util.msg_constructors as msgs
from robot_smach_states.util.designators import Designator, VariableDesignator, PointStampedOfEntityDesignator

class StartChallengeRobust(smach.StateMachine):
    """Initialize, wait for the door to be opened and drive inside"""

    def __init__(self, robot, initial_pose, use_entry_points = False):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"]) 
        assert hasattr(robot, "base")
        # assert hasattr(robot, "reasoner")
        assert hasattr(robot, "speech")

        with self:
            smach.StateMachine.add( "INITIALIZE", 
                                    utility.Initialize(robot), 
                                    transitions={   "initialized"   :"INSTRUCT_WAIT_FOR_DOOR",
                                                    "abort"         :"Aborted"})

            smach.StateMachine.add("INSTRUCT_WAIT_FOR_DOOR",
                                    human_interaction.Say(robot, [  "Hi there, I will now wait until the door is opened"], block=False),
                                    #transitions={   "spoken":"ASSESS_DOOR"})
                                    transitions={   "spoken":"DOOR_OPEN"}) # FOR NOW SKIP LASER PART



            ## !!!!!!!!!!! DOES NOT WORK: 
            rospy.logerr("TODO LOY startup.py: Add check if door is open with designators, NOW DOOR IS OPEN IS ASSUMED!! WATCH OUT!!")
            rospy.sleep(2)

            #perception.py is not there anymore
            #same for reasoning.py

            #  # Start laser sensor that may change the state of the door if the door is open:
            # smach.StateMachine.add( "ASSESS_DOOR", 
            #                         perception.Read_laser(robot, "entrance_door"),
            #                         transitions={   "laser_read":"WAIT_FOR_DOOR"})       
            
            # # define query for the question wether the door is open in the state WAIT_FOR_DOOR
            # dooropen_query = robot.reasoner.state("entrance_door","open")
        
            # # Query if the door is open:
            # smach.StateMachine.add( "WAIT_FOR_DOOR", 
            #                         reasoning.Ask_query_true(robot, dooropen_query),
            #                         transitions={   "query_false":"ASSESS_DOOR",
            #                                         "query_true":"DOOR_OPEN",
            #                                         "waiting":"DOOR_CLOSED",
            #                                         "preempted":"Aborted"})

            # If the door is still closed after certain number of iterations, defined in Ask_query_true 
            # in perception.py, amigo will speak and check again if the door is open
            # smach.StateMachine.add( "DOOR_CLOSED",
            #                         human_interaction.Say(robot, "Door is closed, please open the door"),
            #                         transitions={   "spoken":"ASSESS_DOOR"}) 

            smach.StateMachine.add( "DOOR_OPEN",
                                    human_interaction.Say(robot, "Door is open!", block=False),
                                    transitions={   "spoken":"INIT_POSE"}) 

            # Initial pose is set after opening door, otherwise snapmap will fail if door is still closed and initial pose is set,
            # since it is thinks amigo is standing in front of a wall if door is closed and localization can(/will) be messed up.
            smach.StateMachine.add('INIT_POSE',
                                utility.SetInitialPose(robot, initial_pose),
                                transitions={   'done':'ENTER_ROOM',
                                                'preempted':'Aborted',  # This transition will never happen at the moment.
                                                'error':'ENTER_ROOM'})  # It should never go to aborted.

            # Enter the arena with force drive as back-up
            smach.StateMachine.add('ENTER_ROOM',
                                    EnterArena(robot, initial_pose, use_entry_points),
                                    transitions={   "done":"Done" })
            

# Enter the arena with force drive as back-up
class EnterArena(smach.StateMachine):

    class GoToEntryPoint(State):
        def __init__(self, robot, initial_pose, use_entry_points = False):
            State.__init__(self, locals(), outcomes=["no_goal" , "found", "not_found", "all_unreachable"])

        def run(self, robot, initial_pose, use_entry_points):
            print "TODO: IMPLEMENT THIS STATE"
            return "no_goal"

    class ForceDrive(State):
        def __init__(self, robot):
            State.__init__(self, locals(), outcomes=["done"])

        def run(self, robot):            
            #self.robot.speech.speak("As a back-up scenario I will now drive through the door with my eyes closed.", block=False)  # Amigo should not say that it uses force drive, looks stupid.
            rospy.loginfo("AMIGO uses force drive as a back-up scenario!")
            robot.base.force_drive(0.25, 0, 0, 5.0)    # x, y, z, time in seconds
            return "done"

    def __init__(self, robot, initial_pose, use_entry_points = False):
        smach.StateMachine.__init__(self,outcomes=['done'])
        self.robot = robot

        with self:
            # If the door is open, amigo will say that it goes to the registration table
            smach.StateMachine.add( "THROUGH_DOOR",
                                    human_interaction.Say(robot, "I will start my task now", block=False),
                                    transitions={   "spoken":"FORCE_DRIVE_THROUGH_DOOR"}) 

            smach.StateMachine.add('FORCE_DRIVE_THROUGH_DOOR',
                                    self.ForceDrive(robot),
                                    transitions={   "done":"GO_TO_ENTRY_POINT"})

            smach.StateMachine.add('GO_TO_ENTRY_POINT',
                                    self.GoToEntryPoint(robot, initial_pose, use_entry_points),
                                    transitions={   "found":"done", 
                                                    "not_found":"GO_TO_ENTRY_POINT", 
                                                    "no_goal":"done",
                                                    "all_unreachable":"done"})


class WaitForDoorOpen(State):
    def __init__(self, robot, timeout=None):
        State.__init__(self, locals(), outcomes=["open", "closed"])
        self.laser_sub = rospy.Subscriber("/"+robot.robot_name+"/base_laser/scan", LaserScan, self.process_scan)
        self.distances = [] #TODO Loy: Keeping all of these is quite ugly. Would a ring buffer or collections.deque suffice?
        self.door_open = Event()

    def avg(self, lst):
        return sum(lst)/max(len(lst), 1)

    def process_scan(self, scan_msg):   
        try:     
            middle_index = len(scan_msg.ranges)/2  # Get the middle point
            ranges_at_center = scan_msg.ranges[middle_index-2:middle_index+2]  # Get some points around the middle
            distance_to_door = self.avg(ranges_at_center)  # and the average of the middle range and use it as the distance to the door
            self.distances += [distance_to_door] #store all distances

            avg_distance_at_start = self.avg(self.distances[:5]) #Get the first 5 distances
            avg_distance_now = self.avg(self.distances[5:]) #And the latest 5
            if len(self.distances) > 40: #Get at least 40 samples before checking the difference
                #The current distance should be more than a meter more than what we started with, then assume the door is open
                if avg_distance_now > (avg_distance_at_start + 1.0):
                    rospy.loginfo("Distance to door jumped by a meter")
                    self.door_open.set() #Then set a threading Event that run is waiting for.
        except Exception, e:
            rospy.logerr("Receiving laser failed so unsubscribing: {0}".format(e))
            self.laser_sub.unregister()

    def run(self, robot, timeout):
        rospy.loginfo("Waiting for door...")
        opened_before_timout = self.door_open.wait(timeout)
        self.door_open.clear()
        self.laser_sub.unregister()
        self.distances = []
        if opened_before_timout:
            rospy.loginfo("Door is open")
            return "opened"
        
        rospy.loginfo("Timed out with door still closed")
        return "closed"