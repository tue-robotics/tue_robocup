#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_robo_zoo')
import rospy

import smach

import robot_smach_states as states
from robot_smach_states.util.startup import startup

from psi import Compound, Conjunction, Sequence
from robot_skills.arms import State as ArmState
from robot_skills.util import msg_constructors as msg
from robot_skills.util import transformations


""" TODOs, BUGs:
- DONE:reset pose after grasping a can is in collision, move the arm a bit
- DONE: don't pick up empty cans, takes too much time
- DONE: waiting for person doesn't work, starts placing directly
"""

from visualization_msgs.msg import Marker, MarkerArray
publisher = rospy.Publisher('visualization_markers', MarkerArray)
markerArray = MarkerArray()
ids = 0
def publish_marker(point_stamped, color=(0,0,1)):
    global ids
    marker = Marker()
    marker.id = ids
    ids += 1
    marker.header.frame_id = point_stamped.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.SPHERE
    marker.pose.position.x = point_stamped.point.x
    marker.pose.position.y = point_stamped.point.y
    marker.pose.position.z = point_stamped.point.z
    marker.pose.orientation.w = 1
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    markerArray.markers.append(marker)
    # Publish the MarkerArray
    publisher.publish(markerArray)

class ClearCostmapAround(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot

    def execute(self, userdata=None):
        b = self.robot.base
        #b.force_drive(0.25, 0, 0, 3)
        b.force_drive(0, 0, 0.5, 12) #turn yourself around
        #b.force_drive(-0.25, 0, 0, 3)
        return "Done"


class GrabClog(smach.StateMachine):
    def __init__(self, robot, side):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot
        self.side = side

        query_clog_cup = Conjunction(
                                    Compound( "property_expected", "ObjectID", "class_label", "water"),#TODO: class name noodles may need to be changed to cup
                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))
        with self:
            smach.StateMachine.add( "ASK_FOR_CLOG",
                                    states.Say(robot, ["Let me get some clogs!"], 
                                               block=True),
                                    transitions={'spoken': "GRAB_CLOG_CUP"})
            
            smach.StateMachine.add( "GRAB_CLOG_CUP",
                                    states.GrabMachine(side, robot, query_clog_cup),
                                    transitions={   'succeeded'         :'Done',
                                                    'failed'            :'Failed' })

class ReceiveClog(smach.StateMachine):
    def __init__(self, robot, side):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot
        self.side = side

        with self:
            smach.StateMachine.add( "ASK_FOR_CLOG",
                                    states.Say(robot, ["Can I have clogs?", "Please hand me some clogs", "Can I have some clogs?", "Clogs please"], 
                                               block=True),
                                    transitions={'spoken': "HOLD_ARM_FOR_CLOG2"})
            
            #Upper arm back a bit: -0.00, -0.39, 0.00, 0.00, 0.00, 0.00, 0.00
            #Then the lower arm up: -0.00, -0.39, 0.00, 1.93, 0.00, 0.00, 0.00
            #Then to -0.1, -0.4, 0.00, 1.93, 0.000, 0.00, 0.00
            # smach.StateMachine.add( "HOLD_ARM_FOR_CLOG1",
            #                         states.ArmToJointPos(robot, side, [-0.00, -0.39, 0.00, 0.00, 0.00, 0.00, 0.00], timeout=4), #TODO: Set correct pose
            #                         transitions={   'done'              :'HOLD_ARM_FOR_CLOG2',
            #                                         'failed'            :'RESET_ARM1' })
            
            smach.StateMachine.add( "HOLD_ARM_FOR_CLOG2",
                                    states.ArmToJointPos(robot, side, [0.00, -0.39, 0.00, 1.93, 0.00, 0.00, 0.00], timeout=4), #TODO: Set correct pose
                                    transitions={   'done'              :'HOLD_ARM_FOR_CLOG3',
                                                    'failed'            :'RESET_ARM1' })
            
            smach.StateMachine.add( "HOLD_ARM_FOR_CLOG3",
                                    states.ArmToJointPos(robot, side, [-0.1, -0.4, 0.00, 1.93, 0.000, 0.00, 0.00], timeout=4), #TODO: Set correct pose
                                    transitions={   'done'              :'LOOK_AT_HAND',
                                                    'failed'            :'RESET_ARM1' })
            
            smach.StateMachine.add( "LOOK_AT_HAND",
                                    states.LookAtHand(robot, side), #TODO: Set correct pose
                                    transitions={   'done'              :'OPEN_GRIPPER'})
            
            smach.StateMachine.add( "OPEN_GRIPPER",
                                    states.SetGripper(robot, side, timeout=2),
                                    transitions={   'succeeded'         :'ASK_FOR_CLOG2',
                                                    'failed'            :'ASK_FOR_CLOG2' })
            
            smach.StateMachine.add( "ASK_FOR_CLOG2",
                                    states.Say(robot, ["Please put a clog in my gripper, they're too small for me to pick up"], 
                                               block=False),
                                    transitions={'spoken': "CLOSE_GRIPPER"})
            
            smach.StateMachine.add( "CLOSE_GRIPPER",
                                    states.SetGripper(robot, side, gripperstate=ArmState.CLOSE),
                                    transitions={   'succeeded'         :'RESET_ARM1',
                                                    'failed'            :'RESET_ARM1' })
            
            smach.StateMachine.add( "RESET_ARM1",
                                    states.ArmToJointPos(robot, side, [-0.1, -0.4, 0.00, 1.93, 0.000, 0.00, 0.00], timeout=4), #TODO: Set correct pose
                                    transitions={   'done'              :'RESET_ARM2',
                                                    'failed'            :'RESET_ARM2' })
            
            smach.StateMachine.add( "RESET_ARM2",
                                    states.ArmToJointPos(robot, side, [0.00, -0.39, 0.00, 1.93, 0.00, 0.00, 0.00], timeout=4), #TODO: Set correct pose
                                    transitions={   'done'              :'RESET_ARM',
                                                    'failed'            :'RESET_ARM' })
            
            smach.StateMachine.add( "RESET_ARM",
                                    states.ResetArm(robot, side),
                                    transitions={   'done'              :'Done'})

class GiveClog(smach.StateMachine):
    def __init__(self, robot, side):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot
        self.side = side

        with self:
            smach.StateMachine.add("RESET_SPINDLE_HEAD_UP",
                                states.ResetSpindle_HeadUp(robot),
                                transitions={'done':'SAY_GIVE_CLOGS'})
            
            smach.StateMachine.add( "SAY_GIVE_CLOGS",
                                    states.Say(robot, [ "I'll give you a present!"], 
                                               block=False),
                                    transitions={'spoken': "POOR_1"})

            smach.StateMachine.add( "POOR_1",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.1, 0, 2, 0, 0, 0], timeout=4),
                                    transitions={   'done'                  :'POOR_2',
                                                    'failed'                :'Failed' })
            
            smach.StateMachine.add( "POOR_2",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.4, 0.0, 1.93, 0.0, 0.0, 0.0], timeout=4), 
                                    transitions={   'done'                  :'POOR_3',
                                                    'failed'                :'Failed' })

            smach.StateMachine.add( "POOR_3", 
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.4, 0.0, 1.93, 0.0, 0.0, 0.0], timeout=4),
                                    transitions={   'done'                  : 'POOR_4',
                                                    'failed'                : 'Failed'})

            smach.StateMachine.add( "POOR_4", 
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, 1.3, 0, 0.27, 0, 0, 0], timeout=4),
                                    transitions={   'done'                  : 'SAY_HOLDUP_HAND_FOR_CLOG',
                                                    'failed'                : 'Failed'})
            
            
            smach.StateMachine.add( "SAY_HOLDUP_HAND_FOR_CLOG",
                                    states.Say(robot, [ "Hold up your hand please!"], 
                                               block=True),
                                    transitions={'spoken': "POOR_5"})


            smach.StateMachine.add( "POOR_5",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, 1.3, 0, 0.27, 2.3, 0, 0], timeout=5.0),
                                    transitions={   'done'                  : 'POOR_4_INVERSE',
                                                    'failed'                : 'Failed'})

            smach.StateMachine.add( "POOR_4_INVERSE",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, 1.3, 0, 0.27, 0, 0, 0], timeout=5.0),
                                    transitions={   'done'                  : 'POOR_3_INVERSE',
                                                    'failed'                : 'POOR_3_INVERSE'})

            smach.StateMachine.add( "POOR_3_INVERSE",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.4, 0.0, 1.93, 0.0, 0.0, 0.0], timeout=5.0),
                                    transitions={   'done'                  : 'POOR_2_INVERSE',
                                                    'failed'                : 'POOR_2_INVERSE'})

            smach.StateMachine.add( "POOR_2_INVERSE",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.4, 0.0, 1.93, 0.0, 0.0, 0.0], timeout=5.0),
                                    transitions={   'done'                  : 'POOR_1_INVERSE',
                                                    'failed'                : 'POOR_1_INVERSE'})

            smach.StateMachine.add( "POOR_1_INVERSE",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.1, 0, 2, 0, 0, 0], timeout=5.0),
                                    transitions={   'done'                  : 'Done',
                                                    'failed'                : 'Done'}) #If this worked, we still poored

class GiveCan(smach.StateMachine):
    def __init__(self, robot, side):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot
        self.side = side

        with self:
            smach.StateMachine.add("RESET_SPINDLE_HEAD_UP",
                                states.ResetSpindle_HeadUp(robot),
                                transitions={'done':'SAY_GIVE_CAN'})

            smach.StateMachine.add( "SAY_GIVE_CAN",
                                    states.Say(robot, [ "Here is your drink"], 
                                               block=False),
                                    transitions={'spoken': "GIVE_1"})

            smach.StateMachine.add( "GIVE_1",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.1, 0, 2, 0, 0, 0], timeout=4),
                                    transitions={   'done'                  :'GIVE_2',
                                                    'failed'                :'Failed' })
            
            smach.StateMachine.add( "GIVE_2",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.4, 0.0, 1.93, 0.0, 0.0, 0.0], timeout=4), 
                                    transitions={   'done'                  :'GIVE_3',
                                                    'failed'                :'Failed' })

            smach.StateMachine.add( "GIVE_3", 
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.4, 0.0, 1.93, 0.0, 0.0, 0.0], timeout=4),
                                    transitions={   'done'                  : 'GIVE_3_A',
                                                    'failed'                : 'Failed'})

            smach.StateMachine.add( "GIVE_3_A", 
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, 1.0, 0.0, 1.93, 0.0, 0.0, 0.0], timeout=4),
                                    transitions={   'done'                  : 'GIVE_4',
                                                    'failed'                : 'Failed'})

            smach.StateMachine.add( "GIVE_4", 
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, 1.3, 0, 0.27, 0, 0, 0], timeout=4),
                                    transitions={   'done'                  : 'SAY_HOLDUP_HAND_FOR_CLOG',
                                                    'failed'                : 'Failed'})
            
            
            smach.StateMachine.add( "SAY_HOLDUP_HAND_FOR_CLOG",
                                    states.Say(robot, [ "Please take the drink from my hand, I'll open my gripper"], 
                                               block=True),
                                    transitions={'spoken': "OPEN_GRIPPER"})

            smach.StateMachine.add( 'OPEN_GRIPPER', 
                                    states.SetGripper(robot, self.side, gripperstate=0), #open
                                    transitions={'succeeded'    :   'GIVE_4_INVERSE',
                                             'failed'       :   'GIVE_4_INVERSE'})

            smach.StateMachine.add( "GIVE_4_INVERSE",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, 1.3, 0, 0.27, 0, 0, 0], timeout=5.0),
                                    transitions={   'done'                  : 'GIVE_3_INVERSE',
                                                    'failed'                : 'GIVE_3_INVERSE'})
#TODO: Insert step in between
            smach.StateMachine.add( "GIVE_3_INVERSE",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.4, 0.0, 1.93, 0.0, 0.0, 0.0], timeout=5.0),
                                    transitions={   'done'                  : 'GIVE_2_INVERSE',
                                                    'failed'                : 'GIVE_2_INVERSE'})

            smach.StateMachine.add( "GIVE_2_INVERSE",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.4, 0.0, 1.93, 0.0, 0.0, 0.0], timeout=5.0),
                                    transitions={   'done'                  : 'GIVE_1_INVERSE',
                                                    'failed'                : 'GIVE_1_INVERSE'})

            smach.StateMachine.add( "GIVE_1_INVERSE",
                                    states.ArmToJointPos(self.robot, self.side, [-0.1, -0.1, 0, 2, 0, 0, 0], timeout=5.0),
                                    transitions={   'done'                  : 'Done',
                                                    'failed'                : 'Done'}) #If this worked, we still poored

class RoboZoo(smach.StateMachine):
    """The goal of the challenge is to attract people and be attractive to an audience.
    So, the robot will hand out cans of coke and fanta on one end of a table and clean up empty cans at another end.
    The robot will get the full cans from another table, at the back of the demonstration-stand.

    Optionnally, we can also have to robot ask at the stand what kind of drink to get for a waiting user.

    So, the challenge consists of three parts:
    0:  (Optionally) Asking what drink a 'customer' wants.
        0.0:    NavigateGeneric(query_ordering_table)
        0.1:    Ask the customer what drink he wants. 
                We should have two very different sounding drinks, like coke and fanta etc.
                Speech is tricky, especially in the expected environment.
        0.2:    Assert the ordered drink to the world model
    1:  Getting the (ordered) drink from a table. 
        This involves two states: 
        1.1:    NavigateGeneric(query_storage_table)
        1.2:    GrabMachine(query_ordered_drink)
        1.3:    ReceiveClog (entails grabbing a cup that cointains some clogs)
    2:  Brink the drink to the table and wait for a human to appear. When a face is detected, say that the user can take the drink from his hand. 
        2.1:    NavigateGeneric(query_ordering_table)
        2.2:    Await_queried_perception(face in front)
        2.3:    HandoverToHuman (for the can and and say something nice)
        2.4:    HandOverClogs (by pooring from the can we grasped)
    3:  Then we look if there are any cans delivered to the other end of the table, and clean them up if so.
        3.1:    NavigateGeneric(query_pickup_table)
        3.2:    GrabMachine(query_any_can)
        3.3:    NavigateGeneric(query_trashbin)
        3.4:    Dropoff(query_trashbin).

    So, we have these queries:
    - query_storage_table:  The table where drinks are originally stored.
    - query_ordered_drink:  Which drink the customer wants, so what we have to look for. 
                            We assert some variable to this as well when we get the order
    - query_ordering_table: The table where the customers are at; where we ask for an order and deliver it to.
    - query_pickup_table:   The table where customers can leave their empty cans for the robot to pick up.
    - query_any_can:        At the pickup_table, look for any sort of can, instead of a specific can.
    - query_trashbin:       Location of the trashbin.
    """

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])

        self.robot = robot

        self.can_hand = robot.leftArm
        self.clog_hand = robot.rightArm

        self.init_knowledge()

        query_storage_table =   Compound("point_of_interest", "storage_table", Compound("point_3d", "X", "Y", "Z"))

        #To store an order, assertz Compound("goal", Compound("serve", "Drink")) to the world model
        query_ordered_drink     = Conjunction(
                                    Compound("goal", Compound("serve", "Drink")),
                                    Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                    Compound( "property_expected", "ObjectID", "position", 
                                        Compound("in_front_of", "amigo")),
				    Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))
        
        query_ordering_table    = Compound("point_of_interest", "ordering_table", Compound("point_3d", "X", "Y", "Z"))
        
        poor_point_query        = Compound("point_of_interest", "poor_point", Compound("point_3d", "X", "Y", "Z"))
        
        query_pickup_table      = Compound("point_of_interest", "pickup_table", Compound("point_3d", "X", "Y", "Z"))
        
        query_any_can           = Conjunction(
                                    Compound( "property_expected", "ObjectID", "class_label", "can"),
                                    Compound( "property_expected", "ObjectID", "position", 
                                        Compound("in_front_of", "amigo")))
        
        query_trashbin          = Compound("point_of_interest", "trashbin1", Compound("point_3d", "X", "Y", "Z"))

        query_detect_person     = Conjunction(Compound("property_expected", "ObjectID", "class_label", "person"),
                                          Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                          Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))


        with self:
            smach.StateMachine.add( "LOCK_GMAPPING",
                                    ClearCostmapAround(robot),
                                    transitions={'Done':"FIND_FLIGHTCASE"})
            
            smach.StateMachine.add( "FIND_FLIGHTCASE",
                                    states.ToggleDemoLaser(robot),
                                    transitions={'done':"ASSERT_RELATIVE_TO_FLIGHTCASE",
                                                 'failed':"ASSERT_RELATIVE_TO_FLIGHTCASE"})

            @smach.cb_interface(outcomes=['asserted'])
            def rel_to_flightcase(*args, **kwargs):
                self.robot.reasoner.query(Compound("retractall", Compound("point_of_interest", "Poi", "P")))

                #Points are defined relative to a flightcase, using package demo_laser. The /kist-frame is in the middle of the front of the case
                #waypoint_storage_table  = msg.PoseStamped( -0.7, 0,     0,      frame_id="/kist") #In front of the flightcase
                poi_storage_table       = msg.PointStamped( 0.1, 0,     0.74,   frame_id="/kist") #10cm inside the bounds of the flight case
                poi_trashbin            = msg.PointStamped(-0.4, -1.1,   0.7,    frame_id="/kist") #Trashbin is outside the bounds of the case, 1.1m right of the waypoint_storage_table
                poi_pooring             = msg.PointStamped(-1.9, 0,     0.9,    frame_id="/kist")
                poi_ordering_table      = msg.PointStamped(-1.9, 0,     0.74,   frame_id="/kist")
                poi_pickup_table        = msg.PointStamped(-1.9, -0.5,   0.74,   frame_id="/kist")

                #import ipdb; ipdb.set_trace()
                publish_marker(poi_storage_table,   color=(0,   1.0,0))     #green
                publish_marker(poi_trashbin,        color=(0,   0,  0))     #black
                publish_marker(poi_pooring,         color=(0,   0,  1.0))   #blue
                publish_marker(poi_ordering_table,  color=(1.0, 0,  1.0))   #purple
                publish_marker(poi_pickup_table,    color=(1.0, 0,  0))     #red

                storage = transformations.tf_transform(poi_storage_table, poi_storage_table.header.frame_id, "/map")
                robot.reasoner.assertz(Compound("point_of_interest", "storage_table", Compound("point_3d", storage.x, storage.y, storage.z)))

                trash = transformations.tf_transform(poi_trashbin, poi_trashbin.header.frame_id, "/map")
                robot.reasoner.assertz(Compound("point_of_interest", "trashbin1", Compound("point_3d", trash.x, trash.y, trash.z)))

                poor = transformations.tf_transform(poi_pooring, poi_pooring.header.frame_id, "/map")
                robot.reasoner.assertz(Compound("point_of_interest", "poor_point", Compound("point_3d", poor.x, poor.y, poor.z)))

                order = transformations.tf_transform(poi_ordering_table, poi_ordering_table.header.frame_id, "/map")
                robot.reasoner.assertz(Compound("point_of_interest", "ordering_table", Compound("point_3d", order.x, order.y, order.z)))

                pickup = transformations.tf_transform(poi_pickup_table, poi_pickup_table.header.frame_id, "/map")
                robot.reasoner.assertz(Compound("point_of_interest", "pickup_table", Compound("point_3d", pickup.x, pickup.y, pickup.z)))
                return "asserted"
            smach.StateMachine.add( "ASSERT_RELATIVE_TO_FLIGHTCASE",
                                    smach.CBState(rel_to_flightcase),
                                    transitions={"asserted"             :"SET_CURRENT_DRINK"})

            @smach.cb_interface(outcomes=['asserted'])
            def set_serve_drink(*args, **kwargs):
                drink = kwargs.get("drink", "coke") #Get from key and default to coke if it doesnt exist
                robot.reasoner.query(Compound("retractall", Compound("goal", Compound("serve", "Drink"))))
                robot.reasoner.assertz(Compound("goal", Compound("serve", drink)))
                return "asserted"
            smach.StateMachine.add( "SET_CURRENT_DRINK",
                                    smach.CBState(set_serve_drink, cb_kwargs={'drink':'coke'}),
                                    transitions={"asserted"             :"SAY_GETTING_DRINK"})

            smach.StateMachine.add( "SAY_GETTING_DRINK",
                                    states.Say(robot, [ "You look like you could use a drink, I'll get you something",
                                                        "Let me fetch you a drink",
                                                        "Wait a sec, I'll get you a drink and a present",
                                                        "I will bring you a drink and a special present if you wait a sec.",
                                                        "I'll be back. With a drink and a little gift for you"], 
                                               block=False),
                                    transitions={'spoken': "RESET_ARMS"})
            
            smach.StateMachine.add( "RESET_ARMS",
                                    states.ResetArms(robot),
                                    transitions={"done"                 :"SPINDLE_HIGH_1"})
            
            smach.StateMachine.add( "SPINDLE_HIGH_1",
                                    states.SetSpindle(robot, height=0.4),
                                    transitions={"done"                 :"GOTO_STORAGE"})

            smach.StateMachine.add( "GOTO_STORAGE",
                                    states.NavigateGeneric(robot, lookat_query=query_storage_table),
                                    transitions={   "arrived"           :"LOOK_FOR_DRINK", 
                                                    "unreachable"       :"LOOK_FOR_DRINK", 
                                                    "preempted"         :"Aborted", 
                                                    "goal_not_defined"  :"Failed"})

            smach.StateMachine.add( "LOOK_FOR_DRINK",
                                    states.LookForObjectsAtROI(robot, query_storage_table, query_ordered_drink), 
                                    transitions={   'looking'           :'LOOK_FOR_DRINK',
                                                    'object_found'      :'GRAB_DRINK',
                                                    'no_object_found'   :'HELP_WITH_GETTING_DRINK', #TODO: Not the best option maybe
                                                    'abort'             :'Aborted'})

            smach.StateMachine.add( "HELP_WITH_GETTING_DRINK",
                                    states.Human_handover(self.can_hand, robot),
                                     transitions={  'succeeded'         :'GET_CLOG_1',
                                                    'failed'            :'GET_CLOG_1' }) #We're lost if even this fails

            smach.StateMachine.add( "GRAB_DRINK",
                                    states.GrabMachine("left", robot, query_ordered_drink),
                                    transitions={   'succeeded'         :'MOVE_ARM_AWAY_FROM_COLLISION',
                                                    'failed'            :'HELP_WITH_GETTING_DRINK' })

            smach.StateMachine.add( "MOVE_ARM_AWAY_FROM_COLLISION", 
                                    states.ArmToUserPose(self.can_hand, 0.0, 0.0, -0.1, 0.0, 0.0 , 0.0, time_out=20, pre_grasp=False, frame_id="/base_link", delta=True),
                                    transitions={   'succeeded'             : 'GET_CLOG_1',
                                                    'failed'                : 'GET_CLOG_1'})

            smach.StateMachine.add( "GET_CLOG_1",
                                    GrabClog(robot, "right"),
                                    transitions={   'Done'              :'GOTO_ORDERING',
                                                    'Failed'            :'GET_CLOG_BACKUP' })

            smach.StateMachine.add( "GET_CLOG_BACKUP",
                                    ReceiveClog(robot, self.clog_hand),
                                    transitions={   'Done'              :'GOTO_ORDERING',
                                                    'Failed'            :'GOTO_ORDERING' })

            smach.StateMachine.add( "GOTO_ORDERING",
                                    states.NavigateGeneric(robot, lookat_query=query_ordering_table),
                                    transitions={   "arrived"           :"AWAIT_PERSON", 
                                                    "unreachable"       :"AWAIT_PERSON", #TODO: we should ask for help
                                                    "preempted"         :"Aborted", 
                                                    "goal_not_defined"  :"Failed"})

            smach.StateMachine.add( 'AWAIT_PERSON',
                                    states.Wait_queried_perception(robot, ['ppl_detection'], query_detect_person),
                                    transitions={   'query_true'        :'SAY_HOLDUP_HAND_FOR_DRINK', 
                                                    'timed_out'         :'PLACE_ORDER',
                                                    'preempted'         :'Aborted'})

            smach.StateMachine.add( "SAY_HOLDUP_HAND_FOR_DRINK",
                                    states.Say(robot, [ "I'll give you a drink, hold your hand up", "Hold your hand up, I'll give you a drink"], 
                                               block=False),
                                    transitions={'spoken': "HAND_DRINK_TO_HUMAN"})

            smach.StateMachine.add( "HAND_DRINK_TO_HUMAN",
                                    #states.HandoverToHuman(self.can_hand, robot),
                                    GiveCan(robot, self.can_hand),
                                    transitions={   "Done"         :"SAY_TAKE_DRINK",
                                                    "Aborted"      :"Aborted", 
                                                    "Failed"       :"SAY_TAKE_DRINK"}) #We're lost if even this fails
                                     # transitions={  'succeeded'        :'SAY_TAKE_DRINK',
                                     #                'failed'           :'SAY_TAKE_DRINK' }) #We're lost if even this fails

            smach.StateMachine.add( "PLACE_ORDER",
                                    states.PlaceObject("left", robot, placement_query=query_ordering_table),
                                    transitions={   "succeeded"         :"SAY_TAKE_DRINK",
                                                    "failed"            :"HAND_DRINK_TO_HUMAN", 
                                                    "target_lost"       :"HAND_DRINK_TO_HUMAN"})

            smach.StateMachine.add( "SAY_TAKE_DRINK",
                                    states.Say(robot, [ "Enjoy your drink! I have another present as well for you", 
                                                        "There you go, enjoy your drink. If you wait, I have something else for you as well",
                                                        "Cheers! I have a special gift for you as well, please wait a sec."], 
                                               block=False),
                                    transitions={'spoken': "AWAIT_PERSON_FOR_CLOG"})

            smach.StateMachine.add( 'AWAIT_PERSON_FOR_CLOG',
                                    states.Wait_queried_perception(robot, ['ppl_detection'], query_detect_person),
                                    transitions={   'query_true'        :'GIVE_CLOG', 
                                                    'timed_out'         :'PLACE_CLOG',
                                                    'preempted'         :'Aborted'})

            smach.StateMachine.add( "GIVE_CLOG",
                                    GiveClog(robot, self.clog_hand), 
                                    transitions={   "Done"         :"SAY_TAKE_CLOGS",
                                                    "Aborted"      :"Aborted", 
                                                    "Failed"       :"PLACE_CLOG"})

            smach.StateMachine.add( "PLACE_CLOG",
                                    states.PlaceObject("right", robot, placement_query=query_ordering_table),
                                    transitions={   "succeeded"         :"SAY_TAKE_CLOGS",
                                                    "failed"            :"HELP_WITH_PLACING_CLOG", 
                                                    "target_lost"       :"HELP_WITH_PLACING_CLOG"})

            smach.StateMachine.add( "SAY_TAKE_CLOGS",
                                    states.Say(robot, [ "Enjoy your clogs! If you give them enough water, they'll grow to your size.",
                                                        "Enjoy your wooden shoes and your drink!",
                                                        "Enjoy your drink and your real dutch clogs!",
                                                        "There you are, real Dutch wooden shoes!",
                                                        "Enjoy your special Limited Edition Tech United clogs",
                                                        "Here's my gift to you, real dutch clogs.",
                                                        "Please take these clogs and feel like a real Dutchman."], 
                                               block=False),
                                    transitions={'spoken': "SET_CURRENT_DRINK"}) #Don't pick up empty cans, takes too much time

            smach.StateMachine.add( "HELP_WITH_PLACING_CLOG",
                                    states.HandoverToHuman(self.clog_hand, robot),
                                     transitions={  'succeeded'        :'RESET_ARMS2',
                                                    'failed'           :'RESET_ARMS2' }) #We're lost if even this fails

            smach.StateMachine.add( "RESET_ARMS2",
                                    states.ResetArms(robot),
                                    transitions={"done"                 :"SPINDLE_HIGH_2"})
            
            smach.StateMachine.add( "SPINDLE_HIGH_2",
                                    states.SetSpindle(robot, height=0.4),
                                    transitions={"done"                 :"SAY_TAKE_CLOGS"})

            smach.StateMachine.add( "GOTO_PICKUP",
                                    states.NavigateGeneric(robot, lookat_query=query_pickup_table),
                                    transitions={   "arrived"           :"SAY_BRING_EMPTY_CANS", 
                                                    "unreachable"       :"SAY_BRING_EMPTY_CANS", 
                                                    "preempted"         :"Aborted", 
                                                    "goal_not_defined"  :"Failed"})
            
            smach.StateMachine.add( "SAY_BRING_EMPTY_CANS",
                                    states.Say(robot, [ "Please put you empty cans on the table, I'll clean them up",
                                                        "Your empty cans please. I'll get them from the table",
                                                        "If you put your empty cans on the table, I'll clean up"], 
                                               block=False),
                                    transitions={'spoken': "LOOK_FOR_EMPTY_CAN"})

            smach.StateMachine.add( "LOOK_FOR_EMPTY_CAN",
                                    states.LookForObjectsAtROI(robot, query_pickup_table, query_any_can),
                                    transitions={   'looking'           :'LOOK_FOR_EMPTY_CAN',
                                                    'object_found'      :'GRAB_EMPTY_CAN',
                                                    'no_object_found'   :'SET_CURRENT_DRINK', #Nothing here, so skip this step
                                                    'abort'             :'Aborted'})

            smach.StateMachine.add( "GRAB_EMPTY_CAN",
                                    states.GrabMachine("left", robot, query_any_can),
                                    transitions={   'succeeded'         :'GOTO_TRASHBIN',
                                                    'failed'            :'HELP_WITH_GETTING_EMPTY_CAN' })

            smach.StateMachine.add( "HELP_WITH_GETTING_EMPTY_CAN",
                                    states.Human_handover(self.can_hand, robot),
                                     transitions={  'succeeded'        :'GOTO_TRASHBIN',
                                                    'failed'           :'GOTO_TRASHBIN' }) #We're lost if even this fails

            smach.StateMachine.add( "GOTO_TRASHBIN",
                                    states.NavigateGeneric(robot, lookat_query=query_trashbin),
                                    transitions={   "arrived"           :"DROPOFF_EMPTY_CAN", 
                                                    "unreachable"       :"DROPOFF_EMPTY_CAN", 
                                                    "preempted"         :"Aborted", 
                                                    "goal_not_defined"  :"Failed"})

            smach.StateMachine.add("DROPOFF_EMPTY_CAN",
                                    states.DropObject("left", robot, query_trashbin),
                                    transitions={   'succeeded'         :'SET_CURRENT_DRINK',
                                                    'failed'            :'HELP_WITH_DUMPING_CAN',
                                                    'target_lost'       :'HELP_WITH_DUMPING_CAN'})
            
            smach.StateMachine.add( "HELP_WITH_DUMPING_CAN",
                                    states.HandoverToHuman(self.can_hand, robot),
                                     transitions={  'succeeded'        :'SET_CURRENT_DRINK',
                                                    'failed'           :'SET_CURRENT_DRINK' }) #We're lost if even this fails


    def init_knowledge(self):
        self.robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("goal", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
        self.robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
        self.robot.reasoner.query(Compound("retractall", Compound("disposed", "X")))
        
        # self.robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
        # self.robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
    
        self.robot.reasoner.assertz(Compound("challenge", "robo_zoo"))


if __name__ == "__main__":
    rospy.init_node("challenge_robo_zoo")

    startup(RoboZoo)
