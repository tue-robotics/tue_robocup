#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/Restaurant.tex
In short:

1) The robot follows a guide inside a unknown space while learning the environment. The guide will tell the robot where the tables are.
2) After the robot has been guided back to the kitchen, the robot asks which table he should go to ask an order.
   Robot drives to that location and asks for an order which can be either a drink (coke) or a combo (burger with fries)
   Robot then drives to the kitchen and graps the drink or asks for a tray with the combo.
3) At any time during step 2) when somebody at an other table waves at the robot it should go there and ask for an order
4) Once two orders are specified, the robot should deliver the combo and the drink.
"""

import rospy
import smach
import sys
import math
import time

import robot_smach_states as states
from robot_smach_states.util.startup import startup

from robot_smach_states.util.designators import EdEntityDesignator
from robot_skills.util import transformations, msg_constructors

from robocup_knowledge import load_knowledge
common_knowledge = load_knowledge("common")
knowledge = load_knowledge("challenge_restaurant")

ORDERS = {}

class HeadStraight(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata):
        self._robot.head.look_at_standing_person()
        return "done"

class HeadCancel(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata):
        self._robot.head.cancel_goal()
        return "done"

class StoreKitchen(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata):
        self._robot.ed.update_entity(id="kitchen", posestamped=self._robot.base.get_location(), type="waypoint")
        return "done"

class StoreWaypoint(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "continue"])
        self._robot = robot

    def execute(self, userdata):
        # Stop the base
        self._robot.base.local_planner.cancelCurrentPlan()
        
        base_pose = self._robot.base.get_location()

        choices = knowledge.guiding_choices
        
        self._robot.head.look_at_standing_person()
        result = self._robot.ears.recognize(knowledge.guiding_spec, choices, time_out = rospy.Duration(10)) # Wait 100 secs
        time.sleep(1.0)
        self._robot.head.cancel_goal()

        if result:
            if "continue" in result.choices:
                return "continue"
            if "side" in result.choices and "location" in result.choices:
                side = result.choices["side"]
                location = result.choices["location"]

                self._robot.head.look_at_standing_person()
#                self._robot.speech.speak("%s %s?"%(location, side))
#                result = self._robot.ears.recognize("(yes|no)",{})
#                self._robot.head.cancel_goal()
#                if not result or result.result == "no":
#                    self._robot.speech.speak("Sorry", block=False)
#                    return "continue"

                self._robot.speech.speak("%s %s, it is!"%(location, side))
                self._robot.head.cancel_goal()
                
                if side == "left":
                    base_pose.pose.orientation = transformations.euler_z_to_quaternion(transformations.euler_z_from_quaternion(base_pose.pose.orientation) + math.pi / 2)
                elif side == "right":
                    base_pose.pose.orientation = transformations.euler_z_to_quaternion(transformations.euler_z_from_quaternion(base_pose.pose.orientation) - math.pi / 2)

                # Store waypoint in world model
                print "Asserting waypoint %s to world model"%location   
                self._robot.ed.update_entity(id=location, posestamped=base_pose, type="waypoint")

                return "done"

        return "continue"

class CheckInKitchen(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["in_kitchen", "not_in_kitchen"])
        self._robot = robot

    def execute(self, userdata):
        # Get the robot pose and compare if we are close enough to the kitchen waypoint
        kitchen = self._robot.ed.get_entity(id="kitchen")
        if kitchen:
            current = self._robot.base.get_location() 
            if math.hypot(current.pose.position.x - kitchen.pose.position.x, current.pose.position.y - kitchen.pose.position.y) < knowledge.kitchen_radius:
                return "in_kitchen"
        else:
            print "NO KITCHEN IN ED???"
        return "not_in_kitchen"

class AskOrder(smach.State):
    def __init__(self, robot, location):
        smach.State.__init__(self, outcomes=["next_order",'orders_done'])

        self._robot = robot
        self._location = location

    def execute(self, userdata):
        self._robot.speech.speak("Which combo or beverage do you want?")

        order = None
        while not order:
            self._robot.head.look_at_standing_person()
            result = self._robot.ears.recognize(knowledge.order_spec, knowledge.order_choices)
            self._robot.head.cancel_goal()
            if "beverage" in result.choices:
                order = result.choices["beverage"]
                ORDERS["beverage"] = { "location" : self._location, "name" : order }
            elif "food1" and "food2" in result.choices:
                order = "%s and %s" % (result.choices["food1"], result.choices["food2"])
                ORDERS["combo"] = { "location" : self._location, "name" : order }
                
        self._robot.speech.speak("Ok, I will get you %s"%order, block=False)

        if "combo" in ORDERS and "beverage" in ORDERS:
            return "orders_done"
        else:
            return "next_order"

class SpeakOrders(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["spoken"])

        self._robot = robot

    def execute(self, userdata):
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("Here are the orders:")
        if "beverage" in ORDERS:
            self._robot.speech.speak("Table %s would like to have the beverage %s"%(ORDERS["beverage"]["location"], ORDERS["beverage"]["name"]))
        if "combo" in ORDERS:
            self._robot.speech.speak("Table %s would like to have the combo %s"%(ORDERS["combo"]["location"], ORDERS["combo"]["name"]))

        self._robot.head.cancel_goal()

        return "spoken"

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['done', 'aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE', states.Initialize(robot), transitions={   'initialized':'STORE_KITCHEN', 'abort':'aborted'})
        smach.StateMachine.add('STORE_KITCHEN', StoreKitchen(robot), transitions={   'done':'HEAD_STRAIGHT'})
        smach.StateMachine.add('HEAD_STRAIGHT', HeadStraight(robot), transitions={   'done':'SAY_INTRO'})

        smach.StateMachine.add('SAY_INTRO', states.Say(robot, "Hi, Show me your restaurant please. Say 'Please Follow Me'"), transitions={ 'spoken' :'HEAR_PLEASE_FOLLOW_ME'})
        smach.StateMachine.add('HEAR_PLEASE_FOLLOW_ME', states.HearOptions(robot, ["please follow me"]), transitions={ 'no_result' :'HEAR_PLEASE_FOLLOW_ME', 'please follow me' : 'FOLLOW_TO_FIRST'})
        
        smach.StateMachine.add('FOLLOW_TO_FIRST', states.FollowOperator(robot), transitions={ 'stopped':'STORE_FIRST', 'lost_operator':'FOLLOW_TO_FIRST'})
        smach.StateMachine.add('STORE_FIRST', StoreWaypoint(robot), transitions={ 'done':'FOLLOW_TO_SECOND', 'continue':'FOLLOW_TO_FIRST'})

        smach.StateMachine.add('FOLLOW_TO_SECOND', states.FollowOperator(robot), transitions={ 'stopped':'STORE_SECOND', 'lost_operator':'FOLLOW_TO_SECOND'})
        smach.StateMachine.add('STORE_SECOND', StoreWaypoint(robot), transitions={ 'done':'FOLLOW_TO_THIRD', 'continue':'FOLLOW_TO_SECOND'})

        smach.StateMachine.add('FOLLOW_TO_THIRD', states.FollowOperator(robot), transitions={ 'stopped':'STORE_THIRD', 'lost_operator':'FOLLOW_TO_THIRD'})
        smach.StateMachine.add('STORE_THIRD', StoreWaypoint(robot), transitions={ 'done':'SAY_FOLLOW_TO_KITCHEN', 'continue':'FOLLOW_TO_THIRD'})

        smach.StateMachine.add('SAY_FOLLOW_TO_KITCHEN', states.Say(robot, "I stored all places, please bring me back to the kitchen!"), transitions={ 'spoken' :'FOLLOW_TO_KITCHEN'})

        smach.StateMachine.add('FOLLOW_TO_KITCHEN', states.FollowOperator(robot), transitions={ 'stopped':'CHECK_IN_KITCHEN', 'lost_operator':'FOLLOW_TO_KITCHEN'})
        smach.StateMachine.add('CHECK_IN_KITCHEN', CheckInKitchen(robot), transitions={ 'not_in_kitchen':'FOLLOW_TO_KITCHEN', 'in_kitchen':'SAY_IN_KITCHEN'})

        smach.StateMachine.add('SAY_IN_KITCHEN', states.Say(robot, "We are in the kitchen again!"), transitions={ 'spoken' :'SAY_WHICH_ORDER'})

        # Where to take the order from?
        smach.StateMachine.add('SAY_WHICH_ORDER', states.Say(robot, "From which table should I take the first order?"), transitions={ 'spoken' :'HEAR_WHICH_ORDER'})
        smach.StateMachine.add('HEAR_WHICH_ORDER', states.HearOptions(robot, ["one", "two", "three"]),
            transitions={ 'no_result' :'SAY_WHICH_ORDER', 'one' : 'SAY_TAKE_ORDER_FROM_TABLE_1', 'two': 'SAY_TAKE_ORDER_FROM_TABLE_2', 'three' : "SAY_TAKE_ORDER_FROM_TABLE_3"})

        for i in range(1,4):
            # ############## first table #####################
            smach.StateMachine.add('SAY_TAKE_ORDER_FROM_TABLE_%d'%i, states.Say(robot, "Okay, I will take an order from table %d"%i, block=False), transitions={ 'spoken' :'%d_NAVIGATE_TO_WAYPOINT_TABLE_%d'%(i,i)})

            smach.StateMachine.add('%d_NAVIGATE_TO_WAYPOINT_TABLE_1'%i,
                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="one"), radius = 0.2),
                    transitions={'arrived': '%d_SAY_IF_ORDER_TABLE_1'%i, 'unreachable':'%d_NAVIGATE_TO_WAYPOINT_TABLE_2'%i, 'goal_not_defined':'%d_NAVIGATE_TO_WAYPOINT_TABLE_2'%i})
            smach.StateMachine.add('%d_SAY_IF_ORDER_TABLE_1'%i, states.Say(robot, "Would you like to order something?"), transitions={ 'spoken' :'%d_HEAR_IF_ORDER_TABLE_1'%i})
            smach.StateMachine.add('%d_HEAR_IF_ORDER_TABLE_1'%i, states.HearOptions(robot, ['yes','no'], timeout = rospy.Duration(10)), transitions={ 'no_result' :'%d_NAVIGATE_TO_WAYPOINT_TABLE_2'%i, 'yes':'%d_ASK_ORDER_TABLE_1'%i,'no':'%d_NAVIGATE_TO_WAYPOINT_TABLE_2'%i})
            smach.StateMachine.add('%d_ASK_ORDER_TABLE_1'%i, AskOrder(robot, "one"), transitions={'next_order':'%d_NAVIGATE_TO_WAYPOINT_TABLE_2'%i, 'orders_done' : 'SAY_ORDERS_DONE'})

            smach.StateMachine.add('%d_NAVIGATE_TO_WAYPOINT_TABLE_2'%i,
                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="two"), radius = 0.2),
                    transitions={'arrived': '%d_ASK_ORDER_TABLE_2'%i, 'unreachable':'%d_NAVIGATE_TO_WAYPOINT_TABLE_3'%i, 'goal_not_defined':'%d_NAVIGATE_TO_WAYPOINT_TABLE_3'%i})
            smach.StateMachine.add('%d_SAY_IF_ORDER_TABLE_2'%i, states.Say(robot, "Would you like to order something?"), transitions={ 'spoken' :'%d_HEAR_IF_ORDER_TABLE_2'%i})
            smach.StateMachine.add('%d_HEAR_IF_ORDER_TABLE_2'%i, states.HearOptions(robot, ['yes','no'], timeout = rospy.Duration(10)), transitions={ 'no_result' :'%d_NAVIGATE_TO_WAYPOINT_TABLE_3'%i, 'yes':'%d_ASK_ORDER_TABLE_2'%i,'no':'%d_NAVIGATE_TO_WAYPOINT_TABLE_3'%i})
            smach.StateMachine.add('%d_ASK_ORDER_TABLE_2'%i, AskOrder(robot, "two"), transitions={'next_order':'%d_NAVIGATE_TO_WAYPOINT_TABLE_3'%i, 'orders_done' : 'SAY_ORDERS_DONE'})

            smach.StateMachine.add('%d_NAVIGATE_TO_WAYPOINT_TABLE_3'%i,
                    states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="three"), radius = 0.2),
                    transitions={'arrived': '%d_ASK_ORDER_TABLE_3'%i, 'unreachable':'%d_NAVIGATE_TO_WAYPOINT_TABLE_1'%i, 'goal_not_defined':'%d_NAVIGATE_TO_WAYPOINT_TABLE_1'%i})
            smach.StateMachine.add('%d_SAY_IF_ORDER_TABLE_3'%i, states.Say(robot, "Would you like to order something?"), transitions={ 'spoken' :'%d_HEAR_IF_ORDER_TABLE_3'%i})
            smach.StateMachine.add('%d_HEAR_IF_ORDER_TABLE_3'%i, states.HearOptions(robot, ['yes','no'], timeout = rospy.Duration(10)), transitions={ 'no_result' :'%d_NAVIGATE_TO_WAYPOINT_TABLE_1'%i, 'yes':'%d_ASK_ORDER_TABLE_3'%i,'no':'%d_NAVIGATE_TO_WAYPOINT_TABLE_1'%i})
            smach.StateMachine.add('%d_ASK_ORDER_TABLE_3'%i, AskOrder(robot, "three"), transitions={'next_order':'%d_NAVIGATE_TO_WAYPOINT_TABLE_1'%i, 'orders_done' : 'SAY_ORDERS_DONE'})

        smach.StateMachine.add('SAY_ORDERS_DONE', states.Say(robot, "I received enough orders for now, going back to the kitchen!", block=False), transitions={ 'spoken' :'NAVIGATE_BACK_TO_THE_KITCHEN'})

        smach.StateMachine.add('NAVIGATE_BACK_TO_THE_KITCHEN', states.NavigateToWaypoint(robot, EdEntityDesignator(robot, id="kitchen"), radius = 0.2),
            transitions={'arrived': 'SPEAK_ORDERS', 'unreachable':'done', 'goal_not_defined':'done'})
        smach.StateMachine.add('SPEAK_ORDERS', SpeakOrders(robot), transitions={ 'spoken' :'BANANA_KING'})

        smach.StateMachine.add('BANANA_KING', states.Say(robot, "You're the banana king!", block=False), transitions={ 'spoken' :'done'})

    return sm


############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('restaurant_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE RESTAURANT] Please provide robot name as argument."
        exit(1)

    startup(setup_statemachine, robot_name=robot_name)
