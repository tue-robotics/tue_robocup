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
import rosparam
import rospkg
import smach
import sys
import math
import time

from visualization_msgs.msg import Marker

import robot_smach_states as states
from robot_smach_states.util.startup import startup

from robot_smach_states.util.designators import VariableDesignator, EdEntityDesignator, EntityByIdDesignator, analyse_designators
from robot_skills.util import transformations, msg_constructors
from robot_skills.util.kdl_conversions import FrameStamped

from robocup_knowledge import load_knowledge
common_knowledge = load_knowledge("common")
knowledge = load_knowledge("challenge_restaurant")

from visualize import visualize_location


tables = {1: "one", 2: "two", 3: "three"}
# ORDERS is filled with 2 keys: "beverage" and "combo".
#   - Each value is a dictionary itself, with keys "location" and "name".
#       - key "location" gets its value from the values in the tables-dictionary
#       - key "name" gets the value of the order, e.g. "banana and apple"
ORDERS = {}

WAYPOINT_RADIUS = 0.2

class HearWhichTable(smach.State):
    def __init__(self, robot, timeout = rospy.Duration(10), look_at_standing_person=True):
        smach.State.__init__(self, outcomes=["one", "two", "three", "no_result"])
        self._robot = robot
        self._timeout = timeout
        self.look_at_standing_person = look_at_standing_person

    def execute(self, userdata=None):
        if self.look_at_standing_person:
            self._robot.head.look_at_standing_person()

        answer = self._robot.ears.recognize("<option>", {"option":["one", "two", "three"]}, self._timeout)

        if answer:
            if answer.result:
                if "option" in answer.choices:
                    number = answer.choices["option"]
                    self._robot.speech.speak("Table %s, is this correct?"%number)

                    answer = self._robot.ears.recognize("<option>", {"option":["yes", "no"]}, self._timeout)

                    if answer:
                        if answer.result:
                            if "option" in answer.choices:
                                if answer.choices["option"] == "yes":
                                    if self.look_at_standing_person:
                                        self._robot.head.cancel_goal()
                                    return number
                    else:
                        self._robot.speech.speak("Something is wrong with my ears, please take a look!")
        else:
            self._robot.speech.speak("Something is wrong with my ears, please take a look!")

        if self.look_at_standing_person:
            self._robot.head.cancel_goal()

        return "no_result"

class HeadStraight(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata=None):
        self._robot.head.look_at_standing_person()
        return "done"

class HeadCancel(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata=None):
        self._robot.head.close()
        return "done"

class StoreKitchen(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        robot.base.local_planner.cancelCurrentPlan()

    def execute(self, userdata=None):
        self._robot.ed.update_entity(id="kitchen", frame_stamped=self._robot.base.get_location(), type="waypoint")

        return "done"

class StoreBeverageSide(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata=None):
        self._robot.speech.speak("Is the bar on my left or on my right?")

        self._robot.head.look_at_standing_person()
        base_pose = self._robot.base.get_location().frame
        result = None
        while not result:
            result = self._robot.ears.recognize('<side>', {'side':['left','right']}, time_out = rospy.Duration(10)) # Wait 100 secs

        if result.result == "left":
            base_pose.M.DoRotZ(math.pi / 2)
        elif result.result == "right":
            base_pose.M.DoRotZ(-math.pi / 2)
        else:
            print "\n\n WHUT?? No left or right lol? \n\n"
            print result
            print "\n"

        self._robot.ed.update_entity(id="beverages", kdl_frame_stamped=FrameStamped(base_pose, "/map"), type="waypoint")
        return "done"

def load_waypoints(robot, filename="/param/locations.yaml"):
        rp = rospkg.rospack.RosPack()
        restaurant_package = rp.get_path("challenge_restaurant")
        locations = rosparam.load_file(restaurant_package+filename)
        rospy.set_param("/restaurant_locations/", locations)

        for tablename in tables.values():
            location = locations[0][0][0][0]['locations'][tablename] #Don't ask why there's so many subindices to use...
            base_pose = msg_constructors.PoseStamped(location['x'], location['y'], z=0)
            base_pose.pose.orientation = transformations.euler_z_to_quaternion(location['phi'])

            visualize_location(base_pose, tablename)
            robot.ed.update_entity(id=tablename, kdl_frame_stamped=FrameStamped(base_pose, "/map"), type="waypoint")

if "--custom" not in sys.argv:
    from automatic_side_detection import StoreWaypoint
else:
    class StoreWaypoint(smach.State):
        def __init__(self, robot):
            smach.State.__init__(self, outcomes=["done", "continue"])
            self._robot = robot
            self._robot.speech.speak("Using a custom waiter")

        def execute(self, userdata=None):
            # Stop the base
            self._robot.base.local_planner.cancelCurrentPlan()

            base_pose = self._robot.base.get_location().frame

            choices = knowledge.guiding_choices

            self._robot.head.look_at_standing_person()
            self._robot.speech.speak("Location and side?")
            result = self._robot.ears.recognize(knowledge.guiding_spec, choices, time_out = rospy.Duration(10)) # Wait 100 secs
            self._robot.head.cancel_goal()

            if result:
                if "continue" in result.choices:
                    return "continue"
                if "side" in result.choices and "location" in result.choices:
                    side = result.choices["side"]
                    location = result.choices["location"]

                    self._robot.head.look_at_standing_person()
                   # self._robot.speech.speak("%s %s?"%(location, side))
                   # result = self._robot.ears.recognize("(yes|no)",{})
                   # self._robot.head.cancel_goal()
                   # if not result or result.result == "no":
                   #     self._robot.speech.speak("Sorry", block=False)
                   #     return "continue"

                    self._robot.speech.speak("%s %s, it is!"%(location, side))
                    self._robot.head.cancel_goal()

                    if side == "left":
                        base_pose.M.DoRotZ(math.pi / 2)
                    elif side == "right":
                        base_pose.M.DoRotZ(-math.pi / 2)

                    loc_dict = {'x':base_pose.p.x(), 'y':base_pose.p.y(), 'phi':base_pose.M.GetRPY()[2]}
                    rospy.set_param("/restaurant_locations/{name}".format(name=location), loc_dict)
                    visualize_location(base_pose, location)
                    self._robot.ed.update_entity(id=location, kdl_frame_stamped=FrameStamped(base_pose, "/map"), type="waypoint")

                    return "done"

            return "continue"


class CheckKnowledge(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["yes", "no"])
        self._robot = robot

    def execute(self, userdata=None):
        # Get the robot pose and compare if we are close enough to the kitchen waypoint
        entity_ids = set([e.id for e in self._robot.ed.get_entities()])
        if set(["one", "two", "three"]).issubset(entity_ids):
            return "yes"
        return "no"

class CheckInKitchen(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["in_kitchen", "not_in_kitchen"])
        self._robot = robot

    def execute(self, userdata=None):
        # Get the robot pose and compare if we are close enough to the kitchen waypoint
        kitchen = self._robot.ed.get_entity(id="kitchen")
        if kitchen:
            current = self._robot.base.get_location().frame
            if kitchen.distance_to_2d(current) < knowledge.kitchen_radius:
                return "in_kitchen"
        else:
            print "NO KITCHEN IN ED???"
        return "not_in_kitchen"

class LookAtPersonSitting(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])

        self._robot = robot

    def execute(self, userdata=None):
        self._robot.head.look_at_ground_in_front_of_robot(3)

        return 'done'

class AskOrder(smach.State):
    def __init__(self, robot, location):
        smach.State.__init__(self, outcomes=['next_order','orders_done'])

        self._robot = robot
        self._location = location

    def _confirm(self, tries=3):
        for i in range(0, tries):
            result = self._robot.ears.recognize("<option>", {"option":["yes", "no"]})
            if result and result.result:
                answer = result.result
                return answer == "yes"

            if i != tries - 1:
                self._robot.speech.speak("Please say yes or no")
        return False

    def execute(self, userdata=None):
        self._robot.head.look_at_ground_in_front_of_robot(3)
        self._robot.speech.speak("Which combo or beverage do you want?")

        order = None
        while not order:
            result = self._robot.ears.recognize(knowledge.order_spec, knowledge.order_choices)
            if result and result.result:

                self._robot.speech.speak("I heard %s, is this correct?" % result.result)
                if not self._confirm():
                    continue

                if "beverage" in result.choices:
                    order = result.choices["beverage"]
                    ORDERS["beverage"] = { "location" : self._location, "name" : order }
                elif "food1" and "food2" in result.choices:
                    if result.choices["food1"] == result.choices["food2"]:
                        self._robot.speech.speak("Sorry, I dit not understand, please repeat.")
                    else:
                        order = "%s and %s" % (result.choices["food1"], result.choices["food2"])
                        ORDERS["combo"] = { "location" : self._location, "name" : order }
            else:
                self._robot.speech.speak("Sorry, I dit not understand, please repeat.")

        self._robot.head.cancel_goal()

        self._robot.speech.speak("Ok, I will get you %s"%order, block=False)

        print "\n\n Current orders: \n\n"
        print ORDERS

        if "combo" in ORDERS and "beverage" in ORDERS:
            return "orders_done"
        else:
            return "next_order"

class SpeakOrders(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["spoken"])

        self._robot = robot

    def execute(self, userdata=None):
        self._robot.head.look_at_standing_person()
        self._robot.speech.speak("Mr. Barman I have some orders.")

        sentence_combo = ""
        sentence_beverage = ""
        sentence_final = ""


        if "beverage" in ORDERS:
            sentence_beverage = "Table %s would like to have the beverage %s"%(ORDERS["beverage"]["location"], ORDERS["beverage"]["name"])
            sentence_final = sentence_beverage
        if "combo" in ORDERS:
            sentence_combo = "Table %s wants the combo %s"%(ORDERS["combo"]["location"], ORDERS["combo"]["name"])
            sentence_final = sentence_combo

        ''' add a COMA and an AND abetween sentences better understanding '''
        if "combo" in ORDERS and "beverage" in ORDERS:
            sentence_final = sentence_beverage + ", and " + sentence_combo

        self._robot.speech.speak(sentence_final)

        self._robot.head.cancel_goal()
        return "spoken"

class DeliverOrderWithBasket(smach.StateMachine):
    def __init__(self, robot, order_type):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        beverage_dest_desig = EdEntityDesignator(robot) #.id is overwritten by instruct_barman

        with self:
            @smach.cb_interface(outcomes=['spoken'])
            def instruct_barman(userdata=None):
                try:
                    order = ORDERS[order_type]
                    beverage_dest_desig.id = order['location']
                    robot.speech.speak("Barman, please put a {name} in my basket for table {location}".format(**order))
                except KeyError:
                    rospy.logerr("No beverage in ORDERS")
                return 'spoken'
            smach.StateMachine.add( 'INSTRUCT_BARMAN',
                                    smach.CBState(instruct_barman),
                                    transitions={'spoken'               :'AWAIT_PUT_ORDER_CONFIRMATION'})

            smach.StateMachine.add( 'AWAIT_PUT_ORDER_CONFIRMATION',
                                    states.WaitTime(robot, 8),
                                    transitions={   'waited'            :'GOTO_ORDER_DESTINATION_1',
                                                    'preempted'         :'failed'})

            smach.StateMachine.add( 'GOTO_ORDER_DESTINATION_1', states.NavigateToWaypoint(robot, beverage_dest_desig, radius = WAYPOINT_RADIUS),
                                    transitions={   'arrived'           :'SAY_TAKE_ORDER',
                                                    'unreachable'       :'GOTO_ORDER_DESTINATION_2',
                                                    'goal_not_defined'  :'GOTO_ORDER_DESTINATION_2'})

            smach.StateMachine.add( 'GOTO_ORDER_DESTINATION_2', states.NavigateToWaypoint(robot, beverage_dest_desig, radius = WAYPOINT_RADIUS),
                                    transitions={   'arrived'           :'SAY_TAKE_ORDER',
                                                    'unreachable'       :'failed',
                                                    'goal_not_defined'  :'failed'})

            @smach.cb_interface(outcomes=['spoken'])
            def instruct_guest(userdata=None):
                try:
                    order = ORDERS[order_type]
                    robot.speech.speak("Dear guest at table {location}, you can get your {name} from my basket.".format(**order))
                except KeyError:
                    rospy.logerr("No beverage in ORDERS")
                return 'spoken'
            smach.StateMachine.add( 'SAY_TAKE_ORDER',
                                    smach.CBState(instruct_guest),
                                    transitions={'spoken'               :'AWAIT_TAKE_ORDER_CONFIRMATION'})

            smach.StateMachine.add( 'AWAIT_TAKE_ORDER_CONFIRMATION',
                                    states.WaitTime(robot, 5),
                                    transitions={   'waited'            :'SAY_ENJOY_ORDER',
                                                    'preempted'         :'failed'})

            smach.StateMachine.add( 'SAY_ENJOY_ORDER',
                                    states.Say(robot, ["Enjoy your {}".format(order_type)], block=False),
                                    transitions={   'spoken'            :'succeeded'})

def setup_statemachine(robot):
    load_waypoints(robot)

    operator_id = VariableDesignator(resolve_type=str)

    sm = smach.StateMachine(outcomes=['done', 'aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE',
                               states.Initialize(robot),
                               transitions={'initialized': 'STORE_KITCHEN',
                                            'abort': 'aborted'}
                               )

        smach.StateMachine.add('STORE_KITCHEN',
                               StoreKitchen(robot),
                               transitions={'done': 'HEAD_STRAIGHT'}
                               )

        smach.StateMachine.add('HEAD_STRAIGHT',
                               HeadStraight(robot),
                               transitions={'done': 'SAY_INTRO'}
                               )

        smach.StateMachine.add('SAY_INTRO',
                               states.Say(robot, "Hi, Show me your restaurant please."),
                               transitions={'spoken': 'FOLLOW_INITIAL'}
                               )

        smach.StateMachine.add('FOLLOW_INITIAL',
                               states.FollowOperator(robot,
                                                     operator_timeout=30,
                                                     operator_id_des=operator_id
                                                     ),
                               transitions={'stopped': 'STORE',
                                            'lost_operator': 'FOLLOW_INITIAL',
                                            'no_operator': 'FOLLOW_INITIAL'}
                               )

        smach.StateMachine.add('FOLLOW',
                               states.FollowOperator(robot,
                                                     operator_timeout=30,
                                                     ask_follow=False,
                                                     operator_id_des=operator_id
                                                     ),
                               transitions={'stopped': 'STORE',
                                            'lost_operator': 'FOLLOW_INITIAL',
                                            'no_operator': 'FOLLOW_INITIAL'}
                               )

        smach.StateMachine.add('STORE',
                               StoreWaypoint(robot),
                               transitions={'done': 'CHECK_KNOWLEDGE',
                                            'continue': 'FOLLOW'}
                               )

        smach.StateMachine.add('CHECK_KNOWLEDGE',
                               CheckKnowledge(robot),
                               transitions={'yes': 'SAY_FOLLOW_TO_KITCHEN',
                                            'no':'FOLLOW'}
                               )

        smach.StateMachine.add('SAY_FOLLOW_TO_KITCHEN',
                               states.Say(robot,
                                          "Please bring me back to the kitchen!"
                                          ),
                               transitions={'spoken': 'FOLLOW_TO_KITCHEN'}
                               )

        smach.StateMachine.add('FOLLOW_TO_KITCHEN',
                               states.FollowOperator(robot,
                                                     operator_timeout=30,
                                                     ask_follow=True,
                                                     learn_face=False,
                                                     operator_id_des=operator_id
                                                     ),
                               transitions={'stopped': 'CHECK_IN_KITCHEN',
                                            'lost_operator': 'SAY_GOTO_KITCHEN',
                                            'no_operator': 'SAY_GOTO_KITCHEN'}
                               )

        smach.StateMachine.add('SAY_GOTO_KITCHEN',
                               states.Say(robot,
                                          "You know what? I will go back to the kitchen on my own!",
                                          block=False
                                          ),
                               transitions={'spoken': 'GOTO_KITCHEN'})

        smach.StateMachine.add('GOTO_KITCHEN',
                               states.NavigateToWaypoint(robot,
                                                         EntityByIdDesignator(robot, id="kitchen")
                                                         ),
                               transitions={'arrived': 'SAY_IN_KITCHEN',
                                            'unreachable': 'SAY_I_DONT_KNOW_HOW',
                                            'goal_not_defined': 'SAY_I_DONT_KNOW_HOW'}
                               )

	smach.StateMachine.add('SAY_I_DONT_KNOW_HOW',
				states.Say(robot,
						"Oops, I don't know the way back.",
						block=True
						),
				transitions={'spoken': 'GOTO_KITCHEN'})

#        smach.StateMachine.add('FOLLOW_TO_KITCHEN',
#                               states.FollowOperator(robot,
#                                                     operator_timeout=30,
#                                                     ask_follow=False
#                                                     ),
#                               transitions={'stopped': 'CHECK_IN_KITCHEN',
#                                            'lost_operator': 'FOLLOW_TO_KITCHEN_INITIAL',
#                                            'no_operator': 'FOLLOW_TO_KITCHEN_INITIAL'}
#                               )

        smach.StateMachine.add('CHECK_IN_KITCHEN',
                               CheckInKitchen(robot),
                               transitions={'not_in_kitchen': 'FOLLOW_TO_KITCHEN',
                                            'in_kitchen':'SAY_IN_KITCHEN'}
                               )

        smach.StateMachine.add('SAY_IN_KITCHEN',
                               states.Say(robot,
                                          "We are in the kitchen again!"
                                          ),
                               transitions={'spoken': 'SAY_WHICH_ORDER'}
                               )

        # Where to take the order from?
        smach.StateMachine.add('SAY_WHICH_ORDER', states.Say(robot, "From which table should I take the first order?"), transitions={ 'spoken' :'HEAR_WHICH_ORDER'})
        smach.StateMachine.add('HEAR_WHICH_ORDER', HearWhichTable(robot),
            transitions={ 'no_result' :'SAY_WHICH_ORDER', 'one' : 'FIRST_SAY_TAKE_ORDER_FROM_TABLE_1', 'two': 'FIRST_SAY_TAKE_ORDER_FROM_TABLE_2', 'three' : "FIRST_SAY_TAKE_ORDER_FROM_TABLE_3"})

        # ############## first table ##############
        for i, name in tables.iteritems():
            next_i = i+1
            if next_i > 3:
                next_i = 1

            smach.StateMachine.add('FIRST_SAY_TAKE_ORDER_FROM_TABLE_%d'%i, states.Say(robot, "Okay, I will take an order from table %d"%i, block=False),
                                    transitions={ 'spoken' :'FIRST_NAVIGATE_TO_WAYPOINT_TABLE_%d'%i})
            smach.StateMachine.add('FIRST_NAVIGATE_TO_WAYPOINT_TABLE_%d'%i, states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=name), radius = WAYPOINT_RADIUS),
                                    transitions={'arrived': 'FIRST_ASK_ORDER_TABLE_%d'%i, 'unreachable':'NAVIGATE_TO_WAYPOINT_TABLE_%d'%next_i, 'goal_not_defined':'NAVIGATE_TO_WAYPOINT_TABLE_%d'%next_i})
            smach.StateMachine.add('FIRST_ASK_ORDER_TABLE_%d'%i, AskOrder(robot, name),
                                    transitions={'next_order':'NAVIGATE_TO_WAYPOINT_TABLE_%d'%next_i, 'orders_done' : 'SAY_ORDERS_DONE'})


        # ############## Loop over the reset of the tables until we have a beverage and a combo ##############
        smach.StateMachine.add('NAVIGATE_TO_WAYPOINT_TABLE_1', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="one"), radius = WAYPOINT_RADIUS),
                                transitions={'arrived': 'SAY_IF_ORDER_TABLE_1', 'unreachable':'NAVIGATE_TO_WAYPOINT_TABLE_2', 'goal_not_defined':'NAVIGATE_TO_WAYPOINT_TABLE_2'})
        smach.StateMachine.add('SAY_IF_ORDER_TABLE_1', states.Say(robot, ["Hello, are you ready to order?", "Would you like to order something?"]),
                                transitions={ 'spoken' :'HEAD_DOWN_TABLE_1'})
        smach.StateMachine.add('HEAD_DOWN_TABLE_1', LookAtPersonSitting(robot),
                                transitions={ 'done' :'HEAR_IF_ORDER_TABLE_1'})
        smach.StateMachine.add('HEAR_IF_ORDER_TABLE_1', states.HearOptions(robot, ['yes','no'], timeout = rospy.Duration(10),look_at_standing_person=False),
                                transitions={ 'no_result' :'NAVIGATE_TO_WAYPOINT_TABLE_2', 'yes':'ASK_ORDER_TABLE_1','no':'NAVIGATE_TO_WAYPOINT_TABLE_2'})
        smach.StateMachine.add('ASK_ORDER_TABLE_1', AskOrder(robot, "one"),
                                transitions={'next_order':'NAVIGATE_TO_WAYPOINT_TABLE_2', 'orders_done' : 'SAY_ORDERS_DONE'})


        smach.StateMachine.add('NAVIGATE_TO_WAYPOINT_TABLE_2', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="two"), radius = WAYPOINT_RADIUS),
                                transitions={'arrived': 'SAY_IF_ORDER_TABLE_2', 'unreachable':'NAVIGATE_TO_WAYPOINT_TABLE_3', 'goal_not_defined':'NAVIGATE_TO_WAYPOINT_TABLE_3'})
        smach.StateMachine.add('SAY_IF_ORDER_TABLE_2', states.Say(robot, ["Hello, are you ready to order?", "Would you like to order something?"]),
                                transitions={ 'spoken' :'HEAD_DOWN_TABLE_2'})
        smach.StateMachine.add('HEAD_DOWN_TABLE_2', LookAtPersonSitting(robot),
                                transitions={ 'done' :'HEAR_IF_ORDER_TABLE_2'})
        smach.StateMachine.add('HEAR_IF_ORDER_TABLE_2', states.HearOptions(robot, ['yes','no'], timeout = rospy.Duration(10),look_at_standing_person=False),
                                transitions={ 'no_result' :'NAVIGATE_TO_WAYPOINT_TABLE_3', 'yes':'ASK_ORDER_TABLE_2','no':'NAVIGATE_TO_WAYPOINT_TABLE_3'})
        smach.StateMachine.add('ASK_ORDER_TABLE_2', AskOrder(robot, "two"),
                                transitions={'next_order':'NAVIGATE_TO_WAYPOINT_TABLE_3', 'orders_done' : 'SAY_ORDERS_DONE'})


        smach.StateMachine.add('NAVIGATE_TO_WAYPOINT_TABLE_3', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="three"), radius = WAYPOINT_RADIUS),
                                transitions={'arrived': 'SAY_IF_ORDER_TABLE_3', 'unreachable':'NAVIGATE_TO_WAYPOINT_TABLE_1', 'goal_not_defined':'NAVIGATE_TO_WAYPOINT_TABLE_1'})
        smach.StateMachine.add('SAY_IF_ORDER_TABLE_3', states.Say(robot, ["Hello, are you ready to order?", "Would you like to order something?"]),
                                transitions={ 'spoken' :'HEAD_DOWN_TABLE_3'})
        smach.StateMachine.add('HEAD_DOWN_TABLE_3', LookAtPersonSitting(robot),
                                transitions={ 'done' :'HEAR_IF_ORDER_TABLE_3'})
        smach.StateMachine.add('HEAR_IF_ORDER_TABLE_3', states.HearOptions(robot, ['yes','no'], timeout = rospy.Duration(10),look_at_standing_person=False),
                                transitions={ 'no_result' :'NAVIGATE_TO_WAYPOINT_TABLE_1', 'yes':'ASK_ORDER_TABLE_3','no':'NAVIGATE_TO_WAYPOINT_TABLE_1'})
        smach.StateMachine.add('ASK_ORDER_TABLE_3', AskOrder(robot, "three"),
                                transitions={'next_order':'NAVIGATE_TO_WAYPOINT_TABLE_1', 'orders_done' : 'SAY_ORDERS_DONE'})


        smach.StateMachine.add('SAY_ORDERS_DONE', states.Say(robot, "I received enough orders for now, going back to the kitchen!", block=False),
                                transitions={ 'spoken' :'NAVIGATE_BACK_TO_THE_KITCHEN'})


        smach.StateMachine.add('NAVIGATE_BACK_TO_THE_KITCHEN', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="kitchen"), radius = WAYPOINT_RADIUS),
                                transitions={'arrived': 'SPEAK_ORDERS', 'unreachable':'NAVIGATE_BACK_TO_THE_KITCHEN_BACKUP', 'goal_not_defined':'NAVIGATE_BACK_TO_THE_KITCHEN_BACKUP'})
        smach.StateMachine.add('NAVIGATE_BACK_TO_THE_KITCHEN_BACKUP', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="kitchen"), radius = WAYPOINT_RADIUS+0.2),
                                transitions={'arrived': 'SPEAK_ORDERS', 'unreachable':'NAVIGATE_BACK_TO_THE_KITCHEN_BACKUP_2', 'goal_not_defined':'NAVIGATE_BACK_TO_THE_KITCHEN_BACKUP_2'})
        smach.StateMachine.add('NAVIGATE_BACK_TO_THE_KITCHEN_BACKUP_2', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="kitchen"), radius = WAYPOINT_RADIUS+0.4),
                                transitions={'arrived': 'SPEAK_ORDERS', 'unreachable':'SPEAK_ORDERS', 'goal_not_defined':'SPEAK_ORDERS'})
        smach.StateMachine.add('SPEAK_ORDERS', SpeakOrders(robot),
                                transitions={ 'spoken' :'STORE_BEVERAGE_SIDE'})

        smach.StateMachine.add('STORE_BEVERAGE_SIDE', StoreBeverageSide(robot),
                                transitions={ 'done' : 'NAVIGATE_TO_BEVERAGES'})
        smach.StateMachine.add('NAVIGATE_TO_BEVERAGES', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="beverages"), radius = WAYPOINT_RADIUS),
                                transitions={'arrived': 'SPEAK_I_SEE_THE_BEVERAGES', 'unreachable':'NAVIGATE_TO_BEVERAGES_BACKUP', 'goal_not_defined':'NAVIGATE_TO_BEVERAGES_BACKUP'})
        smach.StateMachine.add('NAVIGATE_TO_BEVERAGES_BACKUP', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="beverages"), radius = WAYPOINT_RADIUS+0.1),
                                transitions={'arrived': 'SPEAK_I_SEE_THE_BEVERAGES', 'unreachable':'STORE_BEVERAGE_SIDE', 'goal_not_defined':'STORE_BEVERAGE_SIDE'})

        smach.StateMachine.add('SPEAK_I_SEE_THE_BEVERAGES', states.Say(robot, "The beverages are in front of me", block=False),
                                transitions={ 'spoken' :'DELIVER_BEVERAGE'})


        smach.StateMachine.add('DELIVER_BEVERAGE', DeliverOrderWithBasket(robot, "beverage"),
                                transitions={'succeeded':'NAVIGATE_TO_KITCHEN', 'failed':'NAVIGATE_TO_KITCHEN'})
        smach.StateMachine.add('NAVIGATE_TO_KITCHEN', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="kitchen"), radius = WAYPOINT_RADIUS),
                                transitions={'arrived': 'DELIVER_COMBO', 'unreachable':'NAVIGATE_TO_KITCHEN_BACKUP', 'goal_not_defined':'NAVIGATE_TO_KITCHEN_BACKUP'})

        smach.StateMachine.add('NAVIGATE_TO_KITCHEN_BACKUP', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="kitchen"), radius = WAYPOINT_RADIUS+0.1),
                                transitions={'arrived': 'DELIVER_COMBO', 'unreachable':'DELIVER_COMBO', 'goal_not_defined':'DELIVER_COMBO'})


        smach.StateMachine.add('DELIVER_COMBO', DeliverOrderWithBasket(robot, "combo"),
                                transitions={'succeeded':'NAVIGATE_BACK_TO_THE_KITCHEN_2', 'failed':'NAVIGATE_BACK_TO_THE_KITCHEN_2'})


        smach.StateMachine.add('NAVIGATE_BACK_TO_THE_KITCHEN_2', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="kitchen"), radius = WAYPOINT_RADIUS),
                                transitions={'arrived': 'SAY_DONE_WITH_CHALLENGE', 'unreachable':'SAY_DONE_WITH_CHALLENGE', 'goal_not_defined':'SAY_DONE_WITH_CHALLENGE'})

        smach.StateMachine.add('SAY_DONE_WITH_CHALLENGE', states.Say(robot, "I'm done with this challenge and you are the banana king!"), transitions={ 'spoken' :'done'})

    analyse_designators(sm, "restaurant")
    return sm


def test_delivery(robot):
    from robot_skills.util.kdl_conversions import kdl_frame_stamped_from_XYZRPY
    robot.ed.update_entity(id="one", kdl_frame_stamped=kdl_frame_stamped_from_XYZRPY(x=1.0, y=0, frame_id="/map"), type="waypoint")
    robot.ed.update_entity(id="two", kdl_frame_stamped=kdl_frame_stamped_from_XYZRPY(x=-1.2, y=0.0, frame_id="/map"), type="waypoint")
    robot.ed.update_entity(id="three", kdl_frame_stamped=kdl_frame_stamped_from_XYZRPY(x=1.950, y=1.551, frame_id="/map"), type="waypoint")

    global ORDERS
    ORDERS = {"beverage":{"name":"coke", "location":"one"}, "combo":{"name":"pringles and chocolate", "location":"two"}}

    deliver = smach.StateMachine(outcomes=['done', 'aborted'])

    with deliver:
        smach.StateMachine.add('DELIVER_BEVERAGE', DeliverOrderWithBasket(robot, "beverage"), transitions={'succeeded':'NAVIGATE_TO_KITCHEN', 'failed':'NAVIGATE_TO_KITCHEN'})
        smach.StateMachine.add('NAVIGATE_TO_KITCHEN', states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id="kitchen"), radius = WAYPOINT_RADIUS),
            transitions={'arrived': 'DELIVER_COMBO', 'unreachable':'DELIVER_COMBO', 'goal_not_defined':'DELIVER_COMBO'})
        smach.StateMachine.add('DELIVER_COMBO', DeliverOrderWithBasket(robot, "combo"), transitions={'succeeded':'done', 'failed':'aborted'})

    deliver.execute(None)

############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('restaurant_exec')

    startup(setup_statemachine, challenge_name="restaurant", argv=[ e for e in sys.argv if e != "--custom" ])

    print "If you want to save the learned locations, do '$ rosparam dump $(rospack find challenge_restaurant)/param/locations.yaml /restaurant_locations/'"
