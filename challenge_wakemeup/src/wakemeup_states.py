#! /usr/bin/env python
import roslib;
import rospy
import smach
import subprocess
import inspect
# import random
# import ed_perception.msg
import robot_skills.util.msg_constructors as msgs
import geometry_msgs.msg as gm
# import math
from smach_ros import SimpleActionState
from robot_smach_states.util.designators import *
from robot_smach_states.human_interaction.human_interaction import HearOptionsExtra
from ed.msg import EntityInfo
from dragonfly_speech_recognition.srv import GetSpeechResponse
from robot_smach_states.util.geometry_helpers import *

from robocup_knowledge import load_knowledge
knowledge_objs = load_knowledge('common').objects
knowledge = load_knowledge('challenge_wakemeup')

# ----------------------------------------------------------------------------------------------------

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


prefix = bcolors.OKBLUE + "[WAKE ME UP] " + bcolors.ENDC

default_milk = "fresh milk"

# load item names
names_fruit = [ o["name"] for o in knowledge_objs if "sub-category" in o and o["sub-category"] is "fruit" ]
names_cereal = [ o["name"] for o in knowledge_objs if "sub-category" in o and o["sub-category"] is "cereal" ]
names_milk = [ o["name"] for o in knowledge_objs if "sub-category" in o and o["sub-category"] is "milk" ]

# # Debug print
# print prefix + "Fruit names from Knowledge: " + str(names_fruit)
# print prefix + "Cereal names from Knowledge: " + str(names_cereal)
# print prefix + "Milk names from Knowledge: " + str(names_milk)

# ----------------------------------------------------------------------------------------------------


class FoodType:
    Cereal = 0
    Fruit = 1
    Milk = 2


def parseFoodType(item, got_fruit, got_cereal, got_milk):
    # print prefix + bcolors.OKBLUE + "parseFoodType" + bcolors.ENDC

    # if a fruit has not been picked, you can search there
    if not got_fruit and item in names_fruit:
        # print prefix + item + " its a fruit!"
        return FoodType.Fruit

    # if a cereal has not been picked, you can search there
    if not got_cereal and item in names_cereal:
        # print prefix + item + " its a cereal"
        return FoodType.Cereal

    # if a milk has not been picked, you can search there
    if not got_milk and item in names_milk:
        # print prefix + item + " its a milk!"
        return FoodType.Milk

    # print prefix + item + " was not matched!"
    return None

    
# ----------------------------------------------------------------------------------------------------


# Ask the persons name
class GetOrder(smach.State):
    def __init__(self, robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
        smach.State.__init__(   self, 
                                outcomes=['succeeded', 'failed'])

        self.robot = robot
        self.breakfastCereal = breakfastCerealDes
        self.breakfastFruit = breakfastFruitDes
        self.breakfastMilk = breakfastMilkDes


    def execute(self, userdata):
        print prefix + bcolors.OKBLUE + "GetOrder" + bcolors.ENDC

        # import ipdb; ipdb.set_trace()
        # Initializations

        got_cereal = False
        got_fruit = False
        got_milk = False
        heard_correctly = False

        word_beginning = ""
        word_preposition = ""
        word_item1 = ""
        word_item2 = ""
        word_item3 = ""

        self.breakfastFruit.current = ""
        self.breakfastCereal.current = ""
        self.breakfastMilk.current = ""

        # define allowed sentences, [] means optional
        sentence = Designator("(([<beginning>] <item1> [<preposition>] <item2> [<preposition>] <item3>) | \
                                ([<beginning>] <item1> [<preposition>] <item2>))")

        choices = Designator({  "beginning" :   ["I want", "I would like", "a", "one"],
                                "preposition" : ["and", "and a", "and an", "with a", "with a", "with", "a"],
                                "item1" :       names_cereal + names_fruit + names_milk,
                                "item2" :       names_cereal + names_fruit + names_milk,
                                "item3" :       names_cereal + names_fruit + names_milk})

        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        state = HearOptionsExtra(self.robot, sentence, choices, answer, time_out=rospy.Duration(20))
        outcome = state.execute()

        # process response
        if outcome == "heard":

            # resolve words separatey since some of them might not have been caught
            try:
                word_beginning = answer.resolve().choices["beginning"]
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving: " + str(ke) + bcolors.ENDC
                pass

            try:
                word_item3 = answer.resolve().choices["item3"]
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving: " + str(ke) + bcolors.ENDC
                pass

            try:
                word_preposition = answer.resolve().choices["preposition"]
                word_item1 = answer.resolve().choices["item1"]
                word_item2 = answer.resolve().choices["item2"]
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving: " + str(ke) + bcolors.ENDC
                pass

            print "{}What was heard: {} {} {} {} {} {}".format(prefix, word_beginning , word_item1 , word_preposition ,  word_item2 , word_preposition , word_item3)

            # find first item's type
            if parseFoodType(word_item1, got_fruit, got_cereal, got_milk) == FoodType.Fruit:
                self.breakfastFruit.current = word_item1
                got_fruit = True
                print "{}First item fruit".format(prefix)
            elif parseFoodType(word_item1, got_fruit, got_cereal, got_milk) == FoodType.Cereal:
                self.breakfastCereal.current = word_item1
                got_cereal = True
                print "{}First item cereal".format(prefix)
            elif parseFoodType(word_item1, got_fruit, got_cereal, got_milk) == FoodType.Milk:
                self.breakfastMilk.current = word_item1
                got_milk = True
                print "{}First item milk".format(prefix)
            else:
                print "{}Could not get a match with word_item1 = {}".format(prefix, word_item1)

            # find second item's type
            if parseFoodType(word_item2, got_fruit, got_cereal, got_milk) == FoodType.Fruit:
                self.breakfastFruit.current = word_item2
                got_fruit = True
                print "{}Second item Fruit".format(prefix)
            elif parseFoodType(word_item2, got_fruit, got_cereal, got_milk) == FoodType.Cereal:
                self.breakfastCereal.current = word_item2
                got_cereal = True
                print "{}Second item Cereal".format(prefix)
            elif parseFoodType(word_item2, got_fruit, got_cereal, got_milk) == FoodType.Milk:
                self.breakfastMilk.current = word_item2
                got_milk = True
                print "{}Second item Milk".format(prefix)
            else:
                print "{}Could not get a match with word_item2 = {}".format(prefix, word_item2)

            # third type might not exist if its milk
            if word_item3 :

                # find second item's type
                if parseFoodType(word_item3, got_fruit, got_cereal, got_milk) == FoodType.Fruit :
                    self.breakfastFruit.current = word_item3
                    got_fruit = True
                    print "{}Third item Fruit".format(prefix)
                elif parseFoodType(word_item3, got_fruit, got_cereal, got_milk) == FoodType.Cereal :
                    self.breakfastCereal.current = word_item3
                    got_cereal = True
                    print "{}Third item Cereal".format(prefix)
                elif parseFoodType(word_item3, got_fruit, got_cereal, got_milk) == FoodType.Milk :
                    self.breakfastMilk.current = word_item3
                    got_milk = True
                    print "{}Third item Milk".format(prefix)
                else:
                    print "{}Could not get a match with word_item3 = {}".format(prefix, word_item3)

                # just a consistency check
                if not got_milk:
                    print prefix + "Still don't know what type of milk it is! Reseting to " + default_milk + bcolors.ENDC
                    self.breakfastMilk.current = default_milk

            else:
                self.breakfastMilk.current = default_milk
                got_milk = True

            print "{}Response: fruit = {}, cereal = {} , milk = {}".format(prefix, self.breakfastFruit.resolve(), self.breakfastCereal.resolve(), self.breakfastMilk.resolve())
            
            if not self.breakfastCereal.resolve() or not self.breakfastFruit.resolve() or not self.breakfastMilk.resolve() :
                heard_correctly = False
                print prefix + bcolors.FAIL + "One of the food types was empty" + bcolors.ENDC
            else:
                heard_correctly = True

        else:
            heard_correctly = False

        # rospy.sleep(2)

        if heard_correctly:
            return 'succeeded'
        else:
            return 'failed'


# ----------------------------------------------------------------------------------------------------


# Ask the persons name
class RepeatOrderToPerson(smach.State):
    def __init__(self, robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
        smach.State.__init__(self, outcomes=['done'])

        self.robot = robot
        self.breakfastCereal = breakfastCerealDes
        self.breakfastFruit = breakfastFruitDes
        self.breakfastMilk = breakfastMilkDes

    def execute(self, userdata):
        print prefix + bcolors.OKBLUE + "RepeatOrderToPerson" + bcolors.ENDC

        self.robot.speech.speak("I will get you a " +  self.breakfastFruit.resolve() + " and " + self.breakfastCereal.resolve() + " with " + self.breakfastMilk.resolve(), block=False)

        return 'done'


# ----------------------------------------------------------------------------------------------------


class CancelHeadGoals(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, robot):
        print prefix + bcolors.OKBLUE + "CancelHeadGoals" + bcolors.ENDC

        self.robot.head.cancel_goal()

        return 'done'


# ----------------------------------------------------------------------------------------------------


class LookAtBedTop(smach.State):
    def __init__(self, robot, entity_id, wakeup_light_color):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot
        self.bed = self.robot.ed.get_entity(id=entity_id)
        self.r = wakeup_light_color[0]
        self.g = wakeup_light_color[1]
        self.b = wakeup_light_color[2]

    def execute(self, robot):
        print prefix + bcolors.OKBLUE + "LookAtBedTop" + bcolors.ENDC

        # set robots pose
        # self.robot.spindle.high()
        self.robot.head.cancel_goal()
        self.robot.lights.set_color(self.r,self.g,self.b)

        # TODO maybe look around a bit to make sure the vision covers the whole bed top

        # look at bed top
        headGoal = msgs.PointStamped(x=self.bed.pose.position.x, y=self.bed.pose.position.y, z=self.bed.pose.position.z+self.bed.z_max, frame_id="/map")
        self.robot.head.look_at_point(point_stamped=headGoal, end_time=0, timeout=4)

        return 'succeeded'


# ----------------------------------------------------------------------------------------------------


# class LookIfSomethingsThere(smach.State):
#     def __init__(self, robot, designator, timeout=0):
#         smach.State.__init__(self, outcomes=['awake', 'not_awake'])
#         self.robot = robot
#         self.designator = designator
#         self.timeout = rospy.Duration(timeout)

#     def execute(self, robot):
#         self.start_time = rospy.Time.now()
#         print (rospy.Time.now() - self.start_time).secs
#         print rospy.Time.now() - self.start_time < self.timeout
#         while rospy.Time.now() - self.start_time < self.timeout:
#             if self.designator.resolve():
#                 self.robot.lights.set_color(0,0,1)
#                 return 'awake'
#             else:
#                 rospy.sleep(0.2)

#         return 'not_awake'

# ----------------------------------------------------------------------------------------------------


class LookIfSomethingsThere(smach.State):
    def __init__(self, robot, designator, timeout=0, sleep=0.2):
        smach.State.__init__(self, outcomes=['there', 'not_there'])
        self.robot = robot
        self.designator = designator
        self.timeout = rospy.Duration(timeout)
        self.sleep = sleep

    def execute(self, robot):
        self.start_time = rospy.Time.now()
        print (rospy.Time.now() - self.start_time).secs
        print rospy.Time.now() - self.start_time < self.timeout
        while rospy.Time.now() - self.start_time < self.timeout:
            if self.designator.resolve():
                self.robot.lights.set_color(0,0,1)
                return 'there'
            else:
                rospy.sleep(self.sleep)

        return 'not_there'

# ----------------------------------------------------------------------------------------------------

class Evaluate(smach.State)
    def __init__(self, options, designator):
        smach.State.__init__(self, outcomes=['all_succeeded','partly_succeeded','all_failed'])
        self.options = options
        self.results = designator.resolve()
        self.something_failed = False
        self.something_succeeded = False

    def execute(self, robot):
        for option in options:
            if self.results[option]:
                something_succeeded = True
            else:
                something_failed = True

        if something_succeeded && something_failed:
            return 'partly_succeeded'
        else if something_succeeded && !something_failed:
            return 'all_succeeded'
        else if !something_succeeded && something_failed:
            return 'all_failed'
        else
            return 'all_failed'

# ----------------------------------------------------------------------------------------------------

class addPositive(smach.State)
    def __init__(self, results_designator, item_designator):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.results = results_designator.resolve()
        self.item = item_designator.resolve()

    def execute(self,robot):
        try:
            if not self.results[self.item]
        except Exception, e:
            print self.item + " is not one of the items in the dictionary."
            raise e
        self.results[self.item] = True
        return


# ----------------------------------------------------------------------------------------------------

class SelectItem(smach.State):
    def __init__(self, robot, options, asked_items, generic_item, specific_item):
        smach.State.__init__(self, outcomes=['selected', 'all_done'])
        self.robot = robot
        self.options = options
        self.asked_items = asked_items
        self.count = len(self.options)
        self.current = 0
        self.generic_item = generic_item
        self.specific_item = specific_item

    def execute(self, robot):
        self.generic_item.current = self.options[current]

        category_items = [i for i in objects if 'sub-category' in i and i['sub-category']==self.generic_item.resolve()]
        self.specific_item.current = list(set(category_items).intersection(asked_items))[0]
        
        self.current++
        if self.current == self.count:
            self.current = 0
            return 'all_done'
        return 'selected'

# ----------------------------------------------------------------------------------------------------

class FindItem(smach.State):
    def __init__(self, robot, goal_item, result):
        smach.State.__init__(self, outcomes=['item_found', 'not_found'])
        self.robot = robot
        self.goal = goal_item
        self.result = result
        self.goal_type = o in objects if 'name' in o and o['name']==goal_item
        
        milk_objects = [o for o in objects if 'sub-category' in o and o['sub-category']=='milk']

    def execute(self,robot):
        entity_ids = self.robot.ed.segment_kinect(max_sensor_range = 2.0)
        filtered_ids = []
        for entity_id in entity_ids:
            e = self.robot.ed.get_entity(entity_id)

            # if e and onTopOff(e, shelf_entity) and not e.type:
            #     filtered_ids.append(e.id)

        entity_types = self.robot.ed.classify(ids=id_list, types=OBJECT_TYPES)







