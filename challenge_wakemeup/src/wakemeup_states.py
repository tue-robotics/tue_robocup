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
from robocup_knowledge import load_knowledge


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
common = load_knowledge('common')
# load item names
knowledge_objs= load_knowledge('common').objects
names_fruit = [ o["name"] for o in knowledge_objs if "sub-category" in o and o["sub-category"] is "fruit" ]
names_cereal = [ o["name"] for o in knowledge_objs if "sub-category" in o and o["sub-category"] is "cereal" ]
names_milk = [ o["name"] for o in knowledge_objs if "sub-category" in o and o["sub-category"] is "milk" ]


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

    print prefix + item + " was not matched!"
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
        self.breakfastFruit.current = ""
        self.breakfastFruit.current = ""

        sentence = Designator("((<beginning> <item1> <preposition> <item2> <preposition> <item3>) | \
                                (<item1> <preposition> <item2> <preposition> <item3>) | \
                                (<beginning> <item1> <preposition> <item2>) | \
                                (<item1> <preposition> <item2>))")

        choices = Designator({  "beginning" :   ["I want", "I would like", "a", "one"],
                                "preposition" : ["and", "and a", "and an", "with a", "with a"],
                                "item1" :       names_cereal + names_fruit + names_milk,
                                "item2" :       names_cereal + names_fruit + names_milk,
                                "item3" :       names_cereal + names_fruit + names_milk})

        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        state = HearOptionsExtra(self.robot, sentence, choices, answer)
        outcome = state.execute()

        # process response
        if outcome == "heard":

            # resolve words separatey since some of them might not have been caught
            try:
                word_beginning = answer.resolve().choices["beginning"]
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving 'word_beginning': " + str(ke) + bcolors.ENDC
                pass

            try:
                word_item3 = answer.resolve().choices["item3"]
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving 'item3': " + str(ke) + bcolors.ENDC
                pass

            try:
                word_preposition = answer.resolve().choices["preposition"]
                word_item1 = answer.resolve().choices["item1"]
                word_item2 = answer.resolve().choices["item2"]
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving words from breakfast request: " + str(ke) + bcolors.ENDC
                pass

            print "{}What was heard: {} {} {} {} {} {}".format(prefix, word_beginning , word_item1 , word_preposition ,  word_item2 , word_preposition , word_item3)

            # print "{}Response: fruit = {}, cereal = {} , milk = {}".format(prefix, self.breakfastFruit.resolve(), self.breakfastCereal.resolve(), self.breakfastMilk.resolve())

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

            # print "{}Response: fruit = {}, cereal = {} , milk = {}".format(prefix, self.breakfastFruit.resolve(), self.breakfastCereal.resolve(), self.breakfastMilk.resolve())

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

            # print "{}Response: fruit = {}, cereal = {} , milk = {}".format(prefix, self.breakfastFruit.resolve(), self.breakfastCereal.resolve(), self.breakfastMilk.resolve())

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

                # print "{}Response: fruit = {}, cereal = {} , milk = {}".format(prefix, self.breakfastFruit.resolve(), self.breakfastCereal.resolve(), self.breakfastMilk.resolve())

                # just a consistency check
                if not got_milk:
                    print prefix + "Still don't know what type of milk it is!! Reseting to 'fresh milk'" + bcolors.ENDC
                    self.breakfastMilk.current = "fresh milk"

            else:
                self.breakfastMilk.current = default_milk
                got_milk = True

            print "{}Response: fruit = {}, cereal = {} , milk = {}".format(prefix, self.breakfastFruit.resolve(), self.breakfastCereal.resolve(), self.breakfastMilk.resolve())
            
            if not self.breakfastCereal.resolve() or not self.breakfastFruit.resolve() or not self.breakfastMilk.resolve() :
                heard_correctly = False
                print prefix + bcolors.FAIL + "One of the food types was empty"+ bcolors.ENDC
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
        smach.State.__init__(   self, 
                                outcomes=['done'])

        self.robot = robot
        self.breakfastCereal = breakfastCerealDes
        self.breakfastFruit = breakfastFruitDes
        self.breakfastMilk = breakfastMilkDes


    def execute(self, userdata):
        print prefix + bcolors.OKBLUE + "RepeatOrderToPerson" + bcolors.ENDC

        self.robot.speech.speak("I will get you a " +  self.breakfastFruit.resolve() + " and " + self.breakfastCereal.resolve() + " with " + self.breakfastMilk.resolve(), block=False)

        return 'done'
