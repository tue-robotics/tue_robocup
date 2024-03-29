#! /usr/bin/env python

from __future__ import print_function

# System
import random
import time

# ROS
import rospy

# TU/e Robotics
from robot_skills import get_robot_from_argv

rospy.init_node("audio_test")
robot = get_robot_from_argv(index=1)

hmi = robot.hmi
s = robot.speech
robot.head.look_at_standing_person()

# data
drinks = ["coke","fanta","beer","milk","yoghurt","pepsi","orangejuice","sevenup"]
names = ["christopher","james","john","robert","michael","william","david","richard","charles","joseph","thomas","mary","patricia","linda","barbara","elizabeth","jennifer","maria","susan","margret","dorothy"]
places = ['bedroom','livingroom', 'hallway', 'kitchen']
object_category = [ 'drink', 'snack', 'food', 'cleaningstuff']
location_category = ['utensil','shelf','seating','table','trashbin','appliance']
actions = ['transport','bring','carry','bring','get','give','point','look','detect','find','move','go','navigate']
restaurant_grammar = """O[P] -> ORDER[P] | can i have a ORDER[P] | i would like ORDER[P] | can i get ORDER[P] | could i have ORDER[P] | may i get ORDER[P] | bring me ORDER[P]
ORDER[OO] -> COMBO[OO] | BEVERAGE[OO]
BEVERAGE[{"beverage": B}] -> BEV[B]
BEVERAGE[{"beverage": B}] -> DET BEV[B]
COMBO[{"food1": F1, "food2": F2}] -> FOOD[F1] and FOOD[F2] | FOOD[F1] with FOOD[F2]
COMBO[{"food1": F1, "food2": F2}] -> DET FOOD[F1] and FOOD[F2] | DET FOOD[F1] with FOOD[F2]
COMBO[{"food1": F1, "food2": F2}] -> FOOD[F1] and DET FOOD[F2] | FOOD[F1] with DET FOOD[F2]
COMBO[{"food1": F1, "food2": F2}] -> DET FOOD[F1] and DET FOOD[F2] | DET FOOD[F1] with DET FOOD[F2]
DET -> a | an
BEV['coke'] -> coke[B]
BEV['fanta'] -> fanta[B]
FOOD['pizza'] -> pizza[B]
BEV['steak'] -> steak[B]"""


def ask(question, grammar, target):
    s.speak(question)
    print("Grammar: " + str(grammar))
    r = hmi.query(question, grammar, target)

    if r:
        s.speak("You said: %s" % r.sentence)
    else:
        s.speak("I don't understand, sorry")

    time.sleep(1)


while not rospy.is_shutdown():
    ask("What would you like to drink?", "T -> " + "|".join(drinks), "T")

    # ask("Ask me a EGPSR question", "<action> (me) (a|an|the) <object> (from|of) (the|a|an) <location> (to|of|from) (the|a|an) <location2> ",
    #     {"action":actions,
    #      "object":object_category,
    #      "location":location_category+places,
    #      "location2":location_category+places})

    ask(random.choice(["What is your name?", "Hey, how should I call you?"]), "Name -> " +"|".join(names), "Name")

    for i in range(0,4):
        ask("What can I get for you?", restaurant_grammar, "O")
