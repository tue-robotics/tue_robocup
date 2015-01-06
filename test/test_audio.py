#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')

from robot_skills.ears import Ears
from robot_skills.speech import Speech

import random
import time

import sys

import rospy

if len(sys.argv) < 2:
    print "Please specify a robot name"
    sys.exit()

robot_name = sys.argv[1]
if robot_name is not "amigo" and not "sergio":
    print "Unknown robot '%s'"%robot_name
    sys.exit()

rospy.init_node("audio_test")
e = Ears(robot_name)
s = Speech(robot_name)

# data
drinks = ["coke","fanta","beer","milk","yoghurt","pepsi","orangejuice","sevenup"]
names = ["christopher","james","john","robert","michael","william","david","richard","charles","joseph","thomas","mary","patricia","linda","barbara","elizabeth","jennifer","maria","susan","margret","dorothy"]
places = ['bedroom','livingroom', 'hallway', 'kitchen']
object_category = [ 'drink', 'snack', 'food', 'cleaningstuff']
location_category = ['utensil','shelf','seating','table','trashbin','appliance']
actions = ['transport','bring','carry','bring','get','give','point','look','detect','find','move','go','navigate']
sides = ["left","right","front","back"]
numbers = ["one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten"]
restaurant_locations = ["ordering location","food location", "drink location"]

def ask(question, spec, choices):
    s.speak(question)
    print "Choices are: " + str(choices)
    r = e.recognize(spec, choices)


    if r:
        s.speak("You said: %s"%r.result)
    else:
        s.speak("I don't understand, sorry")

    time.sleep(1)

while not rospy.is_shutdown():
    try:
        ask("What would you like to drink?", "<drink>", {"drink":drinks})

        ask("Ask me a EGPSR question", "<action> (me) (a|an|the) <object> (from|of) (the|a|an) <location> (to|of|from) (the|a|an) <location2> ", {"action":actions,"object":object_category,"location":location_category+places,"location2":location_category+places})

        ask(random.choice(["What is your name?","Hey, how should I call you?"]), "<name>", {"name":names})

        for i in range(0,4):
            ask("Waiting for restaurant speech command","(<number> <side>|<location>)", {"side":sides,"number":numbers,"location":restaurant_locations})
    except KeyboardInterrupt:
        break



