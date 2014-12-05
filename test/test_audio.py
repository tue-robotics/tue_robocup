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
names = ["james","john","robert","michael","william","david","richard","charles","joseph","thomas","mary","patricia","linda","barbara","elizabeth","jennifer","maria","susan","margret","dorothy"]
places = ['bedroom','livingroom', 'hallway', 'kitchen']
object_category = [ 'drink', 'snack', 'food', 'cleaningstuff']
location_category = ['utensil','shelf','seating','table','trashbin','appliance']
actions = ['transport','bring','carry','bring','get','give','point','look','detect','find','move','go','navigate']

while True:
    s.speak("What would you like to drink?")
    r = e.recognize("<drink>",{"drink":drinks})
    if r:
        s.speak(random.choice(["Nice, I also like %s","%s, that's an excellent choice!","Realy, %s?","Ok, I will get you a %s"])%r.choices["drink"])
    else:
        s.speak("I don't understand, sorry")

    time.sleep(1)

    s.speak("Ask me a difficult Erik double GPSR question")
    r = e.recognize("<action> (me) (a|an|the) <object> (from|of) (the|a|an) <location> (to|of|from) (the|a|an) <location2> ",{"action":actions,"object":object_category,"location":location_category+places,"location2":location_category+places})

    if r:
        s.speak("Ok, I will %s a %s from the %s to the %s"%(r.choices["action"],r.choices["object"],r.choices["location"],r.choices["location2"]))
    else:
        s.speak("I don't understand, sorry")

    time.sleep(1)

    s.speak(random.choice(["What is your name?","Hey, how should I call you?"]))
    r = e.recognize("<name>",{"name":names})
    if r:
        s.speak(random.choice(["Nice, I will call you %s","%s, that's an beautiful name!","Realy, %s?","Ok, from now on, you are %s"])%r.choices["name"])
    else:
        s.speak("I don't understand, sorry")

    time.sleep(1)
