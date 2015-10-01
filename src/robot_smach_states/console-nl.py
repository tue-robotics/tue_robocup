#! /usr/bin/python

import sys

import rospy
import importlib
import threading

from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint
from robot_smach_states.manipulation import Grab, Place
from robot_smach_states.util.designators import Designator, UnoccupiedArmDesignator, EdEntityDesignator, ArmHoldingEntityDesignator

ROBOTS = {}
ACTION = None

# ----------------------------------------------------------------------------------------------------

class SmachAction:

    def __init__(self, machine):
        self.machine = machine
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.machine.execute)
        self.thread.start()

    def cancel(self):
        if self.machine.is_running:
            self.machine.request_preempt()

        # Wait until canceled
        self.thread.join()

# ----------------------------------------------------------------------------------------------------

def say(msg):
    print ""
    print "    ROBOT: %s" % msg
    print ""

# ----------------------------------------------------------------------------------------------------

def stop():
    global ACTION
    if not ACTION:
        return

    ACTION.cancel()
    ACTION = None

# ----------------------------------------------------------------------------------------------------

def length_sq(x, y):
    return x * x + y * y

# ----------------------------------------------------------------------------------------------------

def parse_object(p, robot):
    id = None
    type = None
    size = None
    color = None

    entities = robot.ed.get_entities(parse=False)
    ids = [e.id for e in entities]
    types = [e.type for e in entities]

    # Simply skip articles if they are there
    if p.read("a", "an", "the"):
        pass       

    # Check adjectives
    while True:
        if p.read("red", "green", "blue", "black", "white" "pink", "purple", "orange", "brown", "grey", "yellow"):    
            color = p.last_read[0]
        elif p.read("small", "big", "medium"):
            size = p.last_read[0]
        elif p.read("nearest", "closest"):
            pass # simply skip: we will by default take the nearest object that fulfills the description
        else:
            break

    w_next_l = p.read_var(1)
    if w_next_l:
        w_next = w_next_l[0]

        if w_next in ids:
            id = w_next
        elif w_next in types:
            type = w_next
        elif not w_next in ["object", "entity", "thing"]:
            say("What is a '%s'?" % w_next)
            return

    if p.read("on the", "from the"):
        loc_l = p.read_var(1)
        if loc_l:
            loc = loc[0]

    if id:
        e = robot.ed.get_entity(id=id, parse=False)
        if e:
            return [e]
        else:
            say("There is no entity with id '%s'" % id)
            return []
    elif type:
        entities = robot.ed.get_entities(type=type, parse=False)
    else:
        entities = robot.ed.get_entities(parse=False)

    robot_pos = robot.base.get_location().pose.position

    # Sort entities by distance
    entities = sorted(entities, key=lambda entity: length_sq(robot_pos.x - entity.pose.position.x, robot_pos.y - entity.pose.position.y))

    return entities

# ----------------------------------------------------------------------------------------------------

def grab(p, robot):

    entities = parse_object(p, robot)

    if not entities:
        say("No such entities!")
        return

    # Only filter to entities that do not have a shape but do have a convex hull
    entities = [ e for e in entities if not e.has_shape and len(e.convex_hull) > 0]

    if not entities:
        say("I am sorry, but I cannot grab that object")
        return

    arm = robot.leftArm
    machine = Grab(robot, arm=UnoccupiedArmDesignator(robot.arms, arm), item=EdEntityDesignator(robot, id=entities[0].id))

    stop()
    ACTION = SmachAction(machine)
    ACTION.start()

# ----------------------------------------------------------------------------------------------------

def move(p, robot):

    entities = parse_object(p, robot)

    if not entities:
        say("No such entities!")
        return

    machine = NavigateToObserve(robot, entity_designator=EdEntityDesignator(robot, id=entities[0].id), radius=.5)

    stop()
    ACTION = SmachAction(machine)
    ACTION.start()

# ----------------------------------------------------------------------------------------------------  

def lookat(p, robot):

    move(p, robot)

# ----------------------------------------------------------------------------------------------------

def get_robot(name):
    if name in ROBOTS:
        return ROBOTS[name]

    try:
        module = importlib.import_module('robot_skills.' + name)
        constructor = module.__getattribute__(name.title())
    except ImportError:
        return None

    instance = constructor(wait_services=False)

    ROBOTS[name] = instance
    return instance

# ----------------------------------------------------------------------------------------------------

class Parser:

    def __init__(self, command):
        self.command = command
        self.last_read = []

    def read(self, *args):
        for s in args:
            words = s.split()
            n = len(words)
            if self.command[0:n] == words[0:n]:
                self.command = self.command[n:]
                self.last_read = words
                return True

        return False

    def read_var(self, n = 1):
        v = self.command[0:n]
        self.command = self.command[n:]
        return v

# ----------------------------------------------------------------------------------------------------

def help():
    print """
    Type a command explaining what you want the robot to do. For example:

        go to the bed

        grab the object on the table

    Other commands:

        exit - Exits the console
        """

def main():

    rospy.init_node("robot_console", anonymous=True, log_level=rospy.ERROR, disable_signals=True)

    robot = None

    while True:
        try:
            command = raw_input("> ")
        except KeyboardInterrupt:
            print ""
            command = "exit"

        if not command:
            continue
        elif command in ["help"]:
            help()
        elif command in ["quit", "exit"]:
            break
        else:
            words = command.split()

            p = Parser(words)

            if p.read("amigo", "sergio"):
                robot = get_robot(p.last_read[0])
                if not robot:
                    print "\n    Could not connect to robot\n"
                    continue

            if not robot:
                print  "\n    Please select a robot. For example: \"amigo go to the table\"\n"
                continue

            if p.read("grab", "grasp", "pick up"):
                grab(p, robot)
            elif p.read("goto", "go to", "move to", "navigate to"):
                move(p, robot)
            elif p.read("look at", "lookat"):
                lookat(p, robot)
            elif p.read("stop"):
                stop()
            else:
                say("I don't understand")

if __name__ == "__main__":
    sys.exit(main())
