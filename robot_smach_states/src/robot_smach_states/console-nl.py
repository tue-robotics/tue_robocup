#! /usr/bin/python

# System
import cmd
import importlib
import multiprocessing
import sys

# ROS
import rospy

# TU/e Robotics
from robot_smach_states.manipulation import Grab, Place
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint
import robot_smach_states.util.designators as ds

ROBOTS = {}

global ACTION
ACTION = None

WORDS = ["grab", "grasp", "pick", "up", "goto", "go", "to", "move", "navigate",
         "lookat", "look", "at", "show", "stop", "quit", "exit", "the", "an", "a",
         "red", "green", "blue", "black", "white" "pink", "purple", "orange",
         "brown", "grey", "yellow", "small", "big", "medium", "nearest", "closest",
         "on", "from", "object", "entity", "thing"]

# ----------------------------------------------------------------------------------------------------


class bcolors:
    RED    = '\033[91m'
    ORANGE = '\033[93m'
    BLUE   = '\033[94m'
    GREEN  = '\033[92m'
    ENDC   = '\033[0m'

    @staticmethod
    def wrap(msg, color):
        return "%s%s%s" % (color, msg, bcolors.ENDC)

# ----------------------------------------------------------------------------------------------------


class SmachAction:

    def __init__(self, machine):
        self.machine = machine
        self.thread = None

    def _run(self):
        try:
            self.machine.execute()
        except Exception:
            pass

    def start(self):
        self.thread = multiprocessing.Process(target=self._run)
        self.thread.start()

    def cancel(self):
        if self.machine.is_running:
            self.machine.request_preempt()

        # Wait until canceled
        #self.thread.join()

    def kill(self):
        try:
            self.thread.terminate()
        except Exception:
            pass

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
    types = set([i for sublist in [e.super_types for e in entities] for i in sublist])

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

    robot_location = robot.base.get_location()
    robot_pos = robot_location.frame.p

    # Sort entities by distance
    entities = sorted(entities, key=lambda entity: entity.distance_to_2d(robot_pos))

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
    machine = Grab(robot, arm=ds.UnoccupiedArmDesignator(robot.arms, arm), item=ds.EntityByIdDesignator(robot, id=entities[0].id))

    stop()

    global ACTION
    ACTION = SmachAction(machine)
    ACTION.start()

# ----------------------------------------------------------------------------------------------------

def move(p, robot):

    entities = parse_object(p, robot)

    if not entities:
        say("No such entities!")
        return

    machine = NavigateToObserve(robot, entity_designator=ds.EntityByIdDesignator(robot, id=entities[0].id), radius=.5)

    stop()

    global ACTION
    ACTION = SmachAction(machine)
    ACTION.start()

# ----------------------------------------------------------------------------------------------------

def lookat(p, robot):

    move(p, robot)

# ----------------------------------------------------------------------------------------------------

def show(p, robot):

    if (p.read("ids", "entities", "objects")):
        entities = robot.ed.get_entities(parse=False)
        print "\n    The following entities are in the world model:\n"

        for e in entities:
            print "      - %s (%s)" % (bcolors.wrap(e.id, bcolors.GREEN), bcolors.wrap(", ".join(e.types), bcolors.BLUE))

        print ""
    else:
        print "\n    I don't understand what you want me to show.\n"

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

class REPL(cmd.Cmd):

    def __init__(self):
        cmd.Cmd.__init__(self)
        self.prompt = bcolors.wrap("> ", bcolors.BLUE)
        self.robot = None
        self.use_rawinput = True

    def emptyline(self):
        pass

    def do_help(self, str):
        print """
    Type a command explaining what you want the robot to do. For example:

        go to the bed

        grab the object on the table

    Other commands:

        exit - Exits the console
        """

    def default(self, command):
        if not command:
            return False
        elif command in ["help"]:
            help()
        elif command in ["quit", "exit"]:
            return True  # True means interpreter has to stop
        else:
            words = command.split()

            p = Parser(words)

            if p.read("amigo", "sergio"):
                self.robot = get_robot(p.last_read[0])
                if not self.robot:
                    print "\n    Could not connect to robot\n"
                    return False

            if not self.robot:
                print  "\n    Please select a robot. For example: \"amigo go to the table\"\n"
                return False

            if p.read("grab", "grasp", "pick up"):
                grab(p, self.robot)
            elif p.read("goto", "go to", "move to", "navigate to"):
                move(p, self.robot)
            elif p.read("look at", "lookat"):
                lookat(p, self.robot)
            elif p.read("show"):
                show(p, self.robot)
            elif p.read("stop"):
                stop()
            else:
                say("I don't understand")

        return False # Signals interpreter to continue

    def completedefault(self, text, line, begidx, endidx):
        # text: current word, line: full line

        words = WORDS

        if self.robot:
            entities = self.robot.ed.get_entities(parse=False)
            words += [e.id for e in entities]
            words += [i for sublist in [e.types for e in entities] for i in sublist]

        return [w for w in words if w.startswith(text)]


# ----------------------------------------------------------------------------------------------------

def main():

    rospy.init_node("robot_console", anonymous=True, log_level=rospy.ERROR, disable_signals=True)

    robot = None

    try:
        repl = REPL()
        repl.cmdloop()
    except KeyboardInterrupt:
        pass

    # If there is still an action going on, kill it (we don't want to wait)
    global ACTION
    if ACTION:
        ACTION.kill()

if __name__ == "__main__":
    sys.exit(main())
