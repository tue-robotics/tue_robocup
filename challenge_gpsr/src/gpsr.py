#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Sjoerd van den Dries, 2016

# TODO:
# - initial pose estimate
# - get command from speech recognition
# - Also allow object types (e.g., table, chair, etc) to be parsed. If I try that now, the parser raises and exception
#   for some reason
# - Implement all actions (grab, place, bring, look at, etc)
# - Derive object locations from their type (the TC will announce where e.g. a coke can be found)

# ------------------------------------------------------------------------------------------------------------------------

import os
import sys
import yaml
import cfgparser
import rospy

import robot_smach_states
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states.util.designators import EdEntityDesignator, EntityByIdDesignator, VariableDesignator, DeferToRuntime, analyse_designators
from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('challenge_gpsr')

entity_ids = []
entity_type_to_id = {}
object_to_location = {}

# ------------------------------------------------------------------------------------------------------------------------

def navigate(robot, parameters):
    entity_descr = parameters["entity"]

    if entity_descr in challenge_knowledge.rooms:
        nwc =  NavigateToSymbolic(robot, 
                                        { EntityByIdDesignator(robot, id=entity_descr) : "in" }, 
                                          EntityByIdDesignator(robot, id="dinnertable"))
    else:
        nwc = NavigateToObserve(robot,
                             entity_designator=robot_smach_states.util.designators.EdEntityDesignator(robot, id=entity_descr),
                             radius=.5)

    nwc.execute()

# ------------------------------------------------------------------------------------------------------------------------

def answer_question(robot, parameters):
    answer = parameters["answer"]
    robot.speech.speak("My answer to the question is: %s" % answer)

# ------------------------------------------------------------------------------------------------------------------------

def pick_up(robot, parameters):
    entity_descr = parameters["entity"]

    location = object_to_location[entity_descr]

    # Move to the location
    nwc = NavigateToObserve(robot,
                     entity_designator=robot_smach_states.util.designators.EdEntityDesignator(robot, id=location),
                     radius=.5)
    nwc.execute()

    robot.speech.speak("I should grab a %s, but that is not implemented yet" % entity_descr)

# ------------------------------------------------------------------------------------------------------------------------
# TODO
# ------------------------------------------------------------------------------------------------------------------------

def find(robot, parameters):
    entity_descr = parameters["entity"]

    robot.speech.speak("I should find a %s, but that is not implemented yet" % entity_descr)

# ------------------------------------------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------------------------------------------

def execute_command(sentence, parser, action_functions, robot):
    semantics_str = parser.parse("T", sentence)

    if not semantics_str:
        print "Cannot parse sentence"
        return 1

    semantics = yaml.load(semantics_str)

    actions = []
    if "action1" in semantics:
        actions += [semantics["action1"]]
    if "action2" in semantics:
        actions += [semantics["action2"]]
    if "action3" in semantics:
        actions += [semantics["action3"]]

    for a in actions:
        action_type = a["action"]

        if action_type in action_functions:
            action_functions[action_type](robot, a)
        else:
            print "Unknown action type: '%s'" % action_type

# ------------------------------------------------------------------------------------------------------------------------

def main():
    rospy.init_node("gpsr")

    if len(sys.argv) < 2:
        print "Please specify a robot name 'amigo / sergio'"
        return 1

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        print "unknown robot"
        return 1

    robot = Robot()

    if len(sys.argv) < 3:
        print "Please provide a command"
        return 1

    sentence = sys.argv[2:]

    # Remove commas
    for i in range(0, len(sentence)):
        sentence[i] = sentence[i].replace(",","")

    parser = cfgparser.CFGParser.fromfile(os.path.dirname(sys.argv[0]) + "/grammar.fcfg")

    global entity_ids
    global entity_type_to_id
    global object_to_location

    # Query world model for entities
    entities = robot.ed.get_entities(parse=False)
    for e in entities:
        entity_ids += [e.id]

        parser.add_rule("NP[\"%s\"] -> %s" % (e.id, e.id))
        parser.add_rule("NP[\"%s\"] -> the %s" % (e.id, e.id))
        parser.add_rule("NP[\"%s\"] -> a %s" % (e.id, e.id))  

        for t in e.types:
            if not t in entity_type_to_id:
                entity_type_to_id[t] = [e.id]
            else:
                entity_type_to_id[t] += [e.id]

    #for type in entity_type_to_id.keys():
    #    if type not in entity_ids:
    #        parser.add_rule("NP[\"%s\"] -> %s" % (type, type))
    #        parser.add_rule("NP[\"%s\"] -> the %s" % (type, type))
    #        parser.add_rule("NP[\"%s\"] -> a %s" % (type, type)) 

    for (furniture, objects) in challenge_knowledge.furniture_to_objects.iteritems():
        for obj in objects:
            object_to_location[obj] = furniture
            parser.add_rule("SMALL_OBJECT[\"%s\"] -> %s" % (obj, obj))
            parser.add_rule("SMALL_OBJECT[\"%s\"] -> the %s" % (obj, obj))
            parser.add_rule("SMALL_OBJECT[\"%s\"] -> a %s" % (obj, obj))

    for rooms in challenge_knowledge.rooms:
        parser.add_rule("ROOM[\"%s\"] -> %s" % (rooms, rooms))
        parser.add_rule("ROOM[\"%s\"] -> the %s" % (rooms, rooms))

    action_functions = {}
    action_functions["navigate"] = navigate
    action_functions["find"] = find
    action_functions["answer-question"] = answer_question
    action_functions["pick-up"] = pick_up

    execute_command(sentence, parser, action_functions, robot)
    
if __name__ == "__main__":
    sys.exit(main())
