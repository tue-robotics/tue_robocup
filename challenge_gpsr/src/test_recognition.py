#! /usr/bin/python

import os
import cfgparser
import sys
import rospy

from robocup_knowledge import load_knowledge

challenge_knowledge = load_knowledge('challenge_gpsr')

# ----------------------------------------------------------------------------------------------------

def unwrap_grammar(lname, parser):
    if not lname in parser.rules:
        return ""

    rule = parser.rules[lname]

    s = ""

    opt_strings = []
    for opt in rule.options:
        conj_strings = []

        for conj in opt.conjuncts:
            if conj.is_variable:
                unwrapped_string = unwrap_grammar(conj.name, parser)
                if unwrapped_string:
                    conj_strings.append(unwrapped_string)
            else:
                conj_strings.append(conj.name)

        opt_strings.append(" ".join(conj_strings))

    s = "|".join(opt_strings)

    if len(rule.options) > 1:
        s = "(" + s + ")"

    return s

# ----------------------------------------------------------------------------------------------------

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

    parser = cfgparser.CFGParser.fromfile(os.path.dirname(sys.argv[0]) + "/grammar.fcfg")

    for (furniture, objects) in challenge_knowledge.furniture_to_objects.iteritems():
        for obj in objects:
            # object_to_location[obj] = furniture
            parser.add_rule("SMALL_OBJECT[\"%s\"] -> %s" % (obj, obj))
            parser.add_rule("SMALL_OBJECT[\"%s\"] -> the %s" % (obj, obj))
            parser.add_rule("SMALL_OBJECT[\"%s\"] -> a %s" % (obj, obj))

    for rooms in challenge_knowledge.rooms:
        parser.add_rule("ROOM[\"%s\"] -> %s" % (rooms, rooms))
        parser.add_rule("ROOM[\"%s\"] -> the %s" % (rooms, rooms))

    for (alias, obj) in challenge_knowledge.object_aliases.iteritems():
            parser.add_rule("NP[\"%s\"] -> %s" % (obj, alias))
            parser.add_rule("NP[\"%s\"] -> the %s" % (obj, alias))
            parser.add_rule("NP[\"%s\"] -> a %s" % (obj, alias))

    grammar_string = unwrap_grammar("T", parser)

    robot.head.look_at_standing_person()

    while not rospy.is_shutdown():
        print robot.ears.recognize(grammar_string)

# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
