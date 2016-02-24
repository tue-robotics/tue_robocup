#! /usr/bin/python

import os
import cfgparser
import sys
import rospy

from robocup_knowledge import load_knowledge

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

class CommandRecognizer:

    def __init__(self, grammar_file, challenge_knowledge):
        self.parser = cfgparser.CFGParser.fromfile(grammar_file)

        for (furniture, objects) in challenge_knowledge.furniture_to_objects.iteritems():
            #parser.add_rule("FURNITURE[\"%s\"] -> %s" % (furniture, furniture))
            self.parser.add_rule("FURNITURE[\"%s\"] -> the %s" % (furniture, furniture))
            self.parser.add_rule("FURNITURE[\"%s\"] -> a %s" % (furniture, furniture))

            for obj in objects:
                #parser.add_rule("SMALL_OBJECT[\"%s\"] -> %s" % (obj, obj))
                self.parser.add_rule("SMALL_OBJECT[\"%s\"] -> the %s" % (obj, obj))
                self.parser.add_rule("SMALL_OBJECT[\"%s\"] -> a %s" % (obj, obj))

        for rooms in challenge_knowledge.rooms:
            #parser.add_rule("ROOM[\"%s\"] -> %s" % (rooms, rooms))
            self.parser.add_rule("ROOM[\"%s\"] -> the %s" % (rooms, rooms))

        for (alias, obj) in challenge_knowledge.object_aliases.iteritems():
            #parser.add_rule("NP[\"%s\"] -> %s" % (obj, alias))
            self.parser.add_rule("NP[\"%s\"] -> the %s" % (obj, alias))
            self.parser.add_rule("NP[\"%s\"] -> a %s" % (obj, alias))

        self.grammar_string = unwrap_grammar("T", self.parser)
        

    def recognize(self, robot):
        sentence = robot.ears.recognize(self.grammar_string).result

        if not sentence:
            return None

        return (sentence, self.parser.parse("T", sentence.strip().split(" ")))