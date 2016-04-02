#! /usr/bin/python

import os
import cfgparser
import sys
import rospy

import yaml

from robocup_knowledge import load_knowledge
challenge_knowledge = load_knowledge('challenge_gpsr')

# ----------------------------------------------------------------------------------------------------

class EntityDescription(object):
    def __init__(self, id=None, type=None, location=None, category=None):
        self.id = id
        self.type = type
        self.location = location
        self.category = category
        self.is_undefined = False

    def serialize(self):
        d = {}
        if self.id:
            d["id"] = self.id
        if self.type:
            d["type"] = self.type         
        if self.location:
            d["loc"] = self.location.serialize()   
        if self.category:
            d["cat"] = self.category    

        return d  
  
    def __repr__(self):
        return "(id={}, type={}, location={}, category={})".format(self.id, self.type,
                                                             self.location, self.category)

# ----------------------------------------------------------------------------------------------------

def resolve_entity_description(parameters):
    descr = EntityDescription()

    if isinstance(parameters, str):
        descr.id = parameters

    elif "special" in parameters:
        special = parameters["special"]
        if special =="it":
            descr.is_undefined = True
        elif special == "operator":
            descr.id = "gpsr_starting_pose"
    else:
        if "id" in parameters:
            descr.id = parameters["id"]
        if "type" in parameters:
            descr.type = parameters["type"]
        if "loc" in parameters:
            descr.location = resolve_entity_description(parameters["loc"])
        if "cat" in parameters:
            descr.category = parameters["cat"]

    return descr

# ----------------------------------------------------------------------------------------------------

def hear(robot, sentence, options):
    while True:
        robot.speech.speak(sentence)
        res = robot.ears.recognize("<loc>", {"loc": options})
        if not res:
            robot.speech.speak("Sorry, I could not understand")
        else:
            break

    robot.speech.speak("OK, {}".format(res.result))

    return res.result

# ----------------------------------------------------------------------------------------------------


def fill_in_gaps(robot, parameters, last_entity, last_location, context):
    new_last_loc = None

    entity = resolve_entity_description(parameters)

    if entity.is_undefined:
        if not last_entity:
            entity.id = hear(robot, "What do you want me to {}?".format(context["action"]),
                              challenge_knowledge.common.object_names)
        else:
            entity = last_entity

    elif entity.id:
        if entity.type == "person":
            if not entity.location:
                entity.location = last_location
            if not entity.location:
                loc =hear(robot, "Where can I find {}?".format(entity.id),
                          challenge_knowledge.common.rooms)
                entity.location = EntityDescription(id=loc)
        else:
            new_last_loc = entity.id

    elif entity.type:
        if not entity.location:
            entity.location = last_location
        if not entity.location:
                loc = hear(robot, "Where can I find {}?".format(entity.id),
                          list(set([o["room"] for o in challenge_knowledge.common.locations])))
                entity.location = EntityDescription(id=loc)
    elif entity.category:
        entity.type = hear(robot, "What kind of {}?".format(entity.category), object_names)

    print entity.serialize()

    return (entity.serialize(), entity, new_last_loc)

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

def resolve_name(name, challenge_knowledge):
    if name in challenge_knowledge.translations:
        return challenge_knowledge.translations[name]
    else:
        return name

# ----------------------------------------------------------------------------------------------------

class CommandRecognizer:

    def __init__(self, grammar_file, challenge_knowledge):
        self.parser = cfgparser.CFGParser.fromfile(grammar_file)

        for obj in challenge_knowledge.common.object_names:
            self.parser.add_rule("SMALL_OBJECT[\"%s\"] -> %s" % (obj, resolve_name(obj, challenge_knowledge)))

        location_names = list(set([o["name"] for o in challenge_knowledge.common.locations]))          

        for loc in location_names:
            #parser.add_rule("FURNITURE[\"%s\"] -> %s" % (furniture, furniture))
            self.parser.add_rule("FURNITURE[\"%s\"] -> %s" % (loc, resolve_name(loc, challenge_knowledge)))

        for name in challenge_knowledge.common.names:
            self.parser.add_rule("NAME[\"%s\"] -> %s" % (name.lower(), name.lower()))

            # for obj in objects:
            #     #parser.add_rule("SMALL_OBJECT[\"%s\"] -> %s" % (obj, obj))
            #     self.parser.add_rule("SMALL_OBJECT[\"%s\"] -> the %s" % (obj, obj))
            #     self.parser.add_rule("SMALL_OBJECT[\"%s\"] -> a %s" % (obj, obj))

        for room in challenge_knowledge.rooms:
            #parser.add_rule("ROOM[\"%s\"] -> %s" % (rooms, rooms))
            self.parser.add_rule("ROOM[\"%s\"] -> %s" % (room, resolve_name(room, challenge_knowledge)))

        for obj_cat in challenge_knowledge.common.object_categories:
            self.parser.add_rule("OBJ_CAT[\"%s\"] -> %s" % (obj_cat, obj_cat)) 

        for container in challenge_knowledge.common.get_object_names(category="container"):
            self.parser.add_rule("CONTAINER[\"%s\"] -> %s" % (container, container))            

        # for (alias, obj) in challenge_knowledge.object_aliases.iteritems():
        #     #parser.add_rule("NP[\"%s\"] -> %s" % (obj, alias))
        #     self.parser.add_rule("NP[\"%s\"] -> the %s" % (obj, alias))
        #     self.parser.add_rule("NP[\"%s\"] -> a %s" % (obj, alias))

        self.grammar_string = unwrap_grammar("T", self.parser)

        self.knowledge = challenge_knowledge

        # print len(self.grammar_string)

    def parse(self, sentence, robot):
        semantics_str = self.parser.parse("T", sentence.lower().strip().split(" "))

        if not semantics_str:
            return None

        semantics = yaml.load(semantics_str)

        actions = []
        if "action1" in semantics:
            actions += [semantics["action1"]]
        if "action2" in semantics:
            actions += [semantics["action2"]]
        if "action3" in semantics:
            actions += [semantics["action3"]]

        print actions

        last_location = None
        last_entity = None

        actions_resolved = []
        for a in actions:
            if "entity" in a:
                (e_new, le, ll) = fill_in_gaps(robot, a["entity"], last_entity, last_location, a)
                a["entity"] = e_new
                last_location = ll
                last_entity = le                
            if "to" in a:
                (e_new, le, ll) = fill_in_gaps(robot, a["to"], last_entity, last_location, a)
                a["to"] = e_new
                last_location = ll
                last_entity = le
               
        return (sentence, yaml.dump(semantics))        

    def recognize(self, robot):
        sentence = robot.ears.recognize(self.grammar_string).result

        if not sentence:
            return None

        return self.parse(sentence, robot)