#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Sjoerd van den Dries, 2016

# TODO:
# - initial pose estimate
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
from command_recognizer import CommandRecognizer
from datetime import datetime


challenge_knowledge = load_knowledge('challenge_gpsr')
speech_data = load_knowledge('challenge_speech_recognition')

# ------------------------------------------------------------------------------------------------------------------------

class GPSR:

    def __init__(self):
        self.entity_ids = []
        self.entity_type_to_id = {}
        self.object_to_location = {}

        self.last_entity_id = None

    def resolve_entity_id(self, description):
        if isinstance(description, str):
            if description == "it":
                return self.last_entity_id
            elif description == "operator":
                return "initial_pose"             
            else:
                return description

    # ------------------------------------------------------------------------------------------------------------------------

    def navigate(self, robot, parameters):
        entity_id = self.resolve_entity_id(parameters["entity"])
        self.last_entity_id = entity_id

        robot.speech.speak("I am going to the %s" % entity_id, block=False)

        if entity_id in challenge_knowledge.rooms:
            nwc =  NavigateToSymbolic(robot, 
                                            { EntityByIdDesignator(robot, id=entity_id) : "in" }, 
                                              EntityByIdDesignator(robot, id="dinnertable"))
        else:
            nwc = NavigateToObserve(robot,
                                 entity_designator=robot_smach_states.util.designators.EdEntityDesignator(robot, id=entity_id),
                                 radius=.5)

        nwc.execute()

    # ------------------------------------------------------------------------------------------------------------------------

    def answer_question(self, robot, parameters):
        robot.head.look_at_ground_in_front_of_robot(100)

        res = robot.ears.recognize(spec=speech_data.spec,
                                   choices=speech_data.choices,
                                   time_out=rospy.Duration(15))

        if not res:
            robot.speech.speak("My ears are not working properly, can i get a restart?.")

        if res:
            if "question" in res.choices:
                rospy.loginfo("Question was: '%s'?"%res.result)
                robot.speech.speak("The answer is %s"%speech_data.choice_answer_mapping[res.choices['question']])
            else:
                robot.speech.speak("Sorry, I do not understand your question")

    # ------------------------------------------------------------------------------------------------------------------------

    def say(self, robot, parameters):
        sentence = parameters["sentence"]
        rospy.loginfo('Answering %s', sentence)

        if sentence == 'TIME':
            line = datetime.now().strftime('The time is %H %M')
        elif sentence == "NAME":
            line = 'My name is %s' % robot.robot_name
        elif sentence == 'DAY_OF_MONTH':
            line = datetime.now().strftime('It is day %d of the month')
        elif sentence == 'DAY_OF_WEEK':
            day = datetime.today().weekday() + 1 # weekday() monday is 0
            line = 'It is day %d of the week' % day
        else:
            line = sentence

        robot.speech.speak(line)

    # ------------------------------------------------------------------------------------------------------------------------

    def pick_up(self, robot, parameters):
        entity_id = self.resolve_entity_id(parameters["entity"])
        self.last_entity_id = entity_id

        if "from" in parameters:
            location = self.resolve_entity_id(parameters["from"])
        else:
            location = self.object_to_location[entity_id]

        robot.speech.speak("I am going to the %s to pick up the %s" % (location, entity_id), block=False)


        # Move to the location
        nwc = NavigateToObserve(robot,
                         entity_designator=robot_smach_states.util.designators.EdEntityDesignator(robot, id=location),
                         radius=.5)
        nwc.execute()

        robot.speech.speak("I should grab a %s, but that is not yet implemented" % entity_id, block=False)

    # ------------------------------------------------------------------------------------------------------------------------

    def bring(self, robot, parameters):

        if parameters["entity"] != "it":
            self.pick_up(robot, parameters)

        to_id = self.resolve_entity_id(parameters["to"])

        # Move to the location
        nwc = NavigateToObserve(robot,
                         entity_designator=robot_smach_states.util.designators.EdEntityDesignator(robot, id=to_id),
                         radius=.5)
        nwc.execute()

    # ------------------------------------------------------------------------------------------------------------------------
    # TODO
    # ------------------------------------------------------------------------------------------------------------------------

    def find(self, robot, parameters):
        entity_id = self.resolve_entity_id(parameters["entity"])

        room = self.last_entity_id
        if not room:
            robot.speech.speak("I don't know where to start looking")
            return
        if room not in challenge_knowledge.rooms:
            robot.speech.speech("I don't understand the location")
            return

        locations = challenge_knowledge.room_to_grab_locations[room]
        rospy.loginfo("Location search list: %s", str(locations))

        for location in locations:
            robot.speech.speak("I'm going to search the %s" % location)
            nav = NavigateToSymbolic(robot, {
                    EdEntityDesignator(robot, id=room) : "in",
                    EdEntityDesignator(robot, id=location) : "in_front_of"
                }, EdEntityDesignator(robot, id=location))
            nav.execute()

            robot.speech.speak("Is the %s here?" % entity_id)
            # TODO: inspection logic

        robot.speech.speak("I could not find the %s" % entity_id)

    # ------------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------------

    def execute_command(self, robot, command_recognizer, action_functions, sentence=None):

        if sentence:
            res = command_recognizer.parse(sentence)        
        else:
            res = command_recognizer.recognize(robot)
            print res

        if not res:
            robot.speech.speak("Sorry, I could not understand")
            return False

        (sentence, semantics_str) = res
        print "Sentence: %s" % sentence
        print "Semantics: %s" % semantics_str

        # TODO: re-state the command

        robot.speech.speak("Alright!", block=False)

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

    def run(self):
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

        command_recognizer = CommandRecognizer(os.path.dirname(sys.argv[0]) + "/grammar.fcfg", challenge_knowledge)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

        # Query world model for entities
        entities = robot.ed.get_entities(parse=False)
        for e in entities:
            self.entity_ids += [e.id]

            for t in e.types:
                if not t in self.entity_type_to_id:
                    self.entity_type_to_id[t] = [e.id]
                else:
                    self.entity_type_to_id[t] += [e.id]


        for (furniture, objects) in challenge_knowledge.furniture_to_objects.iteritems():
            for obj in objects:
                self.object_to_location[obj] = furniture

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

        action_functions = {}
        action_functions["navigate"] = self.navigate
        action_functions["find"] = self.find
        action_functions["answer-question"] = self.answer_question
        action_functions["pick-up"] = self.pick_up
        action_functions["bring"] = self.bring
        action_functions["say"] =  self.say

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

        sentence = " ".join([word for word in sys.argv[2:] if word[0] != '_'])

        if sentence:
            self.execute_command(robot, command_recognizer, action_functions, sentence)
        else:
            robot.head.look_at_standing_person()

            robot.speech.speak("What can I do for you?")

            self.execute_command(robot, command_recognizer, action_functions)

# ------------------------------------------------------------------------------------------------------------------------
    
if __name__ == "__main__":
    gpsr = GPSR()
    sys.exit(gpsr.run())
