#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------

import os
import sys
import yaml
import time
import cfgparser
import rospy
import random
import std_msgs
import argparse

import robot_smach_states
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic
from robot_smach_states import SegmentObjects, Grab, Place, HandoverToHuman
from robot_smach_states.util.designators import EdEntityDesignator, EntityByIdDesignator, VariableDesignator, DeferToRuntime, analyse_designators, UnoccupiedArmDesignator, EmptySpotDesignator, OccupiedArmDesignator
from robot_smach_states.utility import Initialize
from robot_skills.util import transformations
from robot_skills.classification_result import ClassificationResult
from robocup_knowledge import load_knowledge
from command_recognizer import CommandRecognizer
from find_person import FindPerson
from datetime import datetime, timedelta
import robot_smach_states.util.designators as ds

from robot_smach_states import LookAtArea, StartChallengeRobust

challenge_knowledge = load_knowledge('challenge_open')
speech_data = load_knowledge('challenge_speech_recognition')

# ------------------------------------------------------------------------------------------------------------------------

class EntityDescription(object):
    def __init__(self, id=None, type=None, location=None):
        self.id = id
        self.type = type
        self.location = location

    def __repr__(self):
        return "(id={}, type={}, location={})".format(self.id, self.type, self.location)

# ------------------------------------------------------------------------------------------------------------------------

def not_implemented(robot, parameters):
    rospy.logerr("This was not implemented, show this to Sjoerd: {}".format(parameters))
    robot.speech.speak("Not implemented! Warn Sjoerd", block=False)
    return

# ------------------------------------------------------------------------------------------------------------------------

class GPSR:

    def __init__(self, robot):
        self.entity_ids = []
        self.entity_type_to_id = {}
        self.object_to_location = {}

        self.last_location = None
        self.last_entity = None

        self.command_data = {}
        self.wait_for_trigger = True

        if robot.robot_name == "amigo":
            self._trigger_sub = rospy.Subscriber("/amigo/trigger", std_msgs.msg.String, self._trigger_callback, queue_size=1)
        elif robot.robot_name == "sergio":
            self._trigger_sub = rospy.Subscriber("/sergio/trigger", std_msgs.msg.String, self._trigger_callback, queue_size=1)
            self.pub_trigger = rospy.Publisher('/amigo/trigger', std_msgs.msg.String)

        self.robot = robot

    def _trigger_callback(self, msg):
        self.wait_for_trigger = False

        if msg.data != "gpsr":
            self.command_data = yaml.load(msg.data)

    def send_trigger(self, msg):
        self.pub_trigger.publish(msg)

    def resolve_entity_description(self, parameters):
        descr = EntityDescription()

        if isinstance(parameters, str):
            descr.id = parameters

        elif "special" in parameters:
            special = parameters["special"]
            if special =="it":
                descr = self.last_entity
            elif special == "operator":
                descr.id = "gpsr_starting_pose"
        else:
            if "id" in parameters:
                descr.id = parameters["id"]
            if "type" in parameters:
                descr.type = parameters["type"]
            if "loc" in parameters:
                descr.location = self.resolve_entity_description(parameters["loc"])

        print descr.id
        if not self.robot.ed.get_entity(id=descr.id, parse=False):
            self.robot.speech.speak("I do not know where the {} is".format(descr.id))
            return None

        return descr

    # ------------------------------------------------------------------------------------------------------------------------

    def move_robot(self, robot, id=None, type=None, nav_area=None, loc=None):

        if id in challenge_knowledge.rooms:
            # Driving to a room

            nwc =  NavigateToSymbolic(robot,
                                            { EntityByIdDesignator(robot, id=id) : "in" },
                                              EntityByIdDesignator(robot, id=id))
            nwc.execute()
        elif type == "person":
            # Driving to a person

            if id:
                nwc =  NavigateToSymbolic(robot,
                                                { EntityByIdDesignator(robot, id=id) : "in" },
                                                  EntityByIdDesignator(robot, id=id))
            elif loc:

                if loc in challenge_knowledge.rooms:
                    room_des = EdEntityDesignator(robot, id=loc)
                    f = FindPerson(robot, room_des)
                    result = f.execute()
                else:
                    # TODO
                    self.move_robot(robot, id=loc)
                    robot.base.force_drive(0, 0, 3.1415 / 4, 4)
                    # robot.speech.speak("I need to find a person near this location, but can't do this yet! Ask Janno!")
                    f = FindPerson(robot, None)
                    result = f.execute()

            else:
                robot.speech.speak("I don't know where I can find the person")

        elif challenge_knowledge.is_location(id):
            # Driving to a location

            if not nav_area:
                nav_area = challenge_knowledge.common.get_inspect_position(id)

            location_des = ds.EntityByIdDesignator(robot, id=id)

            nwc = NavigateToSymbolic( robot,
                  {location_des : nav_area},
                  location_des)

            nwc.execute()
        else:
            # Driving to anything else (e.g. a waypoint)
            nwc = NavigateToObserve(robot, EntityByIdDesignator(robot, id=id))
            nwc.execute()

    # ------------------------------------------------------------------------------------------------------------------------

    def navigate(self, robot, parameters):
        entity_descr = self.resolve_entity_description(parameters["entity"])

        if not entity_descr:
            return

        if not entity_descr.location:
            entity_descr.location = self.last_location

        if entity_descr.type == "person":
            if not entity_descr.location:
                robot.speech.speak("Person location undefined")
                return

            self.move_robot(robot, entity_descr.id, entity_descr.type, loc=entity_descr.location.id)

        elif not entity_descr.id:
            not_implemented(robot, parameters)

        else:
            robot.speech.speak("I am going to the %s" % entity_descr.id, block=False)
            print entity_descr
            self.move_robot(robot, entity_descr.id, entity_descr.type)
            self.last_location = entity_descr

    # ------------------------------------------------------------------------------------------------------------------------

    def answer_question(self, robot, parameters):

        robot.head.look_at_standing_person()
        robot.head.wait_for_motion_done()

        robot.speech.speak("What is your question?")

        res = robot.ears.recognize(spec=speech_data.spec,
                                   choices=speech_data.choices,
                                   time_out=rospy.Duration(15))

        if not res:
            robot.speech.speak("My ears are not working properly, sorry!")

        if res:
            if "question" in res.choices:
                rospy.loginfo("Question was: '%s'?"%res.result)
                robot.speech.speak("The answer is %s" % speech_data.choice_answer_mapping[res.choices['question']])
            else:
                robot.speech.speak("Sorry, I do not understand your question")

    # ------------------------------------------------------------------------------------------------------------------------

    def say(self, robot, parameters):
        sentence = parameters["sentence"]
        rospy.loginfo('Answering %s', sentence)

        if sentence == 'TIME':
            hours = datetime.now().hour
            minutes = datetime.now().minute
            line = "The time is {} {}".format(hours, minutes)
        elif sentence == "ROBOT_NAME":
            line = 'My name is %s' % robot.robot_name
        elif sentence == 'TODAY':
            line = datetime.today().strftime('Today is %A %B %d')
        elif sentence == 'TOMORROW':
            line = (datetime.today() + timedelta(days=1)).strftime('Tomorrow is %A %B %d')
        elif sentence == 'DAY_OF_MONTH':
            line = datetime.now().strftime('It is day %d of the month')
        elif sentence == 'DAY_OF_WEEK':
            line = datetime.today().strftime('Today is a %A')
        else:
            line = sentence

        robot.speech.speak(line)

    # ------------------------------------------------------------------------------------------------------------------------

    def find_and_pick_up(self, robot, parameters, pick_up=True):
        entity_descr = self.resolve_entity_description(parameters["entity"])

        if not entity_descr:
            return

        if not entity_descr.location:
            entity_descr.location = self.last_location

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        if entity_descr.type == "person":

            if not entity_descr.location:
                robot.speech.speak("Person location undefined")
                return

            self.move_robot(robot, id=entity_descr.id, type=entity_descr.type, loc=entity_descr.location.id)
            return

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        self.last_entity = entity_descr

        if entity_descr.location:
            room_or_location = entity_descr.location.id

            if room_or_location in challenge_knowledge.rooms:
                locations = [loc["name"] for loc in challenge_knowledge.common.locations
                             if loc["room"] == room_or_location and loc["manipulation"] == "yes"]
            else:
                locations = [room_or_location]

            locations_with_areas = []
            for location in locations:
                locations_with_areas += [(location, challenge_knowledge.common.get_inspect_areas(location))]
        else:
            obj_cat = None
            for obj in challenge_knowledge.common.objects:
                if obj["name"] == entity_descr.type:
                    obj_cat = obj["category"]

            location = challenge_knowledge.common.category_locations[obj_cat].keys()[0]
            area_name = challenge_knowledge.common.category_locations[obj_cat].values()[0]

            locations_with_areas = [(location, [area_name])]

            robot.speech.speak("The {} is a {}, which is stored on the {}".format(entity_descr.type, obj_cat, location), block=False)

        location_defined = (len(locations_with_areas) == 1)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        possible_entities = []

        for loc_and_areas in locations_with_areas:

            (location, area_names) = loc_and_areas

            robot.speech.speak("Going to the %s" % location, block=False)

            last_nav_area = None

            for area_name in area_names:

                nav_area = challenge_knowledge.common.get_inspect_position(location, area_name)

                if nav_area != last_nav_area:

                    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                    # Move to the location

                    self.move_robot(robot, id=location, nav_area=nav_area)
                    last_nav_area = nav_area

                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                # Look at the area

                look_sm = LookAtArea(robot,
                                     EdEntityDesignator(robot, id=location),
                                     area_name)
                look_sm.execute()

                time.sleep(1)

                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                # Segment

                segmented_entities = robot.ed.update_kinect("{} {}".format(area_name, location))

                found_entity_ids = segmented_entities.new_ids + segmented_entities.updated_ids

                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                # Classify

                entity_types_and_probs = robot.ed.classify(ids=found_entity_ids,
                                                           types=challenge_knowledge.common.objects)

                best_prob = 0
                for det in entity_types_and_probs:
                    if det.type == entity_descr.type and det.probability > best_prob:
                        entity_descr.id = det.id
                        best_prob = det.probability

                if not entity_descr.id:
                    possible_entities += found_entity_ids
                else:
                    robot.speech.speak("Found the {}!".format(entity_descr.type), block=False)

                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

                if entity_descr.id:
                    break

            if entity_descr.id:
                break

        if not entity_descr.id:
            if not possible_entities:
                robot.speech.speak("I really can't find the {}!".format(entity_descr.type), block=False)
            else:
                closest_entity_id = None
                closest_distance = None
                for entity_id in possible_entities:
                    entity = robot.ed.get_entity(id=entity_id, parse=False)
                    if not entity:
                        continue

                    p = transformations.tf_transform(entity.pose.position, "/map",
                                                 robot.robot_name+"/base_link",
                                                 robot.tf_listener)
                    distance = p.x*p.x + p.y*p.y

                    if not closest_entity_id or distance < closest_distance:
                        closest_entity_id = entity_id
                        closest_distance = distance

                entity_descr.id = closest_entity_id

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        if pick_up and entity_descr.id:

            if robot.robot_name == "sergio":
                robot.speech.speak("But, wait a minute! I can't pick this up, I have no arms! Let's call my friend amigo!")
                self.send_trigger(yaml.dump(self.command_data))
                return

            robot.speech.speak("Going to grab the {}".format(entity_descr.type))

            # grab it
            grab = Grab(robot, EdEntityDesignator(robot, id=entity_descr.id),
                 UnoccupiedArmDesignator(robot.arms, robot.leftArm, name="empty_arm_designator"))
            result = grab.execute()

    # ------------------------------------------------------------------------------------------------------------------------

    def bring(self, robot, parameters):

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Check if need to grab an entity and if so, do so

        if "entity" in parameters:
            entity_descr = self.resolve_entity_description(parameters["entity"])

            if not self.last_entity or entity_descr.type != self.last_entity.type:
                self.find_and_pick_up(robot, parameters)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Deliver it

        to_descr = self.resolve_entity_description(parameters["to"])

        if not to_descr:
            return

        if to_descr.type == "person" or to_descr.id == "gpsr_starting_pose":
            if to_descr.location:
                self.move_robot(robot, id=to_descr.id, type=to_descr.type, loc=to_descr.location.id)
            else:
                self.move_robot(robot, id=to_descr.id, type=to_descr.type)

            arm_des = OccupiedArmDesignator(robot.arms, robot.leftArm)

            if not arm_des.resolve():
                robot.speech.speak("I don't have anything to give to you")
            else:
                h = HandoverToHuman(robot, arm_des)
                result = h.execute()
        else:
            # Move to the location
            self.move_robot(robot, id=to_descr.id, nav_area="in_front_of")

            # place
            arm = OccupiedArmDesignator(robot.arms, robot.leftArm)

            if not arm.resolve():
                robot.speech.speak("I don't have anything to place")
            else:
                current_item = EdEntityDesignator(robot)
                location_des = EntityByIdDesignator(robot, id=to_descr.id)
                place_position = EmptySpotDesignator(robot, location_des, area='on_top_of')
                p = Place(robot, current_item, place_position, arm)
                result = p.execute()

                if result != 'done':
                    robot.speech.speak("Sorry, my fault")

        self.last_location = None
        self.last_entity = None

    # ------------------------------------------------------------------------------------------------------------------------

    def find(self, robot, parameters):
        self.find_and_pick_up(robot, parameters, pick_up=False)

    # ------------------------------------------------------------------------------------------------------------------------

    def start_challenge(self, robot):
        s = StartChallengeRobust(robot, challenge_knowledge.starting_point)
        s.execute()

    # ------------------------------------------------------------------------------------------------------------------------

    def execute_command(self, robot, command_recognizer, action_functions, mock_sentence=None):

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # If sentence is given on command-line

        if mock_sentence:
            res = command_recognizer.parse(mock_sentence)
            if not res:
                robot.speech.speak("Sorry, could not parse the given command")
                return False

            (sentence, semantics_str) = res
            print "Sentence: %s" % sentence
            print "Semantics: %s" % semantics_str

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # When using text-to-speech

        else:
            def prompt_once():
                robot.head.look_at_standing_person()
                robot.head.wait_for_motion_done()

                res = None
                while not res:
                    robot.speech.speak("What can I do for you?", block=True)
                    res = command_recognizer.recognize(robot)
                    if not res:
                        robot.speech.speak("Sorry, I could not understand", block=True)

                print "Sentence: %s" % res[0]
                print "Semantics: %s" % res[1]
                return res

            def ask_confirm():
                robot.speech.speak("You want me to %s" % sentence.replace(" your", " my").replace(" me", " you"), block=True)
                answer = robot.ears.recognize("(yes|no)", {})
                if not answer or answer.result != "yes":
                    return False
                else:
                    return True

            (sentence, semantics_str) = prompt_once()

            # confirm
            if not ask_confirm():
                # we heared the wrong thing
                (sentence, semantics_str) = prompt_once()

                if not ask_confirm():
                    # we heared the wrong thing twice
                    robot.speech.speak("Sorry")
                    return

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        semantics = yaml.load(semantics_str)

        self.command_data = semantics
        self.command_data["sentence"] = sentence

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

    def run(self, robot, sentence):

        command_recognizer = CommandRecognizer(os.path.dirname(sys.argv[0]) + "/grammar.fcfg", challenge_knowledge)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        action_functions = {}
        action_functions["navigate"] = self.navigate
        action_functions["find"] = self.find
        action_functions["answer-question"] = self.answer_question
        action_functions["pick-up"] = self.find_and_pick_up
        action_functions["bring"] = self.bring
        action_functions["say"] =  self.say

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Initialize

        robot.lights.set_color(0, 0, 1)  #be sure lights are blue

        robot.leftArm.reset()
        robot.leftArm.send_gripper_goal('close',0.0)
        robot.rightArm.reset()
        robot.rightArm.send_gripper_goal('close',0.0)
        robot.ed.reset()
        robot.torso.reset()
        robot.head.reset()

        if robot.robot_name == "amigo":

            # Wait for trigger to become True
            while self.wait_for_trigger and not rospy.is_shutdown():
                time.sleep(0.1)

            from robot_smach_states import WaitForDoorOpen
            wait_state = WaitForDoorOpen(robot=robot)
            wait_state.run(robot=robot, timeout=None)

            robot.base.set_initial_pose(-1, 0, 0)

            robot.base.force_drive(0.25, 0, 0, 5.0)    # x, y, z, time in seconds

            # s = StartChallengeRobust(robot, challenge_knowledge.starting_point)
            # s.execute()

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        done = False
        while not done and not rospy.is_shutdown():

            robot.head.reset()

            # Wait for trigger to become True
            while self.wait_for_trigger and not rospy.is_shutdown():
                time.sleep(0.1)

            if rospy.is_shutdown():
                return

            try:
                sentence = None
                if self.command_data:
                    sentence = self.command_data["sentence"]

                self.execute_command(robot, command_recognizer, action_functions, sentence)
                self.command_data = {}

                if robot.robot_name == "sergio":
                    self.wait_for_trigger = True

            except Exception as e:
                rospy.logerr("{0}".format(e.message))
                robot.speech.speak("I am truly sorry, but I messed up this assignment")

            self.command_data = {}

# ------------------------------------------------------------------------------------------------------------------------

def main():
    rospy.init_node("gpsr")

    parser = argparse.ArgumentParser()
    parser.add_argument('robot', help='Robot name')
    parser.add_argument('--forever', action='store_true', help='Turn on infinite loop')
    parser.add_argument('--skip', action='store_true', help='Skip enter/exit')
    parser.add_argument('sentence', nargs='*', help='Optional sentence')
    args = parser.parse_args()
    rospy.loginfo('args: %s', args)

    robot_name = args.robot
    run_forever = args.forever
    skip_init = args.skip
    sentence = " ".join([word for word in args.sentence if word[0] != '_'])

    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        print "unknown robot"
        return 1

    robot = Robot()

    # Sleep for 1 second to make sure everything is connected
    time.sleep(1)

    gpsr = GPSR(robot)
    gpsr.run(robot, sentence)

# ------------------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
