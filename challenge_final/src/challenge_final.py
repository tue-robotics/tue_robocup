#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Sjoerd van den Dries, 2016
# ------------------------------------------------------------------------------------------------------------------------

import os
import sys
import rospy
import argparse
import time

import std_msgs

import robot_smach_states
from robot_smach_states.navigation import NavigateToObserve, NavigateToWaypoint, NavigateToSymbolic

from robocup_knowledge import load_knowledge

import action_server
import action_server.command_center as cs 
from action_server.command_center import CommandCenter

import hmi_server
import json

# ------------------------------------------------------------------------------------------------------------------------

class ChallengeFinal:

    # ------------------------------------------------------------------------------------------------------------------------

    def __init__(self, robot, sentence=None):
        self.robot = robot
        self.robot.leftArm.reset()
        self.robot.rightArm.reset()
        self.robot.torso.reset()
        self.robot.head.cancel_goal()
        self._trigger_sub = rospy.Subscriber("/" + robot.robot_name + "/trigger", std_msgs.msg.String, self._trigger_callback, queue_size=1)

        other_robot = "sergio" if robot.robot_name == "amigo" else "amigo"

        self._trigger_pub = rospy.Publisher("/" + other_robot + "/trigger", std_msgs.msg.String)

        self.do_listen_command = False
        self.sentence = sentence

    # ------------------------------------------------------------------------------------------------------------------------

    def _trigger_callback(self, msg):
        print "trigger"
        if msg.data == "listen" or msg.data == "gpsr":
            print "listen"
            self.do_listen_command = True
            self.sentence = None
        else:
            self.do_listen_command = False
            self.sentence = msg.data

    # ------------------------------------------------------------------------------------------------------------------------

    def trigger_other_robot(self, message):
        rospy.loginfo("Triggering other robot with message: {}".format(message))
        self._trigger_pub.publish(std_msgs.msg.String(message))

    # ------------------------------------------------------------------------------------------------------------------------

    def wait_for_trigger(self):
        while not rospy.is_shutdown() and not self.sentence:
            time.sleep(0.1)
        trigger = self.sentence
        self.sentence = None
        return trigger

    # ------------------------------------------------------------------------------------------------------------------------

    def take_order(self, robot, world, parameters):
        entity = cs.actions.resolve_entity_description(world, parameters["entity"])
        cs.actions.move_robot(robot, world, id=entity.id)

        robot.head.look_at_ground_in_front_of_robot(2)
        robot.head.wait_for_motion_done()

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        # Take order

        self.robot.speech.speak("Hello! What can I get you?")

        bar_object_id = None

        while True:            
            try:
                result = robot.hmi.query("What do you want?", "<choice>", {"choice":self.knowledge.bar_objects}, timeout=10)
            except hmi_server.api.TimeoutException:
                self.robot.speech.speak("Please let me know what you want")
            else:
                bar_object_id = result["choice"]
                break

        if not bar_object_id:
            return

        self.robot.speech.speak("Ok, you want a {}".format(bar_object_id), block=False)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        # Ask AMIGO to prepare drink and drive to the bar

        # Optional: check with AMIGO if this bar object exists

        # Send trigger to AMIGO to get drink ready
        self.robot.speech.speak("I will ask my friend AMIGO to prepare it! In the meanwhile, I'll go to the bar!", block=False)
        self.trigger_other_robot("prepare {}".format(bar_object_id))

        # Drive to the kitchen
        cs.actions.move_robot(robot, world, id=self.knowledge.bar_id)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        # Wait until AMIGO has the object ready, and extract the position

        # pos = None

        print "GOT HERE!"

        # Wait for AMIGO's trigger that the entity is there
        while not rospy.is_shutdown():
            trigger = self.wait_for_trigger()

            print "trigger = {}".format(trigger)

            if trigger and trigger.find("receive at") >= 0:
                args = trigger.split(":")
                if (len(args) > 1):
                    json_coordinate = args[1]
                    pos_dict = json.loads(json_coordinate)
                    pos = (pos_dict["x"], pos_dict["y"])
                break

            rospy.loginfo("I'm busy waiting, can't do anything else!")  
            time.sleep(1)            

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        # Tell AMIGO that we are there

        self.robot.speech.speak("Put it on my tray AMIGO!", block=True)
        self.trigger_other_robot("serve {}".format(bar_object_id))

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -         
        # Wait for AMIGO's trigger that the entity is there

        while not rospy.is_shutdown() and self.wait_for_trigger() != "bring it":
            rospy.loginfo("I'm busy waiting, can't do anything else!")  
            time.sleep(1)            

        self.robot.speech.speak("Yay! I've go the {}".format(bar_object_id), block=True)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -         
        # Hand it over to the person

        # Drive back to the person ordering the drink
        cs.actions.move_robot(robot, world, id=entity.id)

        # Tell him to get it
        self.robot.speech.speak("Here you go friend! A lovely {} for you!".format(bar_object_id), block=True)

    # ------------------------------------------------------------------------------------------------------------------------

    def prepare(self, robot, world, parameters):
        # The object is on the bar
        parameters["entity"]["loc"] = self.knowledge.bar_id

        bar_object = cs.actions.resolve_entity_description(world, parameters["entity"])

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -         
        # Pick up the object

        print parameters

        cs.actions.find_and_pick_up(robot, world, parameters, pick_up=True)      

        # TODO: move arm to location

        self.robot.speech.speak("I should move my arm now to a place location, but I can't do that yet!".format(bar_object.id), block=True)
        
        # TODO: get current gripper location

        x = 1.23
        y = 2.45
        self.trigger_other_robot('receive at: { "x": %f, "y": %f }' % (x, y))

    # ------------------------------------------------------------------------------------------------------------------------

    def serve(self, robot, world, parameters):
        bar_object = cs.actions.resolve_entity_description(world, parameters["entity"])
        
        self.robot.speech.speak("Here you go sergio!", block=False)

        # For now just open the gripper
        robot.leftArm.send_gripper_goal("open")
        robot.rightArm.send_gripper_goal("open")

        # TODO

        # Tell SERGIO to bring it!
        self.trigger_other_robot('bring it')        

    # ------------------------------------------------------------------------------------------------------------------------
        
    def run(self):

        self.command_center = CommandCenter(self.robot)

        self.knowledge = load_knowledge('challenge_final')

        self.command_center.set_grammar(os.path.dirname(sys.argv[0]) + "/grammar.fcfg", self.knowledge)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        print "GOT HERE!"
        self.command_center.register_action("take-order", self.take_order)
        self.command_center.register_action("prepare", self.prepare)
        self.command_center.register_action("serve", self.serve)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        sentences = [
                "What can I do for you?",
                "Can I do something?"
                ]

        rospy.loginfo("Ready, waiting for trigger!")

        while not rospy.is_shutdown():

            command_semantics = None

            if self.sentence:
                command_semantics = self.command_center.parse_command(self.sentence)
                if not command_semantics:
                    self.robot.speech.speak("I cannot parse \"{}\"".format(self.sentence))
                self.sentence = None

            elif self.do_listen_command:
                res = self.command_center.request_command(ask_confirmation=True,
                    ask_missing_info=False, sentences=sentences, n_tries=2)

                if not res:
                    self.robot.speech.speak("I did not understand")
                else:
                    (command_words, command_semantics) = res

                self.do_listen_command = False
            else:
                time.sleep(0.1)

            if command_semantics:
                print "Command semantics: {}".format(command_semantics)
                self.command_center.execute_command(command_semantics)
        
# ------------------------------------------------------------------------------------------------------------------------

def main():
    rospy.init_node("challenge_final")

    parser = argparse.ArgumentParser()
    parser.add_argument('robot', help='Robot name')
    parser.add_argument('sentence', nargs='*', help='Optional sentence')
    args = parser.parse_args()
    rospy.loginfo('args: %s', args)

    sentence = " ".join([word for word in args.sentence if word[0] != '_'])

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    if args.robot == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif args.robot == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        raise ValueError('unknown robot')

    robot = Robot()
    
    # Sleep for 1 second to make sure everything is connected
    time.sleep(1)    

    challenge = ChallengeFinal(robot, sentence)
    challenge.run()

if __name__ == "__main__":
    sys.exit(main())
    
