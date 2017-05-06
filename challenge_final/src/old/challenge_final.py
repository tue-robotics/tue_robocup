#! /usr/bin/python

# ------------------------------------------------------------------------------------------------------------------------
# By Sjoerd van den Dries, 2016
# ------------------------------------------------------------------------------------------------------------------------

import os
import sys
import rospy
import argparse
import time
import subprocess
import rospkg

import std_msgs
import robot_smach_states

from robocup_knowledge import load_knowledge

import action_server.command_center as cs
from action_server.command_center import CommandCenter

import hmi_server
import json

import challenge_final.handover_amigo
import challenge_final.handover_sergio

rospack = rospkg.RosPack()
doorbell_path = os.path.join(
    rospack.get_path('challenge_final'), 'data', 'doorbell_short.wav')

# ------------------------------------------------------------------------------------------------------------------------


class ChallengeFinal:

    # ------------------------------------------------------------------------------------------------------------------------

    def __init__(self, robot, sentence=None):
        self.robot = robot
        self.robot.leftArm.reset()
        self.robot.rightArm.reset()
        self.robot.torso.reset()
        self.robot.head.cancel_goal()
        self._trigger_sub = rospy.Subscriber("/" + robot.robot_name + "/trigger",
                                             std_msgs.msg.String,
                                             self._trigger_callback, queue_size=1)

        other_robot = "sergio" if robot.robot_name == "amigo" else "amigo"

        self._trigger_pub = rospy.Publisher("/" + other_robot + "/trigger", std_msgs.msg.String, queue_size=1)

        self.do_listen_command = False
        self.sentence = sentence

        self.first_order = True

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

    def _ask_order_from_person(self, robot, world, message):
        robot.head.look_at_ground_in_front_of_robot(2)
        robot.head.wait_for_motion_done()

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Take order

        self.robot.speech.speak(message)

        bar_object_id = None

        while True:
            try:
                result = robot.hmi.query("What do you want?", "<choice>", {"choice":self.knowledge.object_names}, timeout=10)
            except hmi_server.api.TimeoutException:
                self.robot.speech.speak("Please let me know what you want")
            else:
                bar_object_id = result["choice"]
                break

        if not bar_object_id:
            return None

        self.robot.speech.speak("Ok, you want a {}".format(bar_object_id), block=False)
        return bar_object_id

    # ------------------------------------------------------------------------------------------------------------------------

    def take_order(self, robot, world, parameters):
        ret = subprocess.Popen(['aplay', doorbell_path])
        entity = cs.actions.resolve_entity_description(world, parameters["entity"])
        cs.actions.move_robot(robot, world, id=entity.id)

        ret_status = ret.poll()
        if ret_status == None:
            rospy.logwarn('killing the doorbell')
            ret.terminate()
            rospy.loginfo('killed the doorbell')
        elif ret_status == 0:
            rospy.loginfo('Doorbell was played')
        else:
            rospy.logerr('Doorbell file not found')

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Take order

        bar_object_id = self._ask_order_from_person(robot, world, "Hello! What can I get you?")
        if not bar_object_id:
            return

        if self.first_order:
            self.robot.speech.speak("I will get it! Oh wait, stupid me! I can't get it because I don't have any arms!")
            self.robot.speech.speak("If only I had a bartender friend to help me..")
            self.trigger_other_robot("come in")
            self.first_order = False

            while not rospy.is_shutdown():
                trigger = self.wait_for_trigger()
                if trigger == "continue":
                    break

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Check if item is available

        self.robot.speech.speak("Hey amigo, do we have a {}?".format(bar_object_id), block=True)

        self.trigger_other_robot("check {}".format(bar_object_id.replace("_", " ")))

        while not rospy.is_shutdown():
            trigger = self.wait_for_trigger()
            if trigger == "yes we have" or trigger == "no we do not have":
                break
            else:
                rospy.loginfo("I'm busy waiting, can't do anything else!")

        if trigger != "yes we have":
            bar_object_id = self._ask_order_from_person(robot, world, "I just heard the %s is not available, sorry. Can I get you anything else?" % bar_object_id)

            if not bar_object_id:
                return

        self.robot.speech.speak("We have it!", block=True)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Ask AMIGO to prepare drink and drive to the bar

        self.robot.speech.speak("I will ask my friend AMIGO to prepare it! In the meanwhile, I'll go to the bar!", block=False)

        self.trigger_other_robot("prepare {}".format(bar_object_id.replace("_", " ")))

        # Send trigger to AMIGO to get drink ready
        self.trigger_other_robot("prepare {}".format(bar_object_id.replace("_", " ")))

        # Drive to the kitchen
        # cs.actions.move_robot(robot, world, id=self.knowledge.bar_id)
        # cs.actions.move_robot(robot, world, id="table1")

        challenge_final.handover_sergio.move_sergio_to_pre_handover_pose(robot, self.knowledge.bar_id)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Wait until AMIGO has the object ready, and extract the position

        # pos = None

        # Wait for AMIGO's trigger that the entity is there

        x = None

        while not rospy.is_shutdown():
            trigger = self.wait_for_trigger()

            print "trigger = {}".format(trigger)

            if trigger and trigger.find("receive at") >= 0:
                args = trigger.split("at")
                if (len(args) > 1):
                    params = args[1].strip().split(" ")
                    x = float(params[0])
                    y = float(params[1])
                    theta = float(params[2])

                    print "POS is: {} {} {}".format(x, y, theta)
                break

            rospy.loginfo("I'm busy waiting, can't do anything else!")
            time.sleep(1)

        if not x:
            self.robot.speech.speak("AMIGO did not send the coordinates", block=True)
            return

        challenge_final.handover_sergio.move_sergio_to_handover_pose(robot, x, y, theta)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Tell AMIGO that we are there

        self.robot.speech.speak("Put it on my tray AMIGO!", block=True)
        self.trigger_other_robot("serve {}".format(bar_object_id.replace("_", " ")))

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Wait for AMIGO's trigger that the entity is there

        while not rospy.is_shutdown() and self.wait_for_trigger() != "bring it":
            rospy.loginfo("I'm busy waiting, can't do anything else!")
            time.sleep(1)

        self.robot.speech.speak("Yay! I've go the {}".format(bar_object_id), block=True)

        challenge_final.handover_sergio.move_sergio_back(robot)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Hand it over to the person

        # Drive back to the person ordering the drink
        cs.actions.move_robot(robot, world, id=entity.id)

        # Tell him to get it
        self.robot.speech.speak("Here you go friend! A lovely {} for you!".format(bar_object_id), block=True)

    # ------------------------------------------------------------------------------------------------------------------------

    def come_in(self, robot, world, parameters):
        robot.speech.speak("Knock, knock, will you let me in", block=False)
        wait_state = robot_smach_states.WaitForDoorOpen(robot=robot)
        wait_state.run(robot=robot, timeout=None)
        robot.speech.speak("Here I am, AMIGO the bartender to your service!", block=False)

        robot.base.set_initial_pose(0, 0, 0)
        # Wait 0.5 s just to be sure
        rospy.sleep(rospy.Duration(0.5))

        robot.base.force_drive(0.25, 0, 0, 5.0)    # x, y, z, time in seconds

        # Drive to the kitchen
        cs.actions.move_robot(robot, world, id=self.knowledge.bar_id)

        self.trigger_other_robot("continue")

    # ------------------------------------------------------------------------------------------------------------------------

    def check(self, robot, world, parameters):
        bar_object = cs.actions.resolve_entity_description(world, parameters["entity"])

        challenge_final.handover_amigo.amigo_reset_arm(robot)

        self.robot.speech.speak("Let me see...", block=True)

        if bar_object.id in self.knowledge.bar_objects:
            self.robot.speech.speak("Yes, we have a {}".format(bar_object.id), block=True)
            self.trigger_other_robot("yes we have")
        else:
            self.robot.speech.speak("Nope, I don't have a {}. But I do have tea, pringles and a sponge".format(bar_object.id), block=True)
            self.trigger_other_robot("no we do not have")

    # ------------------------------------------------------------------------------------------------------------------------

    def prepare(self, robot, world, parameters):
        # The object is on the bar
        parameters["entity"]["loc"] = self.knowledge.bar_id

        bar_object = cs.actions.resolve_entity_description(world, parameters["entity"])

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # Pick up the object

        print parameters

        cs.actions.find_and_pick_up(robot, world, parameters, pick_up=True)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        challenge_final.handover_amigo.amigo_navigate_to_handover_pose(robot)
        (res, pose) = challenge_final.handover_amigo.amigo_move_arm_to_place_position(robot)

        if pose:
            x = pose[0]
            y = pose[1]
            theta = pose[2]
            self.trigger_other_robot('receive at %f %f %f' % (x, y, theta))
        else:
            self.robot.speech.speak("I could not move my arm to the correct location!")

    # ------------------------------------------------------------------------------------------------------------------------

    def serve(self, robot, world, parameters):
        bar_object = cs.actions.resolve_entity_description(world, parameters["entity"])

        self.robot.speech.speak("Here you go sergio!", block=False)

        challenge_final.handover_amigo.amigo_place(robot)

        # # For now just open the gripper
        # robot.leftArm.send_gripper_goal("open")
        # robot.rightArm.send_gripper_goal("open")

        # Tell SERGIO to bring it!
        self.trigger_other_robot('bring it')
        time.sleep(5)
        challenge_final.handover_amigo.amigo_reset_arm(robot)

        # Drive to the kitchen
        cs.actions.move_robot(robot, world, id=self.knowledge.bar_id)

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
        self.command_center.register_action("check", self.check)
        self.command_center.register_action("come-in", self.come_in)

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

