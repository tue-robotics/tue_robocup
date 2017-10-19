#! /usr/bin/env python
import rospy

import smach

import robot_smach_states as states
from robot_smach_states.util.startup import startup
import random
import std_msgs.msg

from part1 import TurnAround
# from look_at_person import LookAtPerson
from flash_lights import FlashLights
from walk_like_an_egyptian import WalkLikeAnEgyptian
from gangnam_style import GangNamStyle
# from boo import Boo
from make_jokes import MakeJokes
# from pickuplines import Pickup
from sounds import R2D2, Toeter
from macarana import Macarena
from hoofd_schouders_knie_teen import HoofdSchouderKnieTeen

# from demo_executioner import wave_lights #amigo_demo package is not using the recommended package layout with amigo_demo/src/amigo-demo

# list of available demo challanges:
colormap = {    "SAY_HI":(1,0,1),
                #"WAVE_LIGHTS":(1,1,1), #All colors together 
                "WALK_EGYPTIAN":(1,1,0), #Pyramids are yellow :-S?
                "R2D2":(0,0,1), #R2D2 is partly blue, so Amigo will as well
                "TOETER":(0,1,0), 
                "MACARENA":(0.99, 0.96, 0.90), #Orange
                "GANGNAM":(1,0,0), 
                "HOOFD_SCHOUDERS_KNIE_TEEN":(0,1,1)}
demos = list(colormap.keys())

description_map = { "SAY_HI":"I'll say hi when i'm done",
                    #"WAVE_LIGHTS":"Wave and blink is coming up",
                    "WALK_EGYPTIAN":"I'll walk like an egyptian",
                    "R2D2":"R2D2 coming up",
                    "TOETER":"I'll honk next",
                    "MACARENA":"I'll do the macarena for you!",
                    "GANGNAM":"Gangnam style is next",
                    "HOOFD_SCHOUDERS_KNIE_TEEN":"Next up: a children's dance"}


# Create dictionary lists
random_transitions = {}
for demo in demos:
    random_transitions[demo] = demo
qr_transitions = random_transitions.copy()
qr_transitions["empty"] = "SELECT_RANDOM"

def randomize_list(l):
    """Generate a semi-random version of the list l. Items are not repeated until the whole list has been outputted once"""
    while True:
        shuffled = list(l)
        random.shuffle(shuffled)
        while shuffled: #Checks whether its not empty
            yield shuffled.pop()


class RandomOutcome(smach.State):
    """Of the state's registered outcomes, just select a random one"""

    def __init__(self, robot, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        self.previous_outcome = None
        self.randomizer = randomize_list(outcomes)

    def execute(self, userdata=None):
        """Randomly selects one of its registered outcomes. It excludes the previous outcome so it doesn't do the same thing twice"""
        return self.randomizer.next()
        
class CheckQRMarker(smach.State):

    def __init__(self, robot, rate=None, checks=None):
        smach.State.__init__(self, 
                             outcomes=demos+["empty"])

        # Get the ~private namespace parameters from command line or launch file.
        self.rate = float(rospy.get_param('~rate', '10')) if not rate else rate
        topic     = rospy.get_param('~topic', 'qr_marker')
        
        self.demo = None
        self.robot = robot
                
        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

        rospy.loginfo('topic: /%s', topic)
        rospy.loginfo('rate:  %d Hz', self.rate)

    def execute(self, userdata):
        counter = 0
        while (counter < self.rate and not self.demo and not rospy.is_shutdown()):
            rospy.sleep(rospy.Duration(0.1))
            counter += 1
        
        if self.demo:
            demo = self.demo
            self.demo = None
            rospy.loginfo("Demo = {0}".format(demo))
            try:
                self.robot.lights.set_color(*colormap[demo])
            except KeyError:
                pass
            return demo
        else:
            return "empty"

    def callback(self, str_msg):
        rospy.loginfo("Received new demo request: {0}".format(str_msg.data))
        # ToDo: turn into capitals (if necessary)
        if (str_msg.data in demos):
            if self.demo != str_msg.data:
                #Apparently, the new QR-code is something different, so we'll do that
                self.robot.speech.speak(description_map[str_msg.data])
            self.demo = str_msg.data
            self.robot.lights.set_color(0,1,0)
        else:
            rospy.logwarn("Demo {0} is not in the list".format(str_msg.data))
            self.demo = None
        rospy.loginfo("Next demo = {0}".format(self.demo))
        self.robot.lights.set_color(0,0,1)

class RoboZooSimple(smach.StateMachine):
    """Calls each part in turn and makes a random switch between parts, i.e smach State(Machine)s.

    ADDING A NEW PART:
    - Create a file with a new thing to do for the challenge, like waving or making a joke.
        This file must contain one main state(machine).
    - Import the file and the stae(machine) from it.
    - Add a new state to the machine below. All of its transitions should make it go back to RESET_ALL
    - Edit the SELECT_RANDOM state to have a new transition to the newly added state.
        The list after 'robot' in its constructor is a list of the possible outcomes and should match with the possible outcomes in its transitions.
        RandomOutcome takes a random item from this list as an outcome.

    It could be handy 

    IDEAS:
        - DONE: Reset everything before SELECT_RANDOM
        - DONE: Walk like an egyptian (with arms and music)
        - DONE: Flash lights
        - DONE: wave (smile and wave boys)
        - DONE: Say some pickup lines when a face is detected (we can't select on gender just yet)
        - DONE: Stare into the distance and wait for a face to appear. Then say "boo!" :-)
        - DONE: Macarena move
        - DONE: Oppa Gangnam style
        - TEST: Look at person and say something funny
        - Act like a monkey (with arms and sound)
        - Use AR markers to select actions
        - Photo pose with speech
        - Play the tune from jaws when you are not looking and stop when you are looking.
        - Do movie-quotes and sounds:
            - "I'll be back" from Terminator
            - Roger Roger from the star wars battle droids
            - Play the Mos Eisly pub music
        - When Amigo recognizes a face, say 'These aren't the human I am looking for' and do the jedi arm wiggle
        - Wait for 'stop' to be said and say "Hammertime"
        - Do the Robot-dance (let the arms swing like is dead)

    """
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        self.robot = robot
        with self:
            
            smach.StateMachine.add( "RESET_ALL",
                                    states.ResetArmsTorsoHead(robot, timeout=5.0),
                                    transitions={"done":"WAIT_A_SEC"})

            smach.StateMachine.add( "WAIT_A_SEC", 
                                    states.WaitTime(robot, waittime=1),
                                    transitions={'waited'   :"SAY_SHOW_QR_MARKER",
                                                 'preempted':"Aborted"})

            smach.StateMachine.add( "SAY_SHOW_QR_MARKER",
                                    states.Say(robot, [ "If you show me a QR marker, I will dance with you! ", 
                                                        "Show me one of those paddles with a QR code and I'll do your bidding.",
                                                        "Please show me a QR marker",
                                                        "Show me one of those paddles, please."]),
                                    transitions={"spoken":"CHECK_QR_MARKER"})
                                                 
            smach.StateMachine.add( "CHECK_QR_MARKER",
                                     CheckQRMarker(robot, 100),
                                     transitions=qr_transitions)

            smach.StateMachine.add( "SELECT_RANDOM",
                                    RandomOutcome(robot, demos),
                                    transitions= random_transitions)

            smach.StateMachine.add( "SAY_HI",
                                    states.Say(robot, ["Hello, , , My name is AMIGO. . ., I am the care robot of Eindhoven university of technology. Have a nice day!", 
						       "Hi There! , . What a beautiful day it is. I am happy to show my autonomous care robot skills to the people of Magdenburg!"]),
                                    transitions={"spoken":"RESET_ALL"})

            smach.StateMachine.add( "MAKE_JOKES",
                                    MakeJokes(robot),
                                    transitions={"Done":"RESET_ALL"})

            smach.StateMachine.add( "TURN_AROUND",
                                    TurnAround(robot),
                                    transitions={"Done":"RESET_ALL", "Aborted":"RESET_ALL", "Failed":"RESET_ALL"})

            # smach.StateMachine.add( "LOOK_AT_PERSON",
            #                         LookAtPerson(robot),
            #                         transitions={"Done":"RESET_ALL", "Aborted":"RESET_ALL", "Failed":"RESET_ALL"})
            
            # @smach.cb_interface(outcomes=['done'])
            # def wave_lights_wrapped(*args, **kwargs):
            #     wave_lights(robot)
            #     return 'done'
            # smach.StateMachine.add( "WAVE_LIGHTS",
            #                         smach.CBState(wave_lights_wrapped),
            #                         transitions={"done":"RESET_ALL"})


            smach.StateMachine.add( "FLASH_LIGHTS",
                                    FlashLights(robot),
                                    transitions={"Done":"RESET_ALL"})
            
            smach.StateMachine.add( "WALK_EGYPTIAN",
                                    WalkLikeAnEgyptian(robot),
                                    transitions={"Done":"RESET_ALL"})
            
            # smach.StateMachine.add( "BOO",
            #                         Boo(robot),
            #                         transitions={"Done":"RESET_ALL", "Aborted":"RESET_ALL", "Failed":"RESET_ALL"})

            # smach.StateMachine.add( "PICKUP_LINES",
            #                         Pickup(robot),
            #                         transitions={"Done":"RESET_ALL"})
            
            smach.StateMachine.add( "R2D2",
                                    R2D2(robot),
                                    transitions={"Done":"RESET_ALL"})
            
            smach.StateMachine.add( "TOETER",
                                    Toeter(robot),
                                    transitions={"Done":"RESET_ALL"})
            
            smach.StateMachine.add( "MACARENA",
                                    Macarena(robot),
                                    transitions={"Done":"RESET_ALL"})
            
            smach.StateMachine.add( "GANGNAM",
                                    GangNamStyle(robot),
                                    transitions={"Done":"RESET_ALL"})
            
            smach.StateMachine.add( "HOOFD_SCHOUDERS_KNIE_TEEN",
                                    HoofdSchouderKnieTeen(robot),
                                    transitions={"Done":"RESET_ALL"})
if __name__ == "__main__":
    rospy.init_node("challenge_robo_zoo_exec")
    
    rospy.loginfo("QR transitions: {0}".format(qr_transitions))

    startup(RoboZooSimple)
