#! /usr/bin/env python

from __future__ import print_function

import random

import rospy
import smach

import robot_smach_states as states
from cb_planner_msgs_srvs.msg import PositionConstraint, OrientationConstraint

from robot_smach_states.util.startup import startup
import std_msgs.msg

from robot_smach_states.designators.designator import Designator
from ed_msgs.srv import SimpleQuery


class RandomNavDesignator(EdEntityDesignator):

    def __init__(self, robot):
        super(RandomNavDesignator, self).__init__()
        self._robot = robot
        self.entity_id = None
        self.last_entity_id = None

        # Publisher and subriber
        self.locations_pub = rospy.Publisher("/locations_list", std_msgs.msg.String, queue_size=10)
        rospy.Subscriber("/nav_goal", std_msgs.msg.String, self.goalCallback)
        self.stop_pub = rospy.Publisher("/nav_test_control", std_msgs.msg.String)

    def resolve(self):
        if self.entity_id:
            eid = self.entity_id
            rospy.logwarn("Resolved ID = {0}".format(eid))
            self.last_entity_id = self.entity_id
            self.entity_id = None

            if eid == "stairway":
                msg = std_msgs.msg.String("stop")
                self.stop_pub.publish(msg)

            ed_service = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)
            entities = ed_service(eid)
            return entities[0]

    def getRandomGoal(self):
        # If a goal is already defined by the used, return
        if self.entity_id is not None:
            return self.entity_id

        # Get all entities
        entities = self._robot.ed.get_entities(type="", parse=False)

        # Temp: only pick close targets
        #if entities:
        #    entities = [entity for entity in entities if (
        #    entity.id == "elevator"
        #    or entity.id == "lecture_room_1"
        #    or entity.id == "lecture_room_2"
        #    or entity.id == "copier"
        #    or entity.id == "stairway"
        #    )]

        # If entities found: only take entities with convex hulls, that have a type and are not floor...
        if entities:
            entities = [entity for entity in entities if ( entity.convex_hull
                and "library" in entity.type
                and not entity.id == "floor"
                and not entity.id == "walls"
                and not entity.id == "desks-support"
                and not entity.id == "desks-top"
                )]
        else:
            raise Exception("No entities in this model")

        # If still entities
        if entities:

            # Publish a comma separated list with possible locations
            msg = std_msgs.msg.String()

            for entity in entities:

                # If necessary: add comma
                if msg.data != "":
                    msg.data += ","

                # Add entity
                msg.data += entity.id

                print("ID = {0}, type = {1}".format(entity.id, entity.type))

            self.locations_pub.publish(msg)

            # Select random object
            entity_id = None
            while entity_id is None:
                entity = random.choice(entities)
                if entity.id != self.last_entity_id:
                    entity_id = entity.id

            rospy.loginfo("Entity:\n\tid = {0},\n\ttype = {1}".format(entity.id, entity.type))

            self.entity_id = entity_id
            return self.entity_id
        else:
            raise Exception("No entities with convex hulls")

    def goalCallback(self, msg):
        # Check if already present
        if msg.data == self.entity_id:
            return
        elif msg.data == "":
            return
        else:
            self.entity_id = msg.data

        rospy.loginfo("Next goal: {0}".format(self.entity_id))
        self._robot.speech.speak(random.choice(["I am asked to go to the {0}".format(self.entity_id),
                                                "My next goal will be the {0}".format(self.entity_id),
                                                "My next target will be the {0}".format(self.entity_id),
                                                "Next, I will go to the {0}".format(self.entity_id),
                                                "After this, I will go to the {0}".format(self.entity_id),
                                                "I will soon go to the {0}".format(self.entity_id),
                                                "I will go to the {0} after I have reached this goal".format(self.entity_id),
                                                "I will show you the {0} in a minute".format(self.entity_id),
                                                "I will show you the {0} in a second".format(self.entity_id),
                                                ]))
        return


class SelectAction(smach.State):
    def __init__(self, outcomes=['continue', 'pause', 'stop']):
        self.outcomes= outcomes
        smach.State.__init__(self, outcomes=self.outcomes)
        self.outcome = 'continue'

        self.rate = float(rospy.get_param('~rate', '1.0'))
        topic = rospy.get_param('~topic', '/nav_test_control')

        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

        rospy.loginfo("Use 'navc' to continue, 'navp' to pause and 'navs' to stop this node")

    def execute(self, userdata=None):
        if self.outcome == 'pause':
            rospy.sleep(rospy.Duration(1/self.rate))
        return self.outcome

    def callback(self, msg):
        if msg.data in self.outcomes:
            self.outcome = msg.data
        else:
            rospy.logwarn("{0} is not a possible outcome for SelectAction, possibilities are{1}".format(msg.data, self.outcomes))


class RandomNav(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        self.robot = robot

        self.position_constraint = PositionConstraint()
        self.orientation_constraint = OrientationConstraint()
        self.position_constraint.constraint = "x^2+y^2 < 1"

        self.requested_location = None
        rospy.Subscriber("/location_request", std_msgs.msg.String, self.requestedLocationcallback)

        self.random_nav_designator = RandomNavDesignator(self.robot)

        with self:

            smach.StateMachine.add( "WAIT_A_SEC",
                                    states.WaitTime(robot, waittime=1.0),
                                    transitions={'waited'   :"SELECT_ACTION",
                                                 'preempted':"Aborted"})

            smach.StateMachine.add( "SELECT_ACTION",
                                    SelectAction(),
                                    transitions={   'continue'  : "DETERMINE_TARGET",
                                                    'pause'     : "SELECT_ACTION",
                                                    'stop'      : "SAY_DONE"})

            @smach.cb_interface(outcomes=['target_determined', 'no_targets_available'],
                                input_keys=[],
                                output_keys=[])
            def determine_target(userdata, designator):

                entity_id = designator.getRandomGoal()

                sentences = [   "Lets go look at the %s",
                                "Lets have a look at the %s",
                                "Lets go to the %s",
                                "Lets move to the %s",
                                "I will go to the %s",
                                "I will now move to the %s",
                                "I will now drive to the %s",
                                "I will look the %s",
                                "The %s will be my next location",
                                "The %s it is", "New goal, the %s",
                                "Going to look at the %s",
                                "Moving to the %s",
                                "Driving to the %s",
                                "On to the %s",
                                "On the move to the %s",
                                "Going to the %s"]
                robot.speech.speak(random.choice(sentences)%entity_id, block=False)

                return 'target_determined'

            smach.StateMachine.add('DETERMINE_TARGET', smach.CBState(determine_target,
                                    cb_kwargs={'designator': self.random_nav_designator}),
                                    transitions={   'target_determined':'DRIVE',
                                                    'no_targets_available':'SELECT_ACTION'})

            smach.StateMachine.add( 'DRIVE',
                                    states.NavigateToObserve(robot, self.random_nav_designator),
                                    transitions={   "arrived":"SAY_SUCCEEDED",
                                                    "unreachable":'SAY_UNREACHABLE',
                                                    "goal_not_defined":'SELECT_ACTION'})

            smach.StateMachine.add("SAY_SUCCEEDED",
                                    states.Say(robot, [ "I am here",
                                                        "Goal succeeded",
                                                        "Another goal succeeded",
                                                        "Goal reached",
                                                        "Another goal reached",
                                                        "Target reached",
                                                        "Another target reached",
                                                        "Destination reached",
                                                        "Another destination reached",
                                                        "I have arrived",
                                                        "I have arrived at my goal",
                                                        "I have arrived at my target",
                                                        "I have arrived at my destination",
                                                        "I am at my goal",
                                                        "I am at my target",
                                                        "I am at my destination",
                                                        "Here I am",]),
                                    transitions={   'spoken':'SELECT_ACTION'})

            smach.StateMachine.add("SAY_UNREACHABLE",
                                    states.Say(robot, [ "I can't find a way to my goal, better try something else",
                                                        "This goal is unreachable, I better find somewhere else to go",
                                                        "I am having a hard time getting there so I will look for a new target"]),
                                    transitions={   'spoken':'SELECT_ACTION'})

            smach.StateMachine.add("SAY_DONE",
                                    states.Say(robot, [ "That's all folks", "I'll stay here for a while", "Goodbye"]),
                                    transitions={   'spoken':'Done'})

    def requestedLocationcallback(self, msg):
        self.requested_location = msg.data
        self.robot.speech.speak("I got a request to go to location %"%self.requested_location)
        rospy.loginfo("Requested location is %s"%self.requested_location)


if __name__ == "__main__":
    rospy.init_node('random_nav_exec')

    startup(RandomNav)
