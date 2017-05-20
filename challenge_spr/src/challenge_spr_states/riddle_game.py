#!/usr/bin/python

import roslib;
import rospy
import smach
import sys

import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states.util.designators import Designator, EdEntityDesignator
from hmi import TimeoutException
from robocup_knowledge import load_knowledge

knowledge = load_knowledge('challenge_spr')
common_knowledge = load_knowledge('common')


def hear(robot, time_out):
    try:
        return robot.hmi.query('Question?', knowledge.grammar, 'T', timeout=time_out)
    except TimeoutException:
        return None


def answer(robot, res, crowd_data):
    if res:
        if 'actions' in res.semantics:
            for action in res.semantics['actions']:

                # Predefined questions
                if action['action'] == 'answer':
                    # if res.semantics['actions'] == 'answer':
                    answer = action['solution']#res.semantics['solution']
               
                # Counting people
                if action['action'] == 'count':

                    if action['entity'] == 'people':
                        answer = 'In the crowd are %d people' % crowd_data['crowd_size']

                    if action['entity'] == 'children':
                        answer = 'In the crowd are %d children' % crowd_data['children']

                    if action['entity'] == 'adults':
                        answer = 'In the crowd are %d adults' % crowd_data['adults']

                    if action['entity'] == 'elders':
                        answer = 'In the crowd are %d elders' % crowd_data['elders']

                    if action['entity'] == 'males':
                        answer = 'In the crowd are %d males' % crowd_data['males']

                    if action['entity'] == 'females':
                        answer = 'In the crowd are %d females' % crowd_data['females']

                    if action['entity'] == 'men':
                        answer = 'In the crowd are %d men' % crowd_data['men']

                    if action['entity'] == 'women':
                        answer = 'In the crowd are %d women' % crowd_data['women']

                    if action['entity'] == 'boys':
                        answer = 'In the crowd are %d boys' % crowd_data['boys']

                    if action['entity'] == 'girls':
                        answer = 'In the crowd are %d girls' % crowd_data['girls']

                # Location of placements or beacons
                if action['action'] == 'a_find':
                    entity = action['entity']
                    locations = [l for l in common_knowledge.locations if l['name'] == entity]
                    if len(locations) == 1:
                        loc = locations[0]['room']
                        answer = 'The %s can be found in the %s' % (entity, loc)
                    else:
                        answer = 'I dont know that object'

                # Count placements or beacons in the room
                if action['action'] == 'a_count':
                    entity = action['entity']
                    locations = [l for l in common_knowledge.locations if l['name'] == entity]
                    if len(locations) == 1:
                        loc = locations[0]['room']
                        if loc == action['location']:
                            answer = 'There is one %s in the %s' % (entity, loc)
                        else:
                            answer = 'There are no %s in the %s' % (entity, loc)
                    else:
                        answer = 'I dont know that object'

                # Find objects
                if action['action'] == 'o_find':
                    entity = action['entity']
                    locations = [l for l in common_knowledge.objects if l['name'] == entity]
                    if len(locations) == 1:
                        cat = locations[0]['category']
                        print cat
                        loc, area_name = common_knowledge.get_object_category_location(cat)
                        answer = 'You can find the %s %s %s' % (entity, area_name, loc)
                        
                    else:
                        answer = 'I dont know that object'

                # Find category
                if action['action'] == 'c_find':
                    entity = action['entity']
                    loc, area_name = common_knowledge.get_object_category_location(entity)
                    answer = 'You can find the %s on the %s' % (entity, loc)
    
                # Return objects category
                if action['action'] == 'return_category':
                    entity = action['entity']
                    locations = [l for l in common_knowledge.objects if l['name'] == entity]
                    if len(locations) == 1:
                        cat = locations[0]['category']
                        answer = 'The %s belongs to %s' % (entity, cat)
        
                    else:
                        answer = 'I dont know that object'

            rospy.loginfo("Question was: '%s'?"%res.sentence)
            robot.speech.speak("The answer is %s"%answer)
            return 'answered'
        else:
            robot.speech.speak("Sorry, I do not understand your question")
    else:
        pass

    return 'not_answered'


class HearQuestion(smach.State):
    def __init__(self, robot, time_out=rospy.Duration(15)):
        smach.State.__init__(self, outcomes=["answered", "not_answered"], input_keys=['crowd_data'])
        self.robot = robot
        self.time_out = time_out

    def execute(self, userdata):
        crowd_data = userdata.crowd_data

        self.robot.head.look_at_standing_person()

        res = hear(self.robot, time_out=self.time_out)

        return answer(self.robot, res, crowd_data)

        # Standalone testing -----------------------------------------------------------------

class TestRiddleGame(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        self.userdata.crowd_data = {
            "males": 1,
            "men": 2,
            "females": 3,
            "women": 4,
            "children": 5,
            "boys": 6,
            "girls": 7,
            "adults": 8,
            "elders": 9,
            "crowd_size": 10
        }

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'HEAR_QUESTION',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("HEAR_QUESTION",
                                   HearQuestion(robot),
                                   transitions={'answered': 'HEAR_QUESTION_2', 'not_answered': 'HEAR_QUESTION_2'},
                                   remapping={'crowd_data':'crowd_data'})

            smach.StateMachine.add("HEAR_QUESTION_2",
                                   HearQuestion(robot),
                                   transitions={'answered': 'Done', 'not_answered': 'Done'})

if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(TestRiddleGame, challenge_name="challenge_spr")
