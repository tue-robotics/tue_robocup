#!/usr/bin/python

import roslib
import rospy
import smach
import sys
import random

import robot_smach_states as states
from robot_smach_states.util.startup import startup
from hmi import TimeoutException
from robocup_knowledge import load_knowledge

knowledge = load_knowledge('challenge_spr')
common_knowledge = load_knowledge('common')

def hear(robot, time_out):
    '''
    Robots hears a question

    Output:
        question, if understood, else None
    '''
    try:
        return robot.hmi.query('Question?', knowledge.grammar, 'T', timeout=time_out)
    except TimeoutException:
        return None

def answer(robot, res, crowd_data):
    '''
    Robot answers the heard question

    Arguments:
        res: question or None
        crowd_data: data about the crowd

    Outputs:
        True: if successfully answered the question, else False
    '''
    if not res:
        return False

    if 'actions' not in res.semantics:
        robot.speech.speak("Your question was '%s' but I don't know the answer" % res.sentence)
        return False

    for action in res.semantics['actions']:
        try:
            if 'action' not in action:
                continue

            answer = "Sorry, I cannot %s. I don't know the answer.  " % str(action['action'])

            if action['action'] == 'random_gender':
                answer = random.choice(["A male", "A female"])

            # Predefined questions
            if action['action'] == 'answer':
                answer = action['solution']#res.semantics['solution']

            # Counting people
            if action['action'] == 'count':
                result = answer_count_people(action, crowd_data)
                if result is not None:
                    answer = result

            # Location of placements or beacons
            if action['action'] == 'find_placement':
                entity = action['entity']
                locations = [loc for loc in common_knowledge.locations if loc['name'] == entity]
                if len(locations) == 1:
                    room = locations[0]['room']
                    answer = 'The %s can be found in the %s' % (entity, room)
                else:
                    answer = 'I dont know that object'

            # Count placements or beacons in the room
            if action['action'] == 'count_placement':
                entity = action['entity']
                locations = [loc for loc in common_knowledge.locations if loc['name'] == entity]
                if len(locations) == 1:
                    room = locations[0]['room']
                    if loc == action['location']:
                        answer = 'There is one %s in the %s' % (entity, room)
                    else:
                        answer = 'There are no %s in the %s' % (entity, room)
                else:
                    answer = 'I should count but I dont know that object %s' % entity

            # Find objects
            if action['action'] == 'find_object':
                entity = action['entity']
                objects = [obj for obj in common_knowledge.objects if obj['name'] == entity]
                if len(objects) == 1:
                    cat = objects[0]['category']
                    print cat
                    loc, area_name = common_knowledge.get_object_category_location(cat)
                    answer = 'You can find the %s %s %s' % (entity, area_name, loc)
                else:
                    answer = 'I should find %s but I dont know %s' % entity

            # Find category
            if action['action'] == 'find_category':
                entity = action['entity']
                loc, area_name = common_knowledge.get_object_category_location(entity)
                answer = 'You can find the %s on the %s' % (entity, loc)

            # Return objects category
            if action['action'] == 'return_category':
                entity = action['entity']
                objects = [obj for obj in common_knowledge.objects if obj['name'] == entity]
                if len(objects) == 1:
                    cat = objects[0]['category']
                    answer = 'The %s belongs to %s' % (entity, cat)
                else:
                    answer = 'I should name a category but I dont know %s' % entity

            if action['action'] == "c_count":
                answer = "4 people are wearing %s" % str(action['entity'])

            # Return objects color
            if action['action'] == 'return_color':
                entity = action['entity']
                objects = [obj for obj in common_knowledge.objects if obj['name'] == entity]
                if len(objects) == 1:
                    col = objects[0]['color']
                    answer = 'The color of %s is %s' % (entity, col)
                else:
                    answer = 'I should name the color but I dont know %s' % entity

            # Compare objects categories
            if action['action'] == 'compare_category':
                entity_a = action['entity_a']
                entity_b = action['entity_b']
                objects_a = [obj for obj in common_knowledge.objects if obj['name'] == entity_a]
                objects_b = [obj for obj in common_knowledge.objects if obj['name'] == entity_b]
                if len(locations_a) == 1 and len(locations_b) == 1:
                    cat_a = objects_a[0]['category']
                    cat_b = objects_b[0]['category']
                    if cat_a == cat_b:
                        answer = 'Both objects belong to the same category %s' % cat_a
                    else:
                        answer = 'These objects belong to different categories'
                else:
                    answer = 'I dont know these objects'

            if action['action'] == 'compare':
                answer = action['entity_a']

            # Count how many objects belong to category
            if action['action'] == 'count_object':
                entity = action['entity']
                objects = [obj for obj in common_knowledge.objects if obj['category'] == entity]
                if len(objects) > 0:
                    count = len(objects)
                    answer = 'The number of objects in category %s is %i' % (entity, count)

        except Exception as e:
            rospy.logerr(e)
            robot.speech.speak("Whoops")

    rospy.loginfo("Question was: '%s'?" % res.sentence)
    robot.speech.speak("The answer is %s" % answer)
    return True

def answer_count_people(action, crowd_data):
    crowd_properties = [('people', 'crowd_size'),
                        ('children', 'children'),
                        ('adults', 'adults'),
                        ('elders', 'elders'),
                        ('males', 'males'),
                        ('females', 'females'),
                        ('men', 'men'),
                        ('women', 'women'),
                        ('boys', 'boys'),
                        ('girls', 'girls')]

    for property_name, property_value in crowd_properties:
        if action['entity'] == property_name:
            return 'In the crowd are %d %s' % (crowd_data[property_value], property_name)
    return None

class HearAndAnswerQuestions(smach.State):
    '''
    Robot hears and answers questions from the riddle game in SPR challenge.

    Variables:
        num_questions: number of questions to be heard and answered

    Outputs:
        done: answered all questions
    '''
    def __init__(self, robot, num_questions=1, time_out=15.0):
        smach.State.__init__(self, outcomes=["done"], input_keys=['crowd_data'])
        self.robot = robot
        self.num_questions = num_questions
        self.time_out = time_out

    def execute(self, userdata):
        crowd_data = userdata.crowd_data

        self.robot.head.look_at_standing_person()


        for _ in xrange(self.num_questions):

            res = hear(self.robot, time_out=self.time_out)
            
            if not answer(self.robot, res, crowd_data):
                robot.speech.speak("Please ask next question!")

        return "done"

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
                                   transitions={'initialized': 'RIDDLE_GAME',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("RIDDLE_GAME",
                                   HearAndAnswerQuestions(robot, num_questions=3),
                                   transitions={'done': 'Done'},
                                   remapping={'crowd_data':'crowd_data'})

if __name__ == "__main__":
    rospy.init_node('speech_person_recognition_exec')

    startup(TestRiddleGame, challenge_name="challenge_spr")
