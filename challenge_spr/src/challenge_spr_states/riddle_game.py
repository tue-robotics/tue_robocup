#!/usr/bin/python

# System
import random
import traceback

# ROS
import rospy
import smach
import sys

# TU/e Robotics
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from hmi import TimeoutException
from robocup_knowledge import load_knowledge

knowledge = load_knowledge('challenge_spr')
common_knowledge = load_knowledge('common')

##############################################################################
#
# Default parameters:
#
##############################################################################

DEFAULT_HEAR_TIME = 20.0

##############################################################################
#
# Main class:
#
##############################################################################


class HearAndAnswerQuestions(smach.State):
    """
    Robot hears and answers questions from the riddle game in SPR challenge.

    Variables:
        num_questions: number of questions to be heard and answered

    Outputs:
        done: answered all questions
    """
    def __init__(self, robot, num_questions=1, hear_time=DEFAULT_HEAR_TIME):
        smach.State.__init__(self, outcomes=["done"], input_keys=['crowd_data'])
        self.robot = robot
        self.num_questions = num_questions
        self.hear_time = hear_time

    def execute(self, userdata):
        crowd_data = userdata.crowd_data

        self.robot.head.look_at_standing_person()

        for _ in xrange(self.num_questions):

            res = hear(self.robot, hear_time=self.hear_time)

            if not answer(self.robot, res, crowd_data):
                self.robot.speech.speak("Please ask me the next question!")

        return "done"

##############################################################################
#
# Functions:
#
##############################################################################


def hear(robot, hear_time):
    """
    Robots hears a question

    Output:
        question, if understood, else None
    """
    try:
        return robot.hmi.query('Question?', knowledge.grammar, 'T', timeout=hear_time)
    except TimeoutException:
        return None


def answer(robot, res, crowd_data):
    """
    Robot answers the heard question

    Arguments:
        robot: robot api object
        res: question or None
        crowd_data: data about the crowd

    Outputs:
        True: if successfully answered the question, else False
    """
    if not res:
        return False

    if 'actions' not in res.semantics:
        robot.speech.speak("Your question was '%s' but I don't know the answer" % res.sentence)
        return False

    # Pass on the provided crowd_data to the real answering function.
    def answer_crowd_questions(action):
        return answer_count_people(action, crowd_data)

    assignments = {'random_gender':     answer_random_gender,
                   'answer':            answer_predefined_questions,
                   'count':             answer_crowd_questions,
                   'find_placement':    answer_placement_location,
                   'count_placement':   answer_count_placement,
                   'find_object':       answer_find_objects,
                   'find_category':     answer_find_category,
                   'return_category':   answer_object_category,
                   'return_color':      answer_object_color,
                   'compare_sizes':     answer_compare_objects_sizes,
                   'compare_weight':    answer_compare_objects_weight,
                   'compare_category':  answer_compare_objects_categories,
                   'count_object_cat':  answer_count_objects_in_category,
                   }

    ans = None
    for action in res.semantics['actions']:
        try:
            if 'action' not in action:
                ans = "Sorry, I cannot %s. I don't know the answer." % str(action['action'])
                continue

            func = assignments.get(action['action'])
            if func is not None:
                ans = func(action)              

        except Exception as e:
            rospy.logerr(e)
            robot.speech.speak("Whoops")

    if ans is None:
        return False
    else:
        rospy.loginfo("Question was: '%s'?" % res.sentence)
        robot.speech.speak("The answer is %s" % ans)
        return True


def answer_random_gender(action):
    return random.choice(["A male", "A female"])


def answer_predefined_questions(action):
    return action['solution']


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
                        ('girls', 'girls'),
                        ('waiving', 'waiving'),
                        ('raising_left', 'raising_left'),
                        ('raising_right', 'raising_right'),
                        ('pointing_left', 'pointing_left'),
                        ('pointing_right', 'pointing_right'),
                        ('laying', 'laying'),
                        ('sitting', 'sitting')]

    for property_name, property_value in crowd_properties:
        if action['entity'] == property_name:
            return 'In the crowd are %d %s' % (crowd_data[property_value], property_name)
    return 'I dont know'


def answer_placement_location(action):
    entity = action['entity']
    locations = [loc for loc in common_knowledge.locations if loc['name'] == entity]
    if len(locations) == 1:
        room = locations[0]['room']
        return 'The %s can be found in the %s' % (entity, room)
    else:
        return 'I dont know that object'


def answer_count_placement(action):
    entity = action['entity']
    location = action['location']

    locations = [loc for loc in common_knowledge.locations if loc['name'] == entity]
    if len(locations) == 1:
        room = locations[0]['room']
        if loc == action['location']:
            return 'There is one %s in the %s' % (entity, room)
        else:
            return 'There are no %s in the %s' % (entity, room)
    else:
        return 'I should count but I dont know that object %s' % entity

    locations = [loc for loc in locations if loc['room'] == location]
    if not locations:
        return 'There are no %s in the %s' % (entity, location)
    elif len(locations) == 1:
        return 'There is one %s in the %s' % (entity, location)
    else:
        return 'I should count but I dont know that object %s' % entity

def answer_find_objects(action):
    entity = action['entity']
    objects = [obj for obj in common_knowledge.objects if obj['name'] == entity]
    if len(objects) == 1:
        cat = objects[0]['category']
        print cat
        loc, area_name = common_knowledge.get_object_category_location(cat)
        return 'You can find the %s %s %s' % (entity, area_name, loc)
    else:
        return 'I should find %s but I dont know %s' % entity


def answer_find_category(action):
    entity = action['entity']
    loc, area_name = common_knowledge.get_object_category_location(entity)
    return 'You can find the %s on the %s' % (entity, loc)


def answer_object_category(action):
    entity = action['entity']
    objects = [obj for obj in common_knowledge.objects if obj['name'] == entity]
    if len(objects) == 1:
        cat = objects[0]['category']
        return 'The %s belongs to %s' % (entity, cat)
    else:
        return 'I should name a category but I dont know what is %s' % entity


def answer_object_color(action):
    entity = action['entity']
    objects = [obj for obj in common_knowledge.objects if obj['name'] == entity]
    if len(objects) == 1:
        col = objects[0]['color']
        return 'The color of %s is %s' % (entity, col)
    else:
        return 'I should name the color but I dont know %s' % entity


def answer_compare_objects_sizes(action):
    entity_a = action['entity_a']
    entity_b = action['entity_b']
    objects_a = [obj for obj in common_knowledge.objects if obj['name'] == entity_a]
    objects_b = [obj for obj in common_knowledge.objects if obj['name'] == entity_b]
    if len(objects_a) == 1 and len(objects_b) == 1:
        vol_a = objects_a[0]['volume']
        vol_b = objects_b[0]['volume']
        if vol_a > vol_b:
            return 'The object %s is bigger than the object %s' % (entity_a, entity_b)
        elif vol_a < vol_b:
            return 'The object %s is smaller than the object %s' % (entity_a, entity_b)
        else:
        	return 'The objects %s and %s have exactly the same volume' % (entity_a, entity_b)
    else:
        return 'I dont know these objects'


def answer_compare_objects_weight(action):
    entity_a = action['entity_a']
    entity_b = action['entity_b']
    objects_a = [obj for obj in common_knowledge.objects if obj['name'] == entity_a]
    objects_b = [obj for obj in common_knowledge.objects if obj['name'] == entity_b]
    if len(objects_a) == 1 and len(objects_b) == 1:
        w_a = objects_a[0]['weight']
        w_b = objects_b[0]['weight']
        if w_a > w_b:
            return 'The object %s is heavier than the object %s' % (entity_a, entity_b)
        elif w_a < w_b:
            return 'The object %s is lighter than the object %s' % (entity_a, entity_b)
        else:
            return 'The objects %s and %s have exactly the same weight' % (entity_a, entity_b)
    else:
        return 'I dont know these objects'


def answer_compare_objects_categories(action):
    entity_a = action['entity_a']
    entity_b = action['entity_b']
    objects_a = [obj for obj in common_knowledge.objects if obj['name'] == entity_a]
    objects_b = [obj for obj in common_knowledge.objects if obj['name'] == entity_b]
    if len(objects_a) == 1 and len(objects_b) == 1:
        cat_a = objects_a[0]['category']
        cat_b = objects_b[0]['category']
        if cat_a == cat_b:
            return 'Both objects belong to the same category %s' % cat_a
        else:
            return 'These objects belong to different categories'
    else:
        return 'I dont know these objects'


def answer_count_objects_in_category(action):
    entity = action['entity']
    objects = [obj for obj in common_knowledge.objects if obj['category'] == entity]
    if len(objects) > 0:
        count = len(objects)
        return 'The number of objects in category %s is %i' % (entity, count)

##############################################################################
#
# Standalone testing:
#
##############################################################################


class TestRiddleGame(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        self.userdata.crowd_data = {
            "males": 3,
            "men": 2,
            "females": 5,
            "women": 3,
            "children": 3,
            "boys": 1,
            "girls": 2,
            "adults": 5,
            "elders": 1,
            "crowd_size": 8
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
