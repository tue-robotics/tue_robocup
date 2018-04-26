#!/usr/bin/python

import rospy
import smach
import sys
import random
import traceback

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

DEFAULT_HEAR_TIME = 15.0

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

    assignments = [('random_gender',    answer_random_gender),
                   ('answer',           answer_predefined_questions),
                   ('count',            answer_crowd_questions),
                   ('find_placement',   answer_placement_location),
                   ('count_placement',  answer_count_placement),
                   ('find_object',      answer_find_objects),
                   ('find_category',    answer_find_category),
                   ('return_category',  answer_object_category),
                   ('return_color',     answer_object_color),
                   ('compare_category', answer_compare_objects_categories),
                   ('count_object',     answer_count_objects_in_category),
                   ]

    for action in res.semantics['actions']:
        try:
            if 'action' not in action:
                continue

            answer = "Sorry, I cannot %s. I don't know the answer." % str(action['action'])
            for name, func in assignments:
                if action['action'] == name:
                    answer = func(action)
                    break

        except Exception as e:
            rospy.logerr(traceback.format_exc())
            robot.speech.speak("Whoops")

    rospy.loginfo("Question was: '%s'?" % res.sentence)
    robot.speech.speak("The answer is %s" % answer)
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
                        ('girls', 'girls')]

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
    if not locations:
        return 'I should count but I dont know that object %s' % entity

    locations = [loc for loc in locations if loc['room'] == location]
    if not locations:
        return 'There are no %s in the %s' % (entity, location)
    elif len(locations) == 1:
        return 'There is one %s in the %s' % (entity, location)
    else:
        return 'There are %d %s in the %s' % (len(locations), entity, location)

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
