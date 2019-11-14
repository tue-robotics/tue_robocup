import random

import rospy
import smach

from hmi import HMIResult, TimeoutException
from robot_skills.robot import Robot


def answer(robot, knowledge, res, crowd_data):
    # type: (Robot, object, HMIResult, dict) -> bool
    """
    Robot answers the heard question

    :param robot: robot api object
    :param knowledge: common knowledge data struct
    :param res: HMIResult what has been understood
    :param crowd_data: data about the crowd

    :return: if successfully answered the question, else False
    """
    if 'actions' not in res.semantics:
        robot.speech.speak("Your question was '%s' but I don't know the answer" % res.sentence)
        return False

    # Pass on the provided crowd_data to the real answering function.
    def answer_crowd_questions(_action, _knowledge):
        if not crowd_data:
            robot.speech.speak("I'm sorry but I don't know anything about a crowd.")
            return False
        return answer_count_people(_action, _knowledge, crowd_data)

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
                ans = func(action, knowledge)

        except Exception as e:
            rospy.logerr(e)
            robot.speech.speak("Whoops")

    if ans is None:
        return False
    else:
        rospy.loginfo("Question was: '%s'?" % res.sentence)
        robot.speech.speak("The answer is %s" % ans)
        return True


def answer_random_gender(_, __):
    return random.choice(["A male", "A female"])


def answer_predefined_questions(action, _):
    return action['solution']


def answer_count_people(action, _, crowd_data):
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


def answer_placement_location(action, common_knowledge):
    entity = action['entity']
    locations = [loc for loc in common_knowledge.locations if loc['name'] == entity]
    if len(locations) == 1:
        room = locations[0]['room']
        return 'The %s can be found in the %s' % (entity, room)
    else:
        return 'I dont know that object'


def answer_count_placement(action, common_knowledge):
    entity = action['entity']

    locations = [loc for loc in common_knowledge.locations if loc['name'] == entity]
    if len(locations) == 1:
        room = locations[0]['room']
        if loc == action['location']:
            return 'There is one %s in the %s' % (entity, room)
        else:
            return 'There are no %s in the %s' % (entity, room)
    else:
        return 'I should count but I dont know that object %s' % entity


def answer_find_objects(action, common_knowledge):
    entity = action['entity']
    objects = [obj for obj in common_knowledge.objects if obj['name'] == entity]
    if len(objects) == 1:
        cat = objects[0]['category']
        loc, area_name = common_knowledge.get_object_category_location(cat)
        return 'You can find the %s %s %s' % (entity, area_name, loc)
    else:
        return 'I should find %s but I dont know %s' % entity


def answer_find_category(action, common_knowledge):
    entity = action['entity']
    loc, area_name = common_knowledge.get_object_category_location(entity)
    return 'You can find the %s on the %s' % (entity, loc)


def answer_object_category(action, common_knowledge):
    entity = action['entity']
    objects = [obj for obj in common_knowledge.objects if obj['name'] == entity]
    if len(objects) == 1:
        cat = objects[0]['category']
        return 'The %s belongs to %s' % (entity, cat)
    else:
        return 'I should name a category but I dont know what is %s' % entity


def answer_object_color(action, common_knowledge):
    entity = action['entity']
    objects = [obj for obj in common_knowledge.objects if obj['name'] == entity]
    if len(objects) == 1:
        col = objects[0]['color']
        return 'The color of %s is %s' % (entity, col)
    else:
        return 'I should name the color but I dont know %s' % entity


def answer_compare_objects_sizes(action, common_knowledge):
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


def answer_compare_objects_weight(action, common_knowledge):
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


def answer_compare_objects_categories(action, common_knowledge):
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


def answer_count_objects_in_category(action, common_knowledge):
    entity = action['entity']
    objects = [obj for obj in common_knowledge.objects if obj['category'] == entity]
    if len(objects) > 0:
        count = len(objects)
        return 'The number of objects in category %s is %i' % (entity, count)


class HearAndAnswerQuestions(smach.State):
    def __init__(self, robot, grammar, knowledge, num_questions=1, hear_time=20.0):
        # type: (Robot, str, object, int, float) -> str
        """
        Robot hears and answers questions.

        :param robot: robot api object
        :param grammar: grammar for speech to text
        :param knowledge: knowledge data struct
        :param num_questions: number of questions to be heard and answered
        :param hear_time: timeout for speech to text
        :return: "done"
        """
        smach.State.__init__(self, outcomes=["done"], input_keys=['crowd_data'])
        self.robot = robot
        self.num_questions = num_questions
        self.hear_time = hear_time
        self.grammar = grammar
        self.knowledge = knowledge

    def execute(self, userdata):
        try:
            crowd_data = userdata.crowd_data
        except AttributeError:
            rospy.logwarn("No crowd data in userdata")
            crowd_data = {}

        self.robot.head.look_at_standing_person()

        for _ in xrange(self.num_questions):

            try:
                res = self.robot.hmi.query('Question?', self.grammar, 'T', timeout=self.hear_time)
            except TimeoutException:
                self.robot.speech.speak("I did not hear you. Please ask me the next question!")
                continue

            if not answer(self.robot, self.knowledge, res, crowd_data):
                self.robot.speech.speak("Please ask me the next question!")

        return "done"
