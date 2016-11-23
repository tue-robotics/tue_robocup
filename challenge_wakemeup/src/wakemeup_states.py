#! /usr/bin/env python
import roslib;
import rospy
import smach
import subprocess
import inspect
# import random
# import ed_perception.msg

from std_msgs.msg import Header
import robot_skills.util.msg_constructors as msgs
import geometry_msgs.msg as gm
from cb_planner_msgs_srvs.msg import *
# import math
from smach_ros import SimpleActionState
import robot_smach_states as states
from robot_smach_states.util.designators import *
from robot_smach_states.human_interaction.human_interaction import HearOptionsExtra
from ed.msg import EntityInfo
from dragonfly_speech_recognition.srv import GetSpeechResponse
from robot_smach_states.util.geometry_helpers import *

from robocup_knowledge import load_knowledge
knowledge_objs = load_knowledge('common').objects
# Remove before flight
# knowledge_objs = load_knowledge('challenge_wakemeup').objects
knowledge = load_knowledge('challenge_wakemeup')

# ----------------------------------------------------------------------------------------------------

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


prefix = bcolors.OKBLUE + "[WAKE ME UP] " + bcolors.ENDC

# load item names
object_names = [ o["name"] for o in knowledge_objs if "category" in o and o["category"] is "food" or o["category"] is "drinks"]
names_fruit = [ o["name"] for o in knowledge_objs if "sub-category" in o and o["sub-category"] is "fruit" ]
names_cereal = [ o["name"] for o in knowledge_objs if "sub-category" in o and o["sub-category"] is "cereal" ]
names_milk = [ o["name"] for o in knowledge_objs if "sub-category" in o and o["sub-category"] is "milk" ]

# ----------------------------------------------------------------------------------------------------


class FoodType:
    Cereal = 0
    Fruit = 1
    Milk = 2


def parseFoodType(item, got_fruit, got_cereal, got_milk):

    # print prefix + bcolors.OKBLUE + "parseFoodType" + bcolors.ENDC

    # if a fruit has not been picked, you can search there
    if not got_fruit and item in names_fruit:
        print prefix + item + " its a fruit!"
        return FoodType.Fruit

    # if a cereal has not been picked, you can search there
    if not got_cereal and item in names_cereal:
        print prefix + item + " its a cereal"
        return FoodType.Cereal

    # if a milk has not been picked, you can search there
    if not got_milk and item in names_milk:
        print prefix + item + " its a milk!"
        return FoodType.Milk

    # print prefix + item + " was not matched!"
    return None

# ----------------------------------------------------------------------------------------------------

class ConfigureEd(smach.State):
    '''Smach state to (re)configure ed. Takes a configuration in the form
    of a dictionary containing at least kinect_segmentation_continuous_mode
    to a Bool, perception_continuous_mode to another Bool and
    disabled_plugins to a list of plugins that should be disabled. An
    optional argument is the Bool reset. This resets the world model after
    configuration'''
    def __init__(self, robot, configuration={},reset = None):
        smach.State.__init__( self, outcomes=['done'])
        self.robot = robot

        if type(configuration) == Designator:
            if configuration.resolve_type == dict:
                self.config = configuration.resolve()
            else:
                rospy.logerr("Configuration designator must resolve to a dict with the new ED configuration")
        elif type(configuration) == dict:
            self.config = configuration
        else:
            rospy.logerr("keyword argument 'configuration' must either be a dict with the new ED configuration, or a designator that resolves to such a dict")

        if reset:
            self.reset = reset
        else:
            self.reset = True
            rospy.logwarn("[CONFIGURE ED] Keyword argument 'reset' not set, defaulting to 'True'. If you do not want to reset ed after reconfiguration, set to 'False'.")

    def execute(self, userdata):

        if self.reset:
            self.robot.ed.reset()

        return 'done'





# ----------------------------------------------------------------------------------------------------

# For testing!!!
# class GetOrder(smach.State):
#     def __init__(self, robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
#         smach.State.__init__( self, outcomes=['succeeded', 'failed'])
#         self.breakfastCereal = breakfastCerealDes
#         self.breakfastFruit = breakfastFruitDes
#         self.breakfastMilk = breakfastMilkDes

#     def execute(self, userdata):
#         self.breakfastCereal.write("coconut_cereals")
#         self.breakfastMilk.current   = "papaya_milk"
#         self.breakfastFruit.write("apple")
#         return "succeeded"

# ----------------------------------------------------------------------------------------------------

# TODO: Generalize and put in human interaction?
class GetOrder(smach.State):
    ''' Smach state to listen to and parse the breakfast order. Takes variable
    designators which it will assign strings to. These strings are the names of
    the ordered items.'''
    def __init__(self, robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
        smach.State.__init__(   self,
                                outcomes=['succeeded', 'failed'])

        self.robot = robot

        is_writeable(breakfastCerealDes)
        self.breakfastCereal = breakfastCerealDes

        is_writeable(breakfastFruitDes)
        self.breakfastFruit = breakfastFruitDes

        is_writeable(breakfastMilkDes)
        self.breakfastMilk = breakfastMilkDes


    def execute(self, userdata):
        print prefix + bcolors.OKBLUE + "GetOrder" + bcolors.ENDC

        # import ipdb; ipdb.set_trace()
        # Initializations

        got_cereal = False
        got_fruit = False
        got_milk = False
        heard_correctly = False

        word_beginning = ""
        word_preposition = ""
        word_item1 = ""
        word_item2 = ""
        word_item3 = ""

        self.breakfastFruit.write("")
        self.breakfastCereal.write("")
        self.breakfastMilk.write("")

        # define allowed sentences, [] means optional
        sentence = Designator("(([<beginning>] <item1> [<preposition>] <item2> [<preposition>] <item3>) | \
                                ([<beginning>] <item1> [<preposition>] <item2>))")

        choices = Designator({  "beginning" :   ["I want", "I would like", "a", "one"],
                                "preposition" : ["and", "and a", "and an", "with a", "with a", "with", "a"],
                                "item1" :       [i.replace("_"," ") for i in names_cereal + names_fruit + names_milk],
                                "item2" :       [i.replace("_"," ") for i in names_cereal + names_fruit + names_milk],
                                "item3" :       [i.replace("_"," ") for i in names_cereal + names_fruit + names_milk]})

        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        state = HearOptionsExtra(self.robot, sentence, choices, answer, time_out=rospy.Duration(20))
        outcome = state.execute()

        # process response
        if outcome == "heard":

            # resolve words separatey since some of them might not have been caught
            try:
                word_beginning = answer.resolve().choices["beginning"].replace(" ","_")
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving: " + str(ke) + bcolors.ENDC
                pass

            try:
                word_item3 = answer.resolve().choices["item3"].replace(" ","_")
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving: " + str(ke) + bcolors.ENDC
                pass

            try:
                word_preposition = answer.resolve().choices["preposition"].replace(" ","_")
                word_item1 = answer.resolve().choices["item1"].replace(" ","_")
                word_item2 = answer.resolve().choices["item2"].replace(" ","_")
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving: " + str(ke) + bcolors.ENDC
                pass

            print "{}What was heard: {} {} {} {} {} {}".format(prefix, word_beginning , word_item1 , word_preposition ,  word_item2 , word_preposition , word_item3)

            # find first item's type
            if parseFoodType(word_item1, got_fruit, got_cereal, got_milk) == FoodType.Fruit:
                self.breakfastFruit.write(word_item1)
                got_fruit = True
                print "{}First item fruit".format(prefix)
            elif parseFoodType(word_item1, got_fruit, got_cereal, got_milk) == FoodType.Cereal:
                self.breakfastCereal.write(word_item1)
                got_cereal = True
                print "{}First item cereal".format(prefix)
            elif parseFoodType(word_item1, got_fruit, got_cereal, got_milk) == FoodType.Milk:
                self.breakfastMilk.write(word_item1)
                got_milk = True
                print "{}First item milk".format(prefix)
            else:
                print "{}Could not get a match with word_item1 = {}".format(prefix, word_item1)

            # find second item's type
            if parseFoodType(word_item2, got_fruit, got_cereal, got_milk) == FoodType.Fruit:
                self.breakfastFruit.write(word_item2)
                got_fruit = True
                print "{}Second item Fruit".format(prefix)
            elif parseFoodType(word_item2, got_fruit, got_cereal, got_milk) == FoodType.Cereal:
                self.breakfastCereal.write(word_item2)
                got_cereal = True
                print "{}Second item Cereal".format(prefix)
            elif parseFoodType(word_item2, got_fruit, got_cereal, got_milk) == FoodType.Milk:
                self.breakfastMilk.write(word_item2)
                got_milk = True
                print "{}Second item Milk".format(prefix)
            else:
                print "{}Could not get a match with word_item2 = {}".format(prefix, word_item2)

            # third type might not exist if its milk
            if word_item3 :

                # find second item's type
                if parseFoodType(word_item3, got_fruit, got_cereal, got_milk) == FoodType.Fruit :
                    self.breakfastFruit.write(word_item3)
                    got_fruit = True
                    print "{}Third item Fruit".format(prefix)
                elif parseFoodType(word_item3, got_fruit, got_cereal, got_milk) == FoodType.Cereal :
                    self.breakfastCereal.write(word_item3)
                    got_cereal = True
                    print "{}Third item Cereal".format(prefix)
                elif parseFoodType(word_item3, got_fruit, got_cereal, got_milk) == FoodType.Milk :
                    self.breakfastMilk.write(word_item3)
                    got_milk = True
                    print "{}Third item Milk".format(prefix)
                else:
                    print "{}Could not get a match with word_item3 = {}".format(prefix, word_item3)

                # just a consistency check
                if not got_milk:
                    print prefix + "Still don't know what type of milk it is! Reseting to default." + bcolors.ENDC
                    self.breakfastMilk.write(knowledge.default_milk)

            else:
                self.breakfastMilk.write(knowledge.default_milk)
                got_milk = True

            print "{}Response: fruit = {}, cereal = {} , milk = {}".format(prefix, self.breakfastFruit.resolve(), self.breakfastCereal.resolve(), self.breakfastMilk.resolve())

            if not self.breakfastCereal.resolve() or not self.breakfastFruit.resolve() or not self.breakfastMilk.resolve() :
                heard_correctly = False
                print prefix + bcolors.FAIL + "One of the food types was empty" + bcolors.ENDC
            else:
                heard_correctly = True

        else:
            heard_correctly = False

        # rospy.sleep(2)

        if heard_correctly:
            return 'succeeded'
        else:
            return 'failed'

# ----------------------------------------------------------------------------------------------------

class PickDefaultOrder(smach.State):
    '''Smach state to pick the default order as defined in the knowledge'''
    def __init__(self, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
        smach.State.__init__(self, outcomes=['done'])

        is_writeable(breakfastCerealDes)
        self.breakfastCereal = breakfastCerealDes

        is_writeable(breakfastFruitDes)
        self.breakfastFruit = breakfastFruitDes

        is_writeable(breakfastMilkDes)
        self.breakfastMilk = breakfastMilkDes

    def execute(self, userdata):
        self.breakfastCereal.write(knowledge.default_cereal)
        self.breakfastMilk.write(knowledge.default_milk)
        self.breakfastFruit.write(knowledge.default_fruit)

        return 'done'

# ----------------------------------------------------------------------------------------------------

# TODO: Generalize and put in human interaction smach states?
class ConfirmOrder(smach.State):
    '''Smach state to ask for confirmation of the breakfast order'''
    def __init__(self, robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
        smach.State.__init__(self, outcomes=['done'])

        self.robot = robot
        self.breakfastCereal = breakfastCerealDes
        self.breakfastFruit = breakfastFruitDes
        self.breakfastMilk = breakfastMilkDes

    def execute(self, userdata):
        print prefix + bcolors.OKBLUE + "ConfirmOrder" + bcolors.ENDC

        self.robot.speech.speak("I understand you want an " +  self.breakfastFruit.resolve() + " and " + self.breakfastCereal.resolve() + " with " + self.breakfastMilk.resolve() + ". Is that correct?", block=True)

        return 'done'

# ----------------------------------------------------------------------------------------------------


class CancelHeadGoals(smach.State):
    '''Smach state to cancel all head goals'''
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata):
        print prefix + bcolors.OKBLUE + "CancelHeadGoals" + bcolors.ENDC

        self.robot.head.cancel_goal()

        return 'done'


# ----------------------------------------------------------------------------------------------------


class LookAtBedTop(smach.State):
    '''Smach state to make the robot look at the bed top. Make sure to cancel this head goal when it
    is no longer needed.'''
    def __init__(self, robot, entity_id):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot
        self.entity = self.robot.ed.get_entity(id=entity_id)

    def execute(self, userdata):
        print prefix + bcolors.OKBLUE + "LookAtBedTop" + bcolors.ENDC

        # Cancel previous head goals
        self.robot.head.cancel_goal()

        # Look at bed top
        headGoal = msgs.PointStamped(x=self.entity.pose.position.x, y=self.entity.pose.position.y, z=knowledge.matress_height, frame_id="/map")
        self.robot.head.look_at_point(point_stamped=headGoal, end_time=0, timeout=4)

        return 'succeeded'


# ----------------------------------------------------------------------------------------------------

class LookIfSomethingsThere(smach.State):
    '''Smach state to check if an entitydesignator resolves. It will check for 'timeout' seconds at a
    rate of 1/'sleep' Hz. Returns 'there' if the designator resolves and 'not_there' if it does not.'''
    def __init__(self, robot, designator, timeout=0, sleep=0.2):
        smach.State.__init__(self, outcomes=['there', 'not_there'])
        self.robot = robot
        self.designator = designator
        self.timeout = rospy.Duration(timeout)
        self.sleep = sleep

    def execute(self, userdata):
        self.start_time = rospy.Time.now()

        while rospy.Time.now() - self.start_time < self.timeout:
            if self.designator.resolve():
                return 'there'
            else:
                rospy.sleep(self.sleep)

        return 'not_there'

# ----------------------------------------------------------------------------------------------------

class Evaluate(smach.State):
    '''Smach state to evaluate a number of operations that were to be performed. 'options' is a list of
    keys to values in the dictionary 'results', which are Bools indicating whether this task succeeded
    or not. '''
    def __init__(self, options, results):
        smach.State.__init__(self, outcomes=['all_succeeded','partly_succeeded','all_failed'])
        self.options = options
        self.results = results
        self.something_failed = False
        self.something_succeeded = False

    def execute(self, userdata):
        print self.options
        print self.results.resolve()
        for option in self.options:
            if self.results.resolve()[option]:
                something_succeeded = True
            else:
                something_failed = True

        if self.something_succeeded and self.something_failed:
            return 'partly_succeeded'
        elif self.something_succeeded and not self.something_failed:
            return 'all_succeeded'
        elif not self.something_succeeded and self.something_failed:
            return 'all_failed'
        else:
            return 'all_failed'

# ----------------------------------------------------------------------------------------------------

class AddPositiveResult(smach.State):
    '''Smach state to add a positive result to the dictionary of results the results designator
    resolves to. Uses the string in item_designator as key to this dict. '''
    def __init__(self, results_designator, item_designator):
        smach.State.__init__(self, outcomes=['done'])
        self.results = results_designator
        self.item = item_designator

    def execute(self, userdata):
        item = self.item.resolve()
        print item
        result = self.results.resolve()
        result[item] = True
        self.results.write(result)
        return "done"


# ----------------------------------------------------------------------------------------------------

class SelectItem(smach.State):
    '''From the list of generic options (milk,fruit,cereal), selects the next one, picks the
    associated specific item asked for (from asked_items, which are all within a sub-category in the
    generic options). Fills generic and specific item designators with strings and generates a nav
    goal for getting it. Will return 'all_done' when all items in the options have been selected. '''
    def __init__(self, robot, options, asked_items, generic_item, specific_item, item_nav_goal, item_lookat_goal):
        smach.State.__init__(self, outcomes=['selected', 'all_done'])
        self.robot = robot
        self.options = options
        self.asked_items_des = asked_items
        self.count = len(self.options)
        self.current = 0

        is_writeable(generic_item)
        is_writeable(specific_item)
        is_writeable(item_nav_goal)
        is_writeable(item_lookat_goal)

        self.generic_item = generic_item
        self.specific_item = specific_item
        self.nav_goal = item_nav_goal
        self.lookat_goal = item_lookat_goal

    def execute(self, userdata):
        if self.current == self.count:
            self.current = 0
            return 'all_done'
        else:
            self.generic_item.write(self.options[self.current])

            asked_items = [d.resolve() for d in self.asked_items_des]
            category_items = [i['name'] for i in knowledge_objs if 'sub-category' in i and i['sub-category']==self.generic_item.resolve()]

            print "asked items: {}".format(asked_items)
            print "category items: {}".format(category_items)

            self.specific_item.write(list(set(category_items).intersection(asked_items))[0])

            self.robot.speech.speak("I will get your "+self.generic_item.resolve()+" now.", block=False)

            self.nav_goal.write({
                                        EdEntityDesignator(self.robot, id=knowledge.item_nav_goal['in_front_of_'+self.generic_item.resolve()]) : "in_front_of",
                                        EdEntityDesignator(self.robot, id=knowledge.item_nav_goal['in']) : "in"
                                    })

            self.lookat_goal.write(EdEntityDesignator(self.robot, id=knowledge.item_nav_goal['lookat_'+self.generic_item.resolve()]))

        self.current += 1
        return 'selected'

# ----------------------------------------------------------------------------------------------------
# TODO maybe store recognized objects for future use?

class FindItem(smach.State):
    '''Smach state to find an object of the type contained in the type designator. Looks within
    sensor_range, possibly using the supporting object contained in on_object_des. Assigns the found
    entity info to result_des. '''
    def __init__(self, robot, sensor_range, type_des, result_des, on_object_des=None):
        smach.State.__init__(self, outcomes=['item_found', 'not_found'])
        self.robot = robot
        self.sensor_range = sensor_range
        self.on_object_des = on_object_des
        self.result_type_des = type_des

        is_writeable(result_des)
        self.result_des = result_des

    def execute(self, userdata):
        self.on_object = self.on_object_des.resolve().resolve()
        self.result_type = self.result_type_des.resolve()
        print "result_type = ", self.result_type

        if self.result_type in names_fruit:
            self.items_were_looking_for = names_fruit
        elif self.result_type in names_milk:
            self.items_were_looking_for = names_milk
        elif self.result_type in names_cereal:
            self.items_were_looking_for = names_cereal

        center_point = gm.Point()
        frame_id = "/"+self.on_object.id

        center_point.z = self.on_object.z_max

        rospy.loginfo('Look at %s in frame %s' % (repr(center_point).replace('\n', ' '), frame_id))
        point_stamped = gm.PointStamped(point=center_point,
                                     header=Header(frame_id=frame_id))

        self.robot.head.look_at_point(point_stamped)
        rospy.sleep(rospy.Duration(2.0))

        entity_ids = self.robot.ed.segment_kinect(max_sensor_range = self.sensor_range)
        print "entity_ids 1: ", entity_ids

        filtered_ids = []
        for entity_id in entity_ids:
            e = self.robot.ed.get_entity(entity_id)

            # if onTopOff(e, self.on_object):
            #     print "id is on top of object"
            # else:
            #     print "id is NOT on top of object"

            if e and self.on_object and not e.type and onTopOff(e, self.on_object):
                filtered_ids.append(e.id)

        print "filtered_ids =", filtered_ids
        print "self.items_were_looking_for =", self.items_were_looking_for

        entity_types = self.robot.ed.classify(ids=filtered_ids, types=self.items_were_looking_for)

        self.robot.head.cancel_goal()

        # ###############
        # #print "result designator = ". self.result_des


        # # hack to check grab state.
        # if len(filtered_ids)>0:
        #     self.result_des.write(self.robot.ed.get_entity(filtered_ids[0]))
        #     return 'item_found'
        # ##############

        for i in range(len(filtered_ids)):
            if entity_types[i] == self.result_type:
                self.result_des.write(self.robot.ed.get_entity(filtered_ids[i]))
                return 'item_found'

        # if wanted item is not found then ..
        type_ids = dict(zip(entity_types,filtered_ids))

        print "I found the following items with the ids: {}".format(type_ids)

        found_milk      = list(set(entity_types).intersection(names_milk))
        found_cereal    = list(set(entity_types).intersection(names_cereal))
        found_fruit     = list(set(entity_types).intersection(names_fruit))

        print "I need to get a " + self.result_type
        print "I found the following milk, cereal and fruit items: {}".format(found_milk+found_cereal+found_fruit)

        if len(found_milk) > 0 and self.result_type in names_milk:
            print self.robot.ed.get_entity(found_milk[0])
            self.result_des.write(self.robot.ed.get_entity(type_ids[found_milk[0]]))
            return 'item_found'
        elif len(found_cereal) > 0 and self.result_type in names_cereal:
            print self.robot.ed.get_entity(found_cereal[0])
            self.result_des.write(self.robot.ed.get_entity(type_ids[found_cereal[0]]))
            return 'item_found'
        elif len(found_fruit) > 0 and self.result_type in names_fruit:
            print self.robot.ed.get_entity(found_fruit[0])
            self.result_des.write(self.robot.ed.get_entity(type_ids[found_fruit[0]]))
            return 'item_found'
        elif self.result_type in names_milk and len(filtered_ids)>0:
            rospy.logwarn("No milk found, grabbing something anyway!")
            self.result_des.write(self.robot.ed.get_entity(filtered_ids[0]))

        # TODO: maybe go to another position to look again?



        return 'not_found'

# ----------------------------------------------------------------------------------------------------

class ScanTableTop(smach.State):
    '''Smach state to look at the table top and take a snapshot to see if there are things on it.
    Useful for finding free space before using the EmptySpotDesignator'''
    def __init__(self, robot, table):
        smach.State.__init__(self, outcomes=['done','failed'])
        self.robot = robot
        self.table = table

    def execute(self, userdata):
        table = self.table.resolve()
        center_point = gm.Point()
        frame_id = "/"+self.table.id

        center_point.z = table.z_max

        rospy.loginfo('Look at %s in frame %s' % (repr(center_point).replace('\n', ' '), frame_id))
        point_stamped = gm.PointStamped(point=center_point,
                                     header=Header(frame_id=frame_id))
        self.robot.head.look_at_point(point_stamped)
        rospy.sleep(rospy.Duration(0.5))

        self.robot.ed.segment_kinect(max_sensor_range = 2.0)
        self.robot.head.cancel_goal()
        return 'done'

# ----------------------------------------------------------------------------------------------------

class NavigateToSymbolic(states.NavigateToSymbolic):
    '''NavigateToSymbolic for an entity_designator_area_name_map which is contained in a designator.
    This is useful if the navigation goal changes with different executions of the NavigateToSymbolic
    state.'''
    def __init__(self, robot, entity_designator_area_name_map, entity_lookat_designator):
        super(states.NavigateToSymbolic, self).__init__(robot)

        self.robot                 = robot

        #Check that the entity_designator_area_name_map's keys all resolve to EntityInfo's
        assert(all(entity_desig.resolve_type == EntityInfo for entity_desig in entity_designator_area_name_map.resolve().keys()))
        self.entity_designator_area_name_map = entity_designator_area_name_map

        # check_resolve_type(entity_lookat_designator.resolve(), EntityInfo) #Check that the entity_designator resolves to an Entity
        self.entity_lookat_designator = entity_lookat_designator

    def generateConstraint(self):
        ''' PositionConstraint '''
        entity_id_area_name_map = {}
        for desig, area_name in self.entity_designator_area_name_map.resolve().iteritems():
            entity = desig.resolve()
            if entity:
                entity_id_area_name_map[entity.id] = area_name
            else:
                rospy.logerr("Designator {0} in entity_designator_area_name_map resolved to {1}.".format(desig, entity))
                entity_id_area_name_map[entity] = area_name #Put a None item in the dict. We check on that and if there's a None, something failed.

        if None in entity_id_area_name_map:
            rospy.logerr("At least 1 designator in self.entity_designator_area_name_map failed")
            return None


        pc = self.robot.ed.navigation.get_position_constraint(entity_id_area_name_map)

        #Orientation constraint is the entity itself...
        entity_lookat = self.entity_lookat_designator.resolve().resolve()
        if not entity_lookat:
            rospy.logerr("Could not resolve entity_lookat_designator".format(self.entity_lookat_designator))
            return None

        oc = OrientationConstraint(look_at=entity_lookat.pose.position, frame="/map")

        return pc, oc


# ----------------------------------------------------------------------------------------------------

class SensedHandoverFromHuman(smach.StateMachine):
    '''
    State that enables low level grab reflex. Besides a robot object, needs
    an arm and an entity to grab, which is either one from ed through the
    grabbed_entity_designator or it is made up in the
    CloseGripperOnHandoverToRobot state and given the grabbed_entity_label
    as id.
    '''
    def __init__(self, robot, arm_designator, grabbed_entity_label="", grabbed_entity_designator=None, timeout=10):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        check_type(arm_designator, Arm)
        if not grabbed_entity_designator and grabbed_entity_label == "":
            rospy.logerr("No grabbed entity label or grabbed entity designator given")

        with self:
            smach.StateMachine.add( "POSE",
                                    states.ArmToJointConfig(robot, arm_designator, knowledge.handover_joint_config),
                                    transitions={   'succeeded':'OPEN_BEFORE_INSERT',
                                                    'failed':'OPEN_BEFORE_INSERT'})

            smach.StateMachine.add( 'OPEN_BEFORE_INSERT',
                                    states.SetGripper(robot, arm_designator, gripperstate=states.ArmState.OPEN),
                                    transitions={   'succeeded'    :   'SAY1',
                                                    'failed'       :   'SAY1'})

            smach.StateMachine.add( "SAY1",
                                    states.Say(robot,'Please hand over the object by pushing it gently in my gripper'),
                                    transitions={'spoken':'CLOSE_AFTER_INSERT'})

            smach.StateMachine.add( 'CLOSE_AFTER_INSERT',
                                    CloseGripperOnHandoverToRobot(  robot,
                                                                    arm_designator,
                                                                    grabbed_entity_label=grabbed_entity_label,
                                                                    grabbed_entity_designator=grabbed_entity_designator,
                                                                    timeout=timeout),
                                    transitions={   'succeeded'    :   'succeeded',
                                                    'failed'       :   'OPEN_FALLBACK'})

            smach.StateMachine.add( 'OPEN_FALLBACK',
                                    states.SetGripper(robot, arm_designator, gripperstate=states.ArmState.CLOSE),
                                    transitions={   'succeeded'    :   'succeeded',
                                                    'failed'       :   'failed'})



# ----------------------------------------------------------------------------------------------------

class SensedHandoverToHuman(smach.StateMachine):
    '''
    State that enables low level release reflex. Besides a robot object, needs
    an arm and an entity to grab, which is either one from ed through the
    grabbed_entity_designator or it is made up in the
    CloseGripperOnHandoverToRobot state and given the grabbed_entity_label
    as id.
    '''
    def __init__(self, robot, arm_designator, timeout=10):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        #A designator can resolve to a different item every time its resolved. We don't want that here, so lock
        check_type(arm_designator, Arm)
        locked_arm = LockingDesignator(arm_designator)

        with self:
            smach.StateMachine.add( "LOCK_ARM",
                                    states.LockDesignator(locked_arm),
                                    transitions={'locked'         :'SPINDLE_MEDIUM'})

            smach.StateMachine.add( "SPINDLE_MEDIUM",
                                    states.ResetTorso(robot),
                                    transitions={'done'         :'MOVE_HUMAN_HANDOVER_JOINT_GOAL'})

            smach.StateMachine.add( "MOVE_HUMAN_HANDOVER_JOINT_GOAL",
                                    states.ArmToJointConfig(robot, locked_arm, knowledge.handover_joint_config),
                                    transitions={ 'succeeded'   :'SAY_OPEN_GRIPPER',
                                                  'failed'      :'SAY_OPEN_GRIPPER'})

            smach.StateMachine.add( "SAY_OPEN_GRIPPER",
                                    states.Say(robot, [ "Please pull my gripper to take it after my lights turned white!"]),
                                    transitions={   'spoken'    :'OPEN_GRIPPER_ON_HANDOVER'})

            smach.StateMachine.add( 'OPEN_GRIPPER_ON_HANDOVER',
                                    OpenGripperOnHandoverToHuman(robot, locked_arm, timeout=timeout),
                                    transitions={'succeeded'    :   'SAY_THERE_YOU_ARE',
                                                 'failed'       :   'SAY_OPEN_GRIPPER_ANYWAY'})

            smach.StateMachine.add('CLOSE_GRIPPER_HANDOVER',
                                    states.SetGripper(robot, locked_arm, gripperstate=states.ArmState.CLOSE, timeout=1.0),
                                    transitions={'succeeded'    :   'RESET_TORSO',
                                                 'failed'       :   'RESET_TORSO'})

            smach.StateMachine.add( "SAY_OPEN_GRIPPER_ANYWAY",
                                    states.Say(robot, [ "Please take this from me now"]),
                                    transitions={   'spoken'    :'SAY_THERE_YOU_ARE'})

            smach.StateMachine.add( 'OPEN_GRIPPER_FALLBACK',
                                    states.SetGripper(robot, locked_arm, gripperstate=states.ArmState.OPEN, timeout=2.0),
                                    transitions={'succeeded'    :   'RESET_ARM',
                                                 'failed'       :   'RESET_ARM'})

            smach.StateMachine.add( "SAY_THERE_YOU_ARE",
                                    states.Say(robot, [ "There you are!"], block=False),
                                    transitions={   'spoken'    :'OPEN_GRIPPER_FALLBACK'})

            smach.StateMachine.add( 'RESET_ARM',
                                    states.ArmToJointConfig(robot, locked_arm, 'reset'),
                                    transitions={'succeeded'    :'CLOSE_GRIPPER_HANDOVER',
                                                  'failed'      :'CLOSE_GRIPPER_HANDOVER'   })

            smach.StateMachine.add( 'RESET_TORSO',
                                    states.ResetTorso(robot),
                                    transitions={'done':'UNLOCK_ARM'})

            smach.StateMachine.add( "UNLOCK_ARM",
                                    states.UnlockDesignator(locked_arm),
                                    transitions={'unlocked'         :'succeeded'})

# ----------------------------------------------------------------------------------------------------

class OpenGripperOnHandoverToHuman(smach.State):
    '''Smach state for opening the gripper when handing over an object to a human. Sets lights to
    white as soon as it is ready to feel if someone grabbed something from the gripper.'''
    def __init__(self, robot, arm_designator, timeout=10):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.arm_designator = arm_designator
        self.timeout = timeout

    def execute(self,userdata):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        self.robot.lights.set_color(1.0,1.0,1.0)
        if arm.handover_to_human(self.timeout):
            arm.occupied_by = None
            self.robot.lights.reset()
            return "succeeded"
        else:
            self.robot.lights.reset()
            return "failed"

# ----------------------------------------------------------------------------------------------------

class CloseGripperOnHandoverToRobot(smach.State):
    '''Smach state for closing the gripper when handing over an object to the robot. Sets lights to
    white as soon as it is ready to feel if someone pushed something into the gripper.'''
    def __init__(self, robot, arm_designator, grabbed_entity_label="", grabbed_entity_designator=None, timeout=10):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.arm_designator = arm_designator
        self.timeout = timeout
        self.grabbed_entity_designator = grabbed_entity_designator
        self.grabbed_entity_label = grabbed_entity_label

    def execute(self,userdata):
        arm = self.arm_designator.resolve()
        if not arm:
            rospy.logerr("Could not resolve arm")
            return "failed"

        self.robot.lights.set_color(1.0,1.0,1.0)
        if arm.handover_to_robot(self.timeout):

            if self.grabbed_entity_designator:
                arm.occupied_by = self.grabbed_entity_designator
            else:
                if self.grabbed_entity_label:
                    handed_entity = EntityInfo(id=self.grabbed_entity_label)
                    arm.occupied = handed_entity
                else:
                    rospy.logerr("No grabbed entity designator and no label for dummy entity given")
                    self.robot.lights.reset()
                    return "failed"
            self.robot.lights.reset()
            return "succeeded"
        else:
            self.robot.lights.reset()
            return "failed"

# ----------------------------------------------------------------------------------------------------
