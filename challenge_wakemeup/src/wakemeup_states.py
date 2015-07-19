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
    
class Initialize(states.Initialize):
    def __init__(self, robot=None, ed_configuration={}):
        states.Initialize.__init__(self,robot)
        self.robot = robot
        self.ed_configuration = ed_configuration

    def execute(self, userdata):
        outcome = states.Initialize.execute(self,userdata)
        self.robot.ed.configure_kinect_segmentation(continuous=self.ed_configuration['kinect_segmentation_continuous_mode'])
        self.robot.ed.configure_perception(continuous=self.ed_configuration['perception_continuous_mode'])
        self.robot.ed.disable_plugins(plugin_names=[plugin for plugin in self.ed_configuration["disabled_plugins"]])
        self.robot.ed.reset()

        return outcome


# ----------------------------------------------------------------------------------------------------

# For testing!!!
# class GetOrder(smach.State):
#     def __init__(self, robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
#         smach.State.__init__( self, outcomes=['succeeded', 'failed'])
#         self.breakfastCereal = breakfastCerealDes
#         self.breakfastFruit = breakfastFruitDes
#         self.breakfastMilk = breakfastMilkDes

#     def execute(self, userdata):
#         self.breakfastCereal.current = "coconut_cereals"
#         self.breakfastMilk.current   = "papaya_milk"
#         self.breakfastFruit.current  = "apple"
#         return "succeeded"

# ----------------------------------------------------------------------------------------------------

class GetOrder(smach.State):
    def __init__(self, robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
        smach.State.__init__(   self, 
                                outcomes=['succeeded', 'failed'])

        self.robot = robot
        self.breakfastCereal = breakfastCerealDes
        self.breakfastFruit = breakfastFruitDes
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

        self.breakfastFruit.current  = ""
        self.breakfastCereal.current = ""
        self.breakfastMilk.current   = ""

        # define allowed sentences, [] means optional
        sentence = Designator("(([<beginning>] <item1> [<preposition>] <item2> [<preposition>] <item3>) | \
                                ([<beginning>] <item1> [<preposition>] <item2>))")

        choices = Designator({  "beginning" :   ["I want", "I would like", "a", "one"],
                                "preposition" : ["and", "and a", "and an", "with a", "with a", "with", "a"],
                                "item1" :       names_cereal + names_fruit + names_milk,
                                "item2" :       names_cereal + names_fruit + names_milk,
                                "item3" :       names_cereal + names_fruit + names_milk})

        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        state = HearOptionsExtra(self.robot, sentence, choices, answer, time_out=rospy.Duration(20))
        outcome = state.execute()

        # process response
        if outcome == "heard":

            # resolve words separatey since some of them might not have been caught
            try:
                word_beginning = answer.resolve().choices["beginning"]
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving: " + str(ke) + bcolors.ENDC
                pass

            try:
                word_item3 = answer.resolve().choices["item3"]
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving: " + str(ke) + bcolors.ENDC
                pass

            try:
                word_preposition = answer.resolve().choices["preposition"]
                word_item1 = answer.resolve().choices["item1"]
                word_item2 = answer.resolve().choices["item2"]
            except KeyError, ke:
                print prefix + bcolors.FAIL + "KeyError resolving: " + str(ke) + bcolors.ENDC
                pass

            print "{}What was heard: {} {} {} {} {} {}".format(prefix, word_beginning , word_item1 , word_preposition ,  word_item2 , word_preposition , word_item3)

            # find first item's type
            if parseFoodType(word_item1, got_fruit, got_cereal, got_milk) == FoodType.Fruit:
                self.breakfastFruit.current = word_item1
                got_fruit = True
                print "{}First item fruit".format(prefix)
            elif parseFoodType(word_item1, got_fruit, got_cereal, got_milk) == FoodType.Cereal:
                self.breakfastCereal.current = word_item1
                got_cereal = True
                print "{}First item cereal".format(prefix)
            elif parseFoodType(word_item1, got_fruit, got_cereal, got_milk) == FoodType.Milk:
                self.breakfastMilk.current = word_item1
                got_milk = True
                print "{}First item milk".format(prefix)
            else:
                print "{}Could not get a match with word_item1 = {}".format(prefix, word_item1)

            # find second item's type
            if parseFoodType(word_item2, got_fruit, got_cereal, got_milk) == FoodType.Fruit:
                self.breakfastFruit.current = word_item2
                got_fruit = True
                print "{}Second item Fruit".format(prefix)
            elif parseFoodType(word_item2, got_fruit, got_cereal, got_milk) == FoodType.Cereal:
                self.breakfastCereal.current = word_item2
                got_cereal = True
                print "{}Second item Cereal".format(prefix)
            elif parseFoodType(word_item2, got_fruit, got_cereal, got_milk) == FoodType.Milk:
                self.breakfastMilk.current = word_item2
                got_milk = True
                print "{}Second item Milk".format(prefix)
            else:
                print "{}Could not get a match with word_item2 = {}".format(prefix, word_item2)

            # third type might not exist if its milk
            if word_item3 :

                # find second item's type
                if parseFoodType(word_item3, got_fruit, got_cereal, got_milk) == FoodType.Fruit :
                    self.breakfastFruit.current = word_item3
                    got_fruit = True
                    print "{}Third item Fruit".format(prefix)
                elif parseFoodType(word_item3, got_fruit, got_cereal, got_milk) == FoodType.Cereal :
                    self.breakfastCereal.current = word_item3
                    got_cereal = True
                    print "{}Third item Cereal".format(prefix)
                elif parseFoodType(word_item3, got_fruit, got_cereal, got_milk) == FoodType.Milk :
                    self.breakfastMilk.current = word_item3
                    got_milk = True
                    print "{}Third item Milk".format(prefix)
                else:
                    print "{}Could not get a match with word_item3 = {}".format(prefix, word_item3)

                # just a consistency check
                if not got_milk:
                    print prefix + "Still don't know what type of milk it is! Reseting to default." + bcolors.ENDC
                    self.breakfastMilk.current = knowledge.default_milk

            else:
                self.breakfastMilk.current = knowledge.default_milk
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
    def __init__(self, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
        smach.State.__init__(self, outcomes=['done'])

        self.breakfastCereal = breakfastCerealDes
        self.breakfastFruit = breakfastFruitDes
        self.breakfastMilk = breakfastMilkDes

    def execute(self, userdata):
        self.breakfastCereal.current = knowledge.default_cereal
        self.breakfastMilk.current   = knowledge.default_milk
        self.breakfastFruit.current  = knowledge.default_fruit

        return 'done'

# ----------------------------------------------------------------------------------------------------

class ConfirmOrder(smach.State):
    def __init__(self, robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
        smach.State.__init__(self, outcomes=['done'])

        self.robot = robot
        self.breakfastCereal = breakfastCerealDes
        self.breakfastFruit = breakfastFruitDes
        self.breakfastMilk = breakfastMilkDes

    def execute(self, userdata):
        print prefix + bcolors.OKBLUE + "ConfirmOrder" + bcolors.ENDC

        self.robot.speech.speak("I understand you want a " +  self.breakfastFruit.resolve() + " and " + self.breakfastCereal.resolve() + " with " + self.breakfastMilk.resolve() + ". Is that correct?", block=True)

        return 'done'

# ----------------------------------------------------------------------------------------------------

# class RepeatOrderToPerson(smach.State):
#     def __init__(self, robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes):
#         smach.State.__init__(self, outcomes=['done'])

#         self.robot = robot
#         self.breakfastCereal = breakfastCerealDes
#         self.breakfastFruit = breakfastFruitDes
#         self.breakfastMilk = breakfastMilkDes

#     def execute(self, userdata):
#         print prefix + bcolors.OKBLUE + "RepeatOrderToPerson" + bcolors.ENDC

#         self.robot.speech.speak("I will get you a " +  self.breakfastFruit.resolve() + " and " + self.breakfastCereal.resolve() + " with " + self.breakfastMilk.resolve() + ". Breakfast will be served in the dining room.", block=False)

#         return 'done'

# ----------------------------------------------------------------------------------------------------

class Counter(smach.State):
    def __init__(self, counter, limit):
        smach.State.__init__(self, outcomes=['counted', 'limit_reached'])
        self.limit = limit

        check_resolve_type(counter,int)
        self.counter = counter

    def execute(self, userdata):
        count = self.counter.resolve()

        if count >= self.limit:
            self.counter.current = 0
            return 'limit_reached'
        else:
            self.counter.current += 1
            return 'counted'


# ----------------------------------------------------------------------------------------------------


class CancelHeadGoals(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata):
        print prefix + bcolors.OKBLUE + "CancelHeadGoals" + bcolors.ENDC

        self.robot.head.cancel_goal()

        return 'done'


# ----------------------------------------------------------------------------------------------------


class LookAtBedTop(smach.State):
    def __init__(self, robot, entity_id):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot
        self.entity = self.robot.ed.get_entity(id=entity_id)

    def execute(self, userdata):
        print prefix + bcolors.OKBLUE + "LookAtBedTop" + bcolors.ENDC

        # set robots pose
        # self.robot.spindle.high()
        self.robot.head.cancel_goal()

        # TODO maybe look around a bit to make sure the vision covers the whole bed top

        # look at bed top
        headGoal = msgs.PointStamped(x=self.entity.pose.position.x, y=self.entity.pose.position.y, z=self.entity.pose.position.z+self.entity.z_max, frame_id="/map")
        self.robot.head.look_at_point(point_stamped=headGoal, end_time=0, timeout=4)

        return 'succeeded'


# ----------------------------------------------------------------------------------------------------

class LookIfSomethingsThere(smach.State):
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
                self.robot.ed.configure_kinect_segmentation(continuous=False)
                return 'there'
            else:
                rospy.sleep(self.sleep)

        return 'not_there'

# ----------------------------------------------------------------------------------------------------

class Evaluate(smach.State):
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

class addPositive(smach.State):
    def __init__(self, results_designator, item_designator):
        smach.State.__init__(self, outcomes=['done'])
        self.results = results_designator
        self.item = item_designator

    def execute(self, userdata):
        print self.item.resolve()
        self.results.current[self.item.resolve()] = True
        return "done"


# ----------------------------------------------------------------------------------------------------

class SelectItem(smach.State):
    def __init__(self, robot, options, asked_items, generic_item, specific_item, item_nav_goal, item_lookat_goal):
        smach.State.__init__(self, outcomes=['selected', 'all_done'])
        self.robot = robot
        self.options = options
        self.asked_items_des = asked_items
        self.count = len(self.options)
        self.current = 0
        self.generic_item = generic_item
        self.specific_item = specific_item
        self.nav_goal = item_nav_goal
        self.lookat_goal = item_lookat_goal

    def execute(self, userdata):
        if self.current == self.count:
            self.current = 0
            return 'all_done'
        else:
            self.generic_item.current = self.options[self.current]

            asked_items = [d.resolve() for d in self.asked_items_des]
            category_items = [i['name'] for i in knowledge_objs if 'sub-category' in i and i['sub-category']==self.generic_item.resolve()]

            print "asked items: {}".format(asked_items)
            print "category items: {}".format(category_items)

            self.specific_item.current = list(set(category_items).intersection(asked_items))[0]

            self.robot.speech.speak("I will get your "+self.generic_item.resolve()+" now.", block=False)

            self.nav_goal.current = {
                                        EdEntityDesignator(self.robot, id=knowledge.item_nav_goal['in_front_of_'+self.generic_item.resolve()]) : "in_front_of",
                                        EdEntityDesignator(self.robot, id=knowledge.item_nav_goal['in']) : "in"
                                    }

            self.lookat_goal.current = EdEntityDesignator(self.robot, id=knowledge.item_nav_goal['lookat_'+self.generic_item.resolve()])
        
        self.current += 1
        return 'selected'

# ----------------------------------------------------------------------------------------------------
# TODO maybe store recognized objects for future use?

class FindItem(smach.State):
    def __init__(self, robot, sensor_range, type_des, result_des, on_object_des=None):
        smach.State.__init__(self, outcomes=['item_found', 'not_found'])
        self.robot = robot
        self.sensor_range = sensor_range
        self.on_object_des = on_object_des
        self.result_type_des = type_des
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

        print "look at table, point_stamped = ", point_stamped

        self.robot.head.look_at_point(point_stamped)
        rospy.sleep(rospy.Duration(2.0))

        entity_ids = self.robot.ed.segment_kinect(max_sensor_range = self.sensor_range)
        print "entity_ids 1: ", entity_ids

        filtered_ids = []
        for entity_id in entity_ids:
            e = self.robot.ed.get_entity(entity_id)

            if onTopOff(e, self.on_object):
                print "id is on top of object"
            else:
                print "id is NOT on top of object"

            if e and self.on_object and not e.type:# and onTopOff(e, self.on_object):
                print "ja, toegevoegd"
                filtered_ids.append(e.id)

        print "filtered_ids =", filtered_ids
        print "self.items_were_looking_for =", self.items_were_looking_for
        
        entity_types = self.robot.ed.classify(ids=filtered_ids, types=self.items_were_looking_for)


        print "I found the following items: {}".format(entity_types)

        self.robot.head.cancel_goal()

        ###############
        #print "result designator = ". self.result_des


        # hack to check grab state.
        if len(filtered_ids)>0:
            self.result_des.current = self.robot.ed.get_entity(filtered_ids[0])
            return 'item_found'
        ##############

        for i in range(len(filtered_ids)):
            if entity_types[i] == self.result_type:
                self.result_des.current = self.robot.ed.get_entity(filtered_ids[i])
                return 'item_found'

        # if wanted item is not found then ..
        found_milk      = list(set(entity_types).intersection(names_milk))
        found_cereal    = list(set(entity_types).intersection(names_cereal))
        found_fruit     = list(set(entity_types).intersection(names_fruit))

        print "I found the following milk, cereal and fruit items: {}".format(found_milk+found_cereal+found_fruit)

        if len(found_milk) > 0 and self.result_type in names_milk:
            self.result_des.current = self.robot.ed.get_entity(found_milk[0])
            return 'item_found'
        elif len(found_cereal) > 0 and self.result_type in names_cereal:
            self.result_des.current = self.robot.ed.get_entity(found_cereal[0])
            return 'item_found'
        elif len(found_fruit) > 0 and self.result_type in names_fruit:
            self.result_des.current = self.robot.ed.get_entity(found_fruit[0])
            return 'item_found'

        # TODO: maybe go to another position to look again?

        return 'not_found'

# ----------------------------------------------------------------------------------------------------

class ScanTableTop(smach.State):
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

class EmptySpotDesignator(Designator):
    """Designates an empty spot on the empty placement-shelve.
    It does this by queying ED for entities that occupy some space.
        If the result is no entities, then we found an open spot."""
    def __init__(self, robot, closet_designator):
        super(EmptySpotDesignator, self).__init__(resolve_type=gm.PoseStamped)
        self.robot = robot
        self.closet_designator = closet_designator
        self._edge_distance = 0.1                   # Distance to table edge
        self._spacing = 0.15

    def resolve(self):
        closet = self.closet_designator.resolve()

        # points_of_interest = []
        points_of_interest = self.determinePointsOfInterest(closet)

        def is_poi_occupied(poi):
            entities_at_poi = self.robot.ed.get_entities(center_point=poi, radius=self._spacing)
            return not any(entities_at_poi)

        open_POIs = filter(is_poi_occupied, points_of_interest)

        def distance_to_poi_area(poi):
            #Derived from navigate_to_place
            radius = math.hypot(self.robot.grasp_offset.x, self.robot.grasp_offset.y)
            x = poi.point.x
            y = poi.point.y
            ro = "(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, radius+0.075)
            ri = "(x-%f)^2+(y-%f)^2 > %f^2"%(x, y, radius-0.075)
            pos_constraint = PositionConstraint(constraint=ri+" and "+ro, frame="/map")

            plan_to_poi = self.robot.base.global_planner.getPlan(pos_constraint)

            distance = 10**10 #Just a really really big number for empty plans so they seem far away and are thus unfavorable
            if plan_to_poi:
                distance = len(plan_to_poi)
            print "Distance: %s"%distance
            return distance

        if any(open_POIs):
            best_poi = min(open_POIs, key=distance_to_poi_area)
            placement = msgs.PoseStamped(pointstamped=best_poi)
            rospy.loginfo("Placement = {0}".format(placement).replace('\n', ' '))
            return placement
        else:
            rospy.logerr("Could not find an empty spot")
            return None


    def determinePointsOfInterest(self, e):

        points = []

        x = e.pose.position.x
        y = e.pose.position.y

        if len(e.convex_hull) == 0:
            rospy.logerr('Entity: {0} has an empty convex hull'.format(e.id))
            return []

        ''' Convert convex hull to map frame '''
        center_pose = poseMsgToKdlFrame(e.pose)
        ch = []
        for point in e.convex_hull:
            p = pointMsgToKdlVector(point)
            p = center_pose * p
            ch.append(p)

        ''' Loop over hulls '''
        ch.append(ch[0])
        for i in xrange(len(ch) - 1):
                dx = ch[i+1].x() - ch[i].x()
                dy = ch[i+1].y() - ch[i].y()
                length = math.hypot(dx, dy)

                d = self._edge_distance
                while d < (length-self._edge_distance):

                    ''' Point on edge '''
                    xs = ch[i].x() + d/length*dx
                    ys = ch[i].y() + d/length*dy

                    ''' Shift point inwards and fill message'''
                    ps = gm.PointStamped()
                    ps.header.frame_id = "/map"
                    ps.point.x = xs - dy/length * self._edge_distance
                    ps.point.y = ys + dx/length * self._edge_distance
                    ps.point.z = e.pose.position.z + e.z_max
                    points.append(ps)

                    # ToDo: check if still within hull???
                    d += self._spacing

        return points

# ----------------------------------------------------------------------------------------------------

class NavigateToSymbolic(states.NavigateToSymbolic):
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