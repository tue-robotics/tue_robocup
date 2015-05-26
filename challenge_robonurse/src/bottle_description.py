#!/usr/bin/python

import rospy
import smach

from robot_smach_states.util.designators import check_resolve_type
from ed.msg import EntityInfo
from robot_skills.util import msg_constructors as geom
from collections import OrderedDict
import operator
from robot_skills.util import transformations
import random


class BottleDescription(object):
    def __init__(self, size=None, color=None, label=None):
        self.size = size
        self.color = color
        self.label = label

    def __eq__(self, other):
        """Check equality of self against other
        >>> assert BottleDescription() == BottleDescription()
        >>> assert BottleDescription(size="big") == BottleDescription(size="big")
        >>> assert BottleDescription(label="ibuprofen") == BottleDescription(label="ibuprofen")
        >>> assert BottleDescription(size="big", color="red") == BottleDescription(size="big", color="red")
        
        >>> assert BottleDescription(label="ibuprofen") != BottleDescription(size="small")
        >>> assert BottleDescription(size="big", color="red") != BottleDescription(size="big", color="blue")"""
        return self.__dict__ == other.__dict__  # An object's values are stored in its __dict__, so you can compare those. 

    def __repr__(self):
        return "BottleDescription(size={size}, color={color}, label={label})".format(**self.__dict__)


def get_entity_color(entity):
        try:
            return max(entity.data['perception_result']['color_matcher']['colors'], key=lambda d: d['value'])['name']
        except KeyError, ke:
            rospy.logwarn("Entity {0} has no key {1}".format(entity.id, ke))
            return None
        except TypeError, te:
            rospy.logwarn("{1} for Entity {0}".format(entity.id, te))
            return None


def get_entity_size(entity):
    size = None
    try:
        height = abs(entity.z_min - entity.z_max)
        if height < 0.05:
            size = "small"
        elif 0.05 <= height < 0.10:
            size = "normal sized"
        elif 0.10 <= height:
            size = "big"
        rospy.loginfo("Height of object {0} is {1} so classifying as {2}".format(entity.id, height, size))
    except:
        pass

    return size

class DescribeBottles(smach.State):
    def __init__(self, robot, bottle_collection_designator, spec_designator, choices_designator, bottle_desc_mapping_designator):
        """
        @param robot the robot to run this with
        @bottle_collection_designator designates a bunch of bottles/entities
        @param spec_designator based on the descriptions read aloud by the robot, a spec for speech interpretation is created and stored in this VariableDesignator
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.robot = robot
        check_resolve_type(bottle_collection_designator, [EntityInfo])
        self.bottle_collection_designator = bottle_collection_designator

        self.spec_designator = spec_designator
        self.choices_designator = choices_designator
        self.bottle_desc_mapping_designator = bottle_desc_mapping_designator

    def execute(self, userdata=None):
        self.robot.head.reset()
        # import ipdb; ipdb.set_trace()
        bottles = self.bottle_collection_designator.resolve()
        
        #Lock the IDs in the world model so that they keep their IDs persistently
        bottle_ids = [bottle.id for bottle in bottles]
        rospy.logwarn("Locking IDs {}".format(bottle_ids))
        self.robot.ed.lock_entities(bottle_ids, [])
        
        if not bottles:
            return "failed"

        #TODO: Sort bottles by their Y-coord wrt base_link. We go from large to small, so the leftmost if first
        bottle_to_y_dict = {}
        for bottle in bottles:
            in_map = geom.PointStamped(point=bottle.pose.position, frame_id=bottle.id)
            in_base_link = transformations.tf_transform(in_map, "/map", "/"+self.robot.robot_name+"/base_link", self.robot.tf_listener)
            bottle_to_y_dict[bottle] = in_base_link.y

        sorted_bottles = sorted(bottle_to_y_dict.items(), key=operator.itemgetter(1))  # Sort dict by value, i.e. the bottle's Y

        descriptions = OrderedDict()
        for bottle_at_y in sorted_bottles:
            descriptions[bottle_at_y] = self.describe_bottle(bottle_at_y)

        self.robot.speech.speak("I see {0} bottles, which do you want?".format(len(descriptions)))
        self.robot.speech.speak("From left to right, I have a")
        for (bottle, y), desc in descriptions.iteritems():

            desc_sentence = ""
            if desc.size and desc.color and desc.label:
                desc_sentence = "a {size}, {color} one labeled {label}".format(size=desc.size, color=desc.color, label=desc.label)
            elif desc.size and desc.color:
                desc_sentence = "a {size}, {color} one".format(size=desc.size, color=desc.color)
            elif desc.color and desc.label:
                desc_sentence = "a {color} one labeled {label}".format(label=desc.label, color=desc.color)
            elif desc.size and desc.label:
                desc_sentence = "a {size} one labeled {label}".format(label=desc.label, size=desc.size)
            elif desc.size:
                desc_sentence = "a {size} one".format(size=desc.size)
            elif desc.color:
                desc_sentence = "a {color} one".format(color=desc.color)
            elif desc.label:
                desc_sentence += "one labeled {label}".format(label=desc.label)
            
            rospy.loginfo("Description for {0} = {1} ({2})".format(bottle.id, desc_sentence, desc))
            self.robot.speech.speak(desc_sentence)
        self.robot.speech.speak("Which do you want?")

        colors = set([desc.color for desc in descriptions.values() if desc.color])
        sizes = set([desc.size for desc in descriptions.values() if desc.size])
        labels = set([desc.label for desc in descriptions.values() if desc.label])
        choices = {"color": colors, "size": sizes, "label": labels}

        rospy.loginfo("Choices are {}".format(choices))

        # import ipdb; ipdb.set_trace()
        self.spec_designator.current = "Give me the <size> <color> bottle labeled <label>"  # TODO: allow more sentences
        self.choices_designator.current = choices

        self.bottle_desc_mapping_designator.current = descriptions

        return "succeeded"

    def describe_bottle(self, bottle_at_y):
        bottle_entity, y = bottle_at_y

        # import ipdb; ipdb.set_trace()
        most_probable_color = get_entity_color(bottle_entity)
        size = get_entity_size(bottle_entity)

        return BottleDescription(   size=size,
                                    color=most_probable_color,
                                    label=random.choice(["aspirin", "ibuprofen", None]))

if __name__ == "__main__":
    import doctest
    doctest.testmod()