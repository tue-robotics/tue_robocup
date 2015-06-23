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

import os
from PIL import Image
import cStringIO as StringIO
import numpy as np

from color_analysis import analyze

def save_entity_image_to_file(world_model_ed, entityID, colorname):
    file_name = "images/%s_%s.jpg"%(entityID, colorname)
    if "/" in file_name and not os.path.exists(os.path.dirname(file_name)):
        os.makedirs(os.path.dirname(file_name))

    # ed request
    info = world_model_ed.get_entity_info(entityID)

    try:
        byte_array = bytearray(info.measurement_image)
        f = open(file_name, 'wb')
        f.write(byte_array)
        f.close()
        del f

        # stream = StringIO.StringIO(byte_array)
        
        # import ipdb; ipdb.set_trace()
        image = Image.open(file_name)

        image_data = np.asarray(image)
        image_data_bw = image_data.max(axis=2)
        non_empty_columns = np.where(image_data_bw.max(axis=0)>0)[0]
        non_empty_rows = np.where(image_data_bw.max(axis=1)>0)[0]
        cropBox = (min(non_empty_rows), max(non_empty_rows), min(non_empty_columns), max(non_empty_columns))

        image_data_new = image_data[cropBox[0]:cropBox[1]+1, cropBox[2]:cropBox[3]+1 , :]

        cropped_image = Image.fromarray(image_data_new)
        cropped_file_name = "images/%s_%s_cropped.jpg"%(entityID, colorname)
        cropped_image.save(cropped_file_name)
    except Exception, e:
        rospy.logerr("Could not save image for {} because {}".format(entityID, e))
        return None

    return file_name


class BottleDescription(object):
    def __init__(self, size=None, color=None, label=None, height=None, position=None):
        self.size = size
        self.color = color
        self.label = label
        self.height = height
        self.position = position

        self.height_description = None
        self.position_description = None

    def __eq__(self, other):
        """Check equality of self against other
        >>> assert BottleDescription() == BottleDescription()
        >>> assert BottleDescription(size="big") == BottleDescription(size="big")
        >>> assert BottleDescription(label="ibuprofen") == BottleDescription(label="ibuprofen")
        >>> assert BottleDescription(size="big", color="red") == BottleDescription(size="big", color="red")
        
        >>> assert BottleDescription(label="ibuprofen") != BottleDescription(size="small")
        >>> assert BottleDescription(size="big", color="red") != BottleDescription(size="big", color="blue")"""
        return self.size == other.size and self.color == other.color and self.label == other.label

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
    size_description = None
    try:
        height = abs(entity.z_min - entity.z_max)
        if height < 0.05:
            size_description = "small"
        elif 0.05 <= height < 0.10:
            size_description = "normal sized"
        elif 0.10 <= height:
            size_description = "big"
        rospy.loginfo("Height of object {0} is {1} so classifying as {2}".format(entity.id, height, size_description))
    except:
        pass

    return size_description, height

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
        # self.robot.head.reset()
        # import ipdb; ipdb.set_trace()
        bottles = self.bottle_collection_designator.resolve()
        if not bottles:
            return "failed"
        
        #Lock the IDs in the world model so that they keep their IDs persistently
        bottle_ids = [bottle.id for bottle in bottles]
        rospy.logwarn("Locking IDs {}".format(bottle_ids))
        self.robot.ed.lock_entities(bottle_ids, [])
        

        #TODO: Sort bottles by their Y-coord wrt base_link. We go from large to small, so the leftmost if first
        bottle_to_y_dict = {}
        for bottle in bottles:
            in_map = geom.PointStamped(point=bottle.pose.position, frame_id=bottle.id)
            in_base_link = transformations.tf_transform(in_map, "/map", "/"+self.robot.robot_name+"/base_link", self.robot.tf_listener)
            bottle_to_y_dict[bottle] = in_base_link.y

        sorted_bottles = sorted(bottle_to_y_dict.items(), key=operator.itemgetter(1))  # Sort dict by value, i.e. the bottle's Y

        descriptions = OrderedDict()
        for bottle, y in sorted_bottles:
            descriptions[bottle] = self.describe_bottle((bottle, y))

        self.robot.speech.speak("I see {0} bottles, which do you want?".format(len(descriptions)))
        self.robot.speech.speak("From left to right, I have a")
        for bottle, desc in descriptions.iteritems():

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
        choices = {}
        if colors:
            choices["color"] = colors
        if sizes:
            choices["size"] = sizes
        if labels:
            choices["label"] = labels
        rospy.loginfo("Choices are {}".format(choices))

        # import ipdb; ipdb.set_trace()
        if sizes and colors and labels:
            self.spec_designator.current = "Bring me the <size>, <color> bottle labeled <label>"
        elif sizes and colors:
            self.spec_designator.current = "Bring me the <size>, <color> bottle"
        elif colors and labels:
            self.spec_designator.current = "Bring me the <color> bottle labeled <label>"
        elif sizes and labels:
            self.spec_designator.current = "Bring me the <size> bottle labeled <label>"
        elif sizes:
            self.spec_designator.current = "Bring me the <size> bottle"
        elif colors:
            self.spec_designator.current = "Bring me the <color> bottle"
        elif labels:
            self.spec_designator.current = "Bring me the bottle labeled <label>"
        
        self.choices_designator.current = choices

        self.bottle_desc_mapping_designator.current = descriptions

        return "succeeded"

    def describe_bottle(self, bottle_at_y):
        bottle_entity, y = bottle_at_y

        # import ipdb; ipdb.set_trace()
        most_probable_color = get_entity_color(bottle_entity)
        try:
            filename = save_entity_image_to_file(self.robot.ed, bottle_entity.id, most_probable_color)
            
            try:
                most_probable_color = analyze(Image.open(filename))
            except Exception, e:
                rospy.logwarn("Could not get the dominant color in {}".format(filename))

            rospy.loginfo("{} has color {}".format(filename, most_probable_color))
        except Exception, e:
            rospy.logwarn("Could not save image of entity {}: {}".format(bottle_entity.id, e))

        size_description, height = get_entity_size(bottle_entity)

        return BottleDescription(   size=size_description,
                                    color=most_probable_color,
                                    label=None,
                                    position=y,
                                    height=height)

if __name__ == "__main__":
    import doctest
    doctest.testmod()