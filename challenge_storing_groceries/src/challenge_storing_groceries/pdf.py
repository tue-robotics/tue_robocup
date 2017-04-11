#!/usr/bin/python

import cStringIO as StringIO
import datetime
import numpy as np
import os
from PIL import Image
from xhtml2pdf import pisa

# ROS
import rospy
import smach

# TU/e Robotics
from robot_skills import world_model_ed

# Challenge storing groceries
from challenge_storing_groceries import config


class WritePdf(smach.State):
    """ Writes pdf based on entityinfo.

    """
    def __init__(self, robot):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=["done"])

        self._robot = robot
        self._items = {}  # Dict mapping entity id to tuples: entity, probability, and filename of images

    def execute(self, userdata):

        # # Get all entities
        # entities = self._robot.ed.get_entities()
        #
        # # Filter for the robot itself, the floor and walls and furniture objects
        # entities = [e for e in entities if self._robot.robot_name not in e.id and e.id not in ["floor", "walls"] and
        #             "furniture" not in e.super_types]

        # config.DETECTED_OBJECTS_WITH_PROBS.append((entity, e.probability))
        # ToDo: store probabilities in the world model

        # Get DETECTED_OBJECTS_WITH_PROBS, i.e., the detections resulting from inspection
        for entity, probability in config.DETECTED_OBJECTS_WITH_PROBS:
            if entity.id not in self._items:
                image = save_entity_image_to_file(world_model_ed, entity.id)
                self._items.append((entity, probability, image))

        # For now, write everything to file. ToDo: filter
        entities_to_pdf(self._items, "tech_united_eindhoven", directory="/home/amigo/usb")

        return "done"


def html2pdf(source_html, output_filename):
    """ Converts html source to a pdf

    :param source_html: source html string to a pdf file
    :param output_filename: string with output pdf file (including path and extension)
    :return: possible error messages
    """
    with open(output_filename, "w+b") as resultFile:
        pisa_status = pisa.CreatePDF(source_html, dest=resultFile)
    return pisa_status.err


def save_entity_image_to_file(world_model_ed, entity_id):
    """ Saves an image of the entity identified by entity_id to a jpg.

    :param world_model_ed: world_model_ed object (part of the robot object)
    :param entity_id: string with the entity id
    :return: filename
    """
    # ed request
    info = world_model_ed.get_entity_info(entity_id)

    if len(info.measurement_image_unmasked) == 0:
        rospy.logerr("Received empty image for %s from ED.", entity_id)
        return None

    try:
        byte_array = bytearray(info.measurement_image_unmasked)
        stream = StringIO.StringIO(byte_array)
        image = Image.open(stream)
    except Exception as e:
        rospy.logerr("Failed to load image from entity %s", entity_id)
        rospy.logerr("Failed to load image ... Try installing the latest version of PILLOW: sudo pip install -I pillow")
        rospy.logerr(e)
        return None

    try:
        image_data = np.asarray(image)
        image_data_bw = image_data.max(axis=2)
        non_empty_columns = np.where(image_data_bw.max(axis=0)>0)[0]
        non_empty_rows = np.where(image_data_bw.max(axis=1)>0)[0]
        cropBox = (min(non_empty_rows), max(non_empty_rows), min(non_empty_columns), max(non_empty_columns))

        image_data_new = image_data[cropBox[0]:cropBox[1]+1, cropBox[2]:cropBox[3]+1 , :]

        cropped_image = Image.fromarray(image_data_new)
    except:
        rospy.logerr("Could not crop image, I will use the original image as cropped image")
        cropped_image = image

    file_name = "images/%s.jpg" % entity_id

    if "/" in file_name and not os.path.exists(os.path.dirname(file_name)):
        os.makedirs(os.path.dirname(file_name))

    cropped_image.save(file_name)
    rospy.loginfo("Saving image to '%s'" % file_name)

    return file_name


def entities_to_pdf(items, name, directory="/home/amigo/usb"):
    """ Writes the entities to a pdf file

    :param items: list with tuples with entity, probability, image filename
    :param name: string with name that will be used for the resulting pdf file
    :param directory: directory in which stuff should be stored
    :return:
    """

    html = "<html>"
    html += "<head>"
    html += "<title>%s</title>" % name
    html += "</head>"

    html += "<body>"
    html += "<h1>%s</h1>" % name

    for item in items:
        entity = item[0]
        image = item[2]
        if len(entity.id) == 32 and entity.type != "":
            # image = save_entity_image_to_file(world_model_ed, entity.id)
            print "Created entry for %s (%s)" % (entity.id, entity.type)
            html += "<table border='1'><tr>"
            if image:
                html += "<td><img src='%s' alt='%s' /></td>"%(image, entity.id)
            else:
                html += "<td>!! NO IMAGE FOR '%s' !!</td>"%entity.id
            html += "<td><center>"
            html += "<h2>%s</h2>" % entity.id
            html += "<p><b>Type: </b>%s</p>" % entity.type
            html += "<p><b>Position (x,y,z): </b>(%.2f,%.2f,%.2f)</p>" % (entity._pose.p.x(),
                                                                          entity._pose.p.y(),
                                                                          entity._pose.p.z())
            html += "</center></td>"
            html += "</tr></table>"
            html += "<br />"

    html += "</body>"
    html += "</html>"

    date_str = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M")
    filename = "%s_%s.pdf" % (name, date_str)

    try:
        html2pdf(html, "%s/%s" % (directory, filename))
    except IOError, ioerror:
        rospy.logerr(ioerror)
        rospy.logwarn("Writing to local file instead")
        html2pdf(html, "%s" % filename)

    html2pdf(html, "%s/%s" % (os.path.expanduser("~"), filename))


if __name__ == '__main__':
    rospy.init_node("testpdf")
    pisa.showLogging()

    ed = world_model_ed.ED("amigo", "nbanana");
    entities_to_pdf(ed, ed.get_entities(), "all_entities")
