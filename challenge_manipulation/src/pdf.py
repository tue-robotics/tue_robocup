#!/usr/bin/python
import rospy

from xhtml2pdf import pisa

import os

from robot_skills import world_model_ed
from PIL import Image
import cStringIO as StringIO
import numpy as np
import datetime

report = '''
# H1

## H2

Test list

- item 1
- item 2
- item 3
    '''


def html2pdf(sourceHtml, outputFilename):
    with open(outputFilename, "w+b") as resultFile:
        pisaStatus = pisa.CreatePDF(
                                    sourceHtml,
                                    dest=resultFile)
    return pisaStatus.err


def save_entity_image_to_file(world_model_ed, entityID):
    # ed request
    info = world_model_ed.get_entity_info(entityID)

    if len(info.measurement_image_unmasked) == 0:
        rospy.logerr("Received empty image for %s from ED.", entityID)
        return None

    try:
        byte_array = bytearray(info.measurement_image_unmasked)
        stream = StringIO.StringIO(byte_array)
        image = Image.open(stream)
    except Exception as e:
        rospy.logerr("Failed to load image from entity %s", entityID)
        rospy.logerr("Failed to load image ... Try installing the latest version of PILLOW: sudo apt-get install python-imaging")
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

    file_name = "images/%s.jpg"%entityID

    if "/" in file_name and not os.path.exists(os.path.dirname(file_name)):
        os.makedirs(os.path.dirname(file_name))

    cropped_image.save(file_name)
    rospy.loginfo("Saving image to '%s'" % file_name)

    return file_name


def entities_to_pdf(world_model_ed, entities, name, directory = "/home/amigo/usb"):

    html = "<html>"
    html += "<head>"
    html += "<title>%s</title>"%name
    html += "</head>"

    html += "<body>"
    html += "<h1>%s</h1>"%name

    for entity in entities:
        if len(entity.id) == 32 and entity.type != "":
            image = save_entity_image_to_file(world_model_ed, entity.id)
            print "Created entry for %s (%s)"%(entity.id, entity.type)
            html += "<table border='1'><tr>"
            if image:
                html += "<td><img src='%s' alt='%s' /></td>"%(image, entity.id)
            else:
                html += "<td>!! NO IMAGE FOR '%s' !!</td>"%entity.id
            html += "<td><center>"
            html += "<h2>%s</h2>"%entity.id
            html += "<p><b>Type: </b>%s</p>"%entity.type
            html += "<p><b>Position (x,y,z): </b>(%.2f,%.2f,%.2f)</p>"%(entity.pose.frame.p.x(), entity.frame.p.y(), entity.frame.p.z())
            html += "</center></td>"
            html += "</tr></table>"
            html += "<br />"

    html += "</body>"
    html += "</html>"

    date_str = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M")
    filename = "%s_%s.pdf"%(name, date_str)

    try:
        html2pdf(html, "%s/%s"%(directory, filename))
    except IOError, ioerror:
        rospy.logerr(ioerror)
        rospy.logwarn("Writing to local file instead")
        html2pdf(html, "%s"%filename)

    html2pdf(html, "%s/%s"%(os.path.expanduser("~"), filename))

if __name__ == '__main__':
    rospy.init_node("testpdf")
    pisa.showLogging()

    ed = world_model_ed.ED("amigo", "nbanana");
    entities_to_pdf(ed, ed.get_entities(), "all_entities")
