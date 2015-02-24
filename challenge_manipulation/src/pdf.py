#!/usr/bin/python
import rospy

from markdown import markdown
from xhtml2pdf import pisa

from ed_gui_server.srv import *


report = '''
# H1

## H2

Test list

- item 1
- item 2
- item 3
    '''

# ed_get_measurements = rospy.ServiceProxy('/ed/get_measurements', SimpleQuery)

def html2pdf(sourceHtml, outputFilename):
    with open(outputFilename, "w+b") as resultFile:
        pisaStatus = pisa.CreatePDF(
                                    sourceHtml,
                                    dest=resultFile)
    return pisaStatus.err

get_entity_info = rospy.ServiceProxy('/ed/gui/get_entity_info', GetEntityInfo)

def save_entity_image_to_file(entityID):

    # ed request
    info = get_entity_info(entityID)

    # write bytes to file
    file_name = entityID+".jpg"
    with open(file_name , 'wb') as f:
        f.write(info.measurement_image)

    return file_name

def items2markdown(entities):
    rospy.logdebug("TODO: Exporting PDF")

    md = ""

    fmt = "#{title}: ![{title}]({path})"
    for entity in entities:
        md += fmt.format(title=entity.type, path=save_entity_image_to_file(entity.id))
        md += "\n"

    md = md.replace("<", '').replace(">", '') #May not be needed when not using mocks

    html = markdown(md)

    pdfFilename = "manipulation_challenge.pdf"
    html2pdf(html, pdfFilename)



if __name__ == '__main__':
    pisa.showLogging()

    print report
    md = markdown(report)
    print md

    outputFilename = "test.pdf"
    html2pdf(md, outputFilename)
