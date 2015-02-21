#!/usr/bin/python
import rospy

from markdown import markdown
from xhtml2pdf import pisa

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

def save_entity_image_to_file(entityID):
    #TODO: Get measurements of entityID and save to image
    return entityID+".png"
    # return "/home/loy/Pictures/screenshot.png"

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
