#!/usr/bin/python
import rospy
import smach
import sys

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

def html2pdf(sourceHtml, outputFilename):
    with open(outputFilename, "w+b") as resultFile:
        pisaStatus = pisa.CreatePDF(
                                    sourceHtml,
                                    dest=resultFile)
    return pisaStatus.err

if __name__ == '__main__':
    pisa.showLogging()

    print report
    md = markdown(report)
    print md

    outputFilename = "test.pdf"
    html2pdf(md, outputFilename)
