#!/usr/bin/env python
"""ROS node that listens to maluuba_ros Interpretation-messages.
The Interpretation is processed by the node when it can handle the requested action"""

import roslib
roslib.load_manifest('challenge_demo')
import os
import rospy
import yaml
from datetime import datetime, timedelta

from maluuba_ros.msg import Interpretation
from maluuba_ros.srv import Normalize, NormalizeResponse


def make_ical(startdate, summary, enddate=None, filename='reminder.ics'):
    from icalendar import Calendar, Event
    cal = Calendar()
    cal.add('prodid', '-//My calendar product//mxm.dk//')
    cal.add('version', '2.0')

    import pytz
    event = Event()
    event.add('summary', summary)
    event.add('dtstart', startdate)
              # datetime(2005,4,4,8,0,0,tzinfo=pytz.utc))
    if enddate:
        event.add('dtend', enddate)
                  # datetime(2005,4,4,10,0,0,tzinfo=pytz.utc))
    event.add('dtstamp', datetime.now())
              #datetime(2005,4,4,0,10,0,tzinfo=pytz.utc))
    event['uid'] = '20050115T101010/27346262376@mxm.dk'
    event.add('priority', 5)

    cal.add_component(event)

    f = open(filename, 'wb')
    f.write(cal.to_ical())
    f.close()


class MailInterpreter(object):

    """Interprets Maluuba EMAIL_SEND actions"""

    def __init__(self, configfile):
        self.normalizer = rospy.ServiceProxy('maluuba/normalize', Normalize)
        self.subscriber = rospy.Subscriber(
            "/maluuba/interpretations", Interpretation, self.process_interpretation)
        self.mailadress = None
        self.password = None
        self.address_book = {}

        self.configure(configfile)

    def configure(self, configfile):
        config = list(yaml.load_all(configfile))

        account = config[0]
        address_book = config[1]

        self.address_book = dict([(name, value[
                                 'email']) for name, value in address_book.iteritems()])

        self.mailadress = account["sender"]
        self.password = account["password"]

    def sendmail(self, subject, message, receivers, attachments={}):
        # a list as default arg can get bad, as they are mutable
        """Based off http://stackoverflow.com/questions/6270782/sending-email-with-python"""
        # Import smtplib for the actual sending function
        import smtplib

        # Import the email modules we'll need
        from email.MIMEMultipart import MIMEMultipart
        from email.MIMEBase import MIMEBase
        from email.MIMEText import MIMEText
        from email import Encoders
        # Create a text/plain message

        msg = MIMEMultipart()
        msg.attach(MIMEText(message))
        msg['Subject'] = subject
        msg['From'] = self.mailadress
        msg['To'] = receivers[0]

        s = smtplib.SMTP('smtp.gmail.com', 587)
        s.starttls()
        s.login(self.mailadress, self.password)

        for f, (mimeapp, mimecontent) in attachments.iteritems():
            part = MIMEBase(
                mimeapp, mimecontent)  # MIMEBase('application', "octet-stream")
            part.set_payload(open(f, "rb").read())
            Encoders.encode_base64(part)
            part.add_header(
                'Content-Disposition', 'attachment; filename="%s"' % os.path.basename(f))
            msg.attach(part)

        s.sendmail(self.mailadress, receivers, msg.as_string())
        rospy.loginfo("Sending {0}, {1}, {2}".format(
            subject, message, receivers))
        s.quit()

    @staticmethod
    def process_time(dt):
        from dateutil import tz
        from_zone = tz.gettz('America/Montreal')
        to_zone = tz.gettz('Europe/Amsterdam')
        dt = dt.replace(tzinfo=from_zone)
        dt = dt.astimezone(to_zone)
        return dt


    def extract_times(self, entities, default_duration=timedelta(hours=1)):
        #import ipdb; ipdb.set_trace()
        if entities.time:
            start = datetime.fromtimestamp(entities.time)
            start = self.process_time(start)

            end = start + default_duration

            return start, end
        if entities.timeRange.start and entities.timeRange.end:
            start = datetime.fromtimestamp(entities.timeRange.start)
            end = datetime.fromtimestamp(entities.timeRange.end)
            return start, end
        if entities.dateRange.start and entities.dateRange.end:
            start = datetime.fromtimestamp(entities.dateRange.start)
            end = datetime.fromtimestamp(entities.dateRange.end)
            return start, end
        else:
            start = datetime.now()
            end = start + default_duration
            return start, end

    @staticmethod
    def datetime_as_sentence(dt):
        monthmap = {1:"January", 2:"February", 3:"March", 4:"April", 5:"May", 6:"June", 7:"July", 8:"August", 9:"September", 10:"October", 11:"November", 12:"December"}
        now = datetime.now()
        year, month, day, hour, minute = dt.year, dt.month, dt.day, dt.hour, dt.minute
        if hour > 12:
            hour -= 12
        timestr = "{m} past {h}".format(h=hour, m=minute)

        def plural(num):
            """Converts 1 to first, 2 to second etc"""
            plurals = {1:'first', 2:'second', 3:'third'}
            if num in plurals:
                return plurals[num]
            else:
                return "{0}th".format(num)

        daystr = ""
        if (year, month, day) == (now.year, now.month, now.day):
            daystr = "today"
        elif (year, month, day) == (now.year, now.month, now.day+1):
            daystr = "tomorrow"
        elif (year, month, day) == (now.year, now.month, now.day+2):
            daystr = "the day after tomorrow"    
        else:
            daystr = "{0} {1}, {2}".format(monthmap[month], plural(day), year)

        return "{0} at {1}".format(daystr, timestr)




    def process_interpretation(self, msg):
        if msg.action == "EMAIL_SEND":
            subject = msg.entities.subject if msg.entities.subject else ""
            message = msg.entities.message if msg.entities.message else ""
            receivers = set([self.address_book.get(
                contact.name.lower(), self.mailadress) for contact in msg.entities.contacts])

            self.sendmail(subject, message, list(receivers))

        elif msg.action == "REMINDER_SET":
            receivers = ["loy.vanbeek@gmail.com"]
            
            import re
            words = re.findall(r"[\w']+", msg.phrase)
            names = filter(lambda word: self.address_book.has_key(word.lower()), words) #Find names in the phrase that oalso occur in the adress book
            receivers += list(set([self.address_book.get(name.lower(), self.mailadress) for name in names]))

            message = msg.entities.message if msg.entities.message else "reminder"

            start, end = self.extract_times(msg.entities)

            make_ical(start, message, filename='reminder.ics', enddate=end)
            rospy.loginfo("Made iCAL with ({0} --> {3}), {1}. Sending to {2}".format(start, message, list(receivers), end))
            self.sendmail("Reminder", message, list(receivers),{'reminder.ics':("text", "calendar")})


if __name__ == "__main__":
    import sys
    rospy.init_node("interpret_email_send")

    try:
        interpreter = MailInterpreter(open(sys.argv[1]))
    except IOError as e:
        rospy.logerr(e)

    rospy.spin()
