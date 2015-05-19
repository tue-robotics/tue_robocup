#!/usr/bin/env python


from datetime import datetime
from datetime import date

# print "a\n"
# print datetime.now()
# print "b\n"
# print date.today()
# print "c\n"
# print datetime.month()
# print "d\n"
# print datetime.date()

time      = "It is %s"%datetime.now().strftime("%I %M %p")
print time
date2 = "It is %s"%datetime.now().strftime("%A")
print date2
# today     =
# tomorrow  =
# day_month = "It is the %s of %s"%datetime.now().strftime("%b"),datetime.now().strftime("%d") 
# print day_month
day_month = "It is the %s "%datetime.now().strftime("%b")
print day_month
day_month = "It is the %s "%datetime.now().strftime("%d")
print day_month
day_week  = "It is %s"%datetime.now().strftime("%A") 
print day_week


# speak_special = {
#         "your name":"my name is amigo",
#         "the name of your team": "my team's name is tech united",
#         "the time": time,
#         "what time is it": time,
#         "tell the date": date,
#         "what day is today": today,
#         "what day is tomorrow":tomorrow,
#         "tell the day of the month": day_month,
#         "tell the day of the week": day_week
# }


# say_result = replace_word(result,"me","you")

# print "say_result = ", say_result