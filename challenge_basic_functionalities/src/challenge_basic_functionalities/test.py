#!/usr/bin/python

# dictionary of questions and answers
QA_MAP = { "what time is it" : "time to buy a watch",
           "what is the capital of germany" : "berlin",
           "what is the heaviest animal in the world" : "Is quite ok",
           "who is the president of america" : "barack obama",
           "what is your name" : "amigo",
           "who is your example" : "erik geerts",
           "what is your motto" : "yolo",
           "which football club is the best" : "feyenoord",
           "who is the best looking person around here" : "erik geerts of course",
           "which town has been bombed" : "schijndel",
           "which person is not able to say yes" : "dirk holtz",
           "what is the capital of brazil" : "brasilia",
           "what is the oldest drug used on earth" : "alcohol",
           "in which year was robocup founded" : "nineteen ninety seven",
           "how many rings has the olympic flag" : "five",
           "what is the worlds most popular green vegetable" : "lettuce",
           "which insect has the best eyesight" : "dragonfly",
           "who lives in a pineapple under the sea" : "spongebob squarepants",
           "what is your teams name" : "tech united eindhoven",
           "what is the answer to the ultimate question about life the universe and everything" : "forty two",
           "what is the capital of poland" : "warsaw",
           "which country grows the most potatoes" : "russia",
           "which country grew the first orange" : "china",
           "how many countries are in europe" : "fifty",
           "which fish can hold objects in its tail" : "sea horse"}


# [ x for x in QA_MAP ]:
#     print x
# print len(QA_MAP)
#a=QA_MAP.keys()
for x in range(0, len(QA_MAP)):
	print "Question", 1+x,":", QA_MAP.keys()[x],"?"
	
print "hallo!"
 	