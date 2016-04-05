#!/usr/bin/python

import re
import random

#import data
from robocup_knowledge import load_knowledge
data = load_knowledge('challenge_open')

test_spec       = data.spec1_amigo_task
test_choices    = data.choices1_amigo_task

# Pick random group if available
while re.search('\([^\)]+\)', test_spec):
    options = re.findall('\([^\(\)]+\)', test_spec)
    for option in options:
        test_spec = test_spec.replace(option,random.choice(option[1:-1].split("|")), 1)

# Fetch all the residual choices
choices = re.findall("<([^<>]+)>", test_spec)

# Parse the choices in the ending result :)
for c in choices:
    for k,v in test_choices.iteritems():
        if k == c:
            value = random.choice(v)

            test_spec = test_spec.replace("<%s>"%c, value)

# Check if result is clean
if re.match(".*[<>\(\)\[\]]+.*", test_spec):
    print "Not all choices could be resolved in the specification, residual result: '%s'"
else:
    print "Question to ask amigo:" 
    print test_spec
    print "\n"

print "If question is .. needs your help .. or .. go back to ... then next answer is valid \n"

test_spec       = data.spec2_amigo_task_followup
test_choices    = data.choices2_amigo_task_followup

# Pick random group if available
while re.search('\([^\)]+\)', test_spec):
    options = re.findall('\([^\(\)]+\)', test_spec)
    for option in options:
        test_spec = test_spec.replace(option,random.choice(option[1:-1].split("|")), 1)

# Fetch all the residual choices
choices = re.findall("<([^<>]+)>", test_spec)

# Parse the choices in the ending result :)
for c in choices:
    for k,v in test_choices.iteritems():
        if k == c:
            value = random.choice(v)

            test_spec = test_spec.replace("<%s>"%c, value)

# Check if result is clean
if re.match(".*[<>\(\)\[\]]+.*", test_spec):
    print "Not all choices could be resolved in the specification, residual result: '%s'"
else:
    print "Answer to Amigo:"
    print test_spec
