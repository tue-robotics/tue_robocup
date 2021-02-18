#!/usr/bin/python

import re
import random

from robocup_knowledge import load_knowledge
data = load_knowledge('challenge_speech_recognition')

# Pick random group if available
while re.search('\([^\)]+\)', data.spec):
    options = re.findall('\([^\(\)]+\)', data.spec)
    for option in options:
        data.spec = data.spec.replace(option,random.choice(option[1:-1].split("|")), 1)

# Fetch all the residual choices
choices = re.findall("<([^<>]+)>", data.spec)

# Parse the choices in the ending result :)
for c in choices:
    for k, v in data.choices.items():
        if k == c:
            value = random.choice(v)

            data.spec = data.spec.replace("<%s>"%c, value)

# Check if result is clean
if re.match(".*[<>\(\)\[\]]+.*", data.spec):
    print("Not all choices could be resolved in the specification, residual result: '%s'")
else:
    print("Question: " + data.spec)
    print("Answer: " + data.choice_answer_mapping[data.spec])
