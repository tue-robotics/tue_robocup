#! /usr/bin/python

import os
import sys
import rospy
import re, random

from robocup_knowledge import load_knowledge

import action_server
from action_server.command_center import CommandCenter

# ----------------------------------------------------------------------------------------------------

def main():

    num_sentences = 1
    if len(sys.argv) > 1:
        num_sentences = int(sys.argv[1])

    command_center = CommandCenter()
    challenge_knowledge = load_knowledge('challenge_gpsr')
    command_center.set_grammar(os.path.dirname(sys.argv[0]) + "/grammar.fcfg", challenge_knowledge)

    req_spec = command_center.command_recognizer.grammar_string
    req_choices = command_center.command_recognizer.choices

    for i in range(0, num_sentences):

        # Copy request
        example = "(%s)" % req_spec

        # Pick random group if available
        while re.search('\([^\)]+\)', example):
            options = re.findall('\([^\(\)]+\)', example)
            for option in options:
                example = example.replace(option, random.choice(option[1:-1].split("|")), 1)

        # Fetch all the residual choices
        choices = re.findall("<([^<>]+)>", example)

        # Parse the choices in the ending result :)
        for c in choices:
            for req_c in req_choices:
                if req_c == c:
                    value = random.choice(req_choices[req_c])
                    example = example.replace("<%s>"%c, value)

        print example

# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())
