#! /usr/bin/python

import sys
import os
import random
from datetime import datetime

from robocup_knowledge import load_knowledge

# ----------------------------------------------------------------------------------------------------

class CommandGenerator:

    def __init__(self, gpsr_category=1):
        sentence_data_file = os.path.dirname(sys.argv[0]) + ("/../test/sentences/sentences_cat_{}.txt".format(gpsr_category))

        self.example_commands = []
        with open(sentence_data_file) as f:
            for line in f:
                line = line.strip()
                if line:
                    self.example_commands += [line]

        # Seed random
        random.seed(datetime.now())

        self.knowledge = load_knowledge('challenge_gpsr')

    def generate_command(self):
        command = random.choice(self.example_commands)
        # command = command.replace("OBJ", random.choice(self.knowledge.common.object_names))
        # command = command.replace("OBJ_CAT", random.choice(self.knowledge.common.object_categories))
        # command = command.replace("NAME", random.choice(self.knowledge.common.names))
        # command = command.replace("ROOM", random.choice(self.knowledge.rooms))
        # command = command.replace("SPECIFIC_LOC", random.choice(self.knowledge.grab_locations))
        # command = command.replace("FURNITURE", random.choice(self.knowledge.grab_locations))

        return command

# ----------------------------------------------------------------------------------------------------

def main():
    gen = CommandGenerator()
    print gen.generate_command() 

# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    sys.exit(main())