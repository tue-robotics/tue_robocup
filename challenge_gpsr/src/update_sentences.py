#!/usr/bin/env python
from __future__ import print_function
import logging
import os
from robocup_knowledge import knowledge_loader
challenge_speech_recognition_data = knowledge_loader.load_knowledge("common")

logging.basicConfig()
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)


def openfile(name):
    path = os.path.join(os.path.dirname(__file__), 'gpsr_command_generator', name)
    return open(path, 'w')


logger.info('writing names')
names = challenge_speech_recognition_data.names
with openfile('names.txt') as f:
    print('# list of person names', file=f)
    for name in sorted(names):
        print(name.capitalize(), file=f)

logger.info('writing items')
items = challenge_speech_recognition_data.objects
with openfile('items.txt') as itemsf, openfile('item_categories.txt') as item_categoriesf:
    print('# list of items/objects', file=itemsf)
    print('# categories for the items/objects (in items.txt)', file=item_categoriesf)
    for item in sorted(sorted(items, key=lambda k: k['name']), key=lambda k: k['category']):
        print(item['name'].capitalize(), file=itemsf)
        print(item['category'].capitalize().replace('_', ' '), file=item_categoriesf)

logger.info('writing locations')
locations = challenge_speech_recognition_data.locations
with openfile('locations.txt') as locationsf, openfile('location_categories.txt') as location_categoriesf:
    print('# list of locations', file=locationsf);
    print('# categories for the list of locations (in locations.txt)', file=location_categoriesf)
    for location in sorted(sorted(locations, key=lambda k: k['name']), key=lambda k: k['location_category']):
        print(location['name'], file=locationsf)
        print(location['location_category'], file=location_categoriesf)
