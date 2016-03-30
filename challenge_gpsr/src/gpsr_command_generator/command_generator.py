# Software License Agreement (BSD License)
#
# Copyright (c) 2009-2010, Tijn van der Zant and Komei Sugiura.
# Copyright (c) 2010-2013, Dirk Holz and Luca Iocchi.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# imports
import random
import sys
import copy

# read the locations, objects and sentences files
# and clean up the lists from the files
locations       = []
location_categories = []
items           = []
item_categories = []
cat1Sentences   = []
cat2Sentences   = []
cat3Situations  = []
names           = []
# get rid of empty lines and do not use anything that starts with a '#'
for loc in [location.strip('\n') for location in open('locations.txt', 'r').readlines()]:
    if loc != '':
        if loc[0] != '#':
            locations.append(loc)
for loc_cat in [location_category.strip('\n') for location_category in open('location_categories.txt', 'r').readlines()]:
    if loc_cat != '':
        if loc_cat[0] != '#':
            location_categories.append(loc_cat)
for it  in [item.strip('\n')     for item     in open('items.txt',   'r').readlines()]:
    if it  != '':
        if it[0] != '#':
            items.append(it)
for item_cat in [item_category.strip('\n') for item_category in open('item_categories.txt', 'r').readlines()]:
    if item_cat != '':
        if item_cat[0] != '#':
            item_categories.append(item_cat)
for sentence in [str(sent).strip('\n') for sent in open('cat1Sentences.txt' , 'r').readlines()]:
    if sentence != '':
        if sentence[0] != '#':
            cat1Sentences.append(sentence)
for sentence in [str(sent).strip('\n') for sent in open('cat2Sentences.txt' , 'r').readlines()]:
    if sentence != '':
        if sentence[0] != '#':
            cat2Sentences.append(sentence)
situations  = []
questions   = []
for sit in [str(sent).strip('\n') for sent in open('cat3Situations.txt' , 'r').readlines()]:
    if sit != '':
        if sit[0] != '#':
            if sit.split()[0] == 'situation:':
                situations.append( sit )
            if sit.split()[0] == 'question:':
                questions.append( sit )
cat3Situations = zip( situations, questions )
for name in [nam.strip('\n')     for nam     in open('names.txt',   'r').readlines()]:
    if name  != '':
        if name[0] != '#':
            names.append(name)

# are there at least two locations?
if len(locations) < 2:
    print 'Not enough locations. Exiting program'
    sys.exit(1)
# are there at least two items?
if len(items) < 2:
    print 'Not enough items. Exiting program'
    sys.exit(1)
# check if items/locations match their categories
if len(items) != len(item_categories):
    print 'Number of items does not match number of item categories'
    sys.exit(1)
if len(locations) != len(location_categories):
    print 'Number of locations does not match number of location categories'
    sys.exit(1)


# the function 'fillIn' takes a sentence and replaces
# the word 'location' for an actual location
# and replaces the word 'item' for an actual item
# as defined in the files:
# locations.txt
# and items.txt
def fillIn(sentence):
    #shuffle the items and the locations
    random.shuffle(items)
    random.shuffle(locations)
    random.shuffle(names)

    #fill in the locations and items in the sentence
    # the counters are used so an item or location is not used twice
    # hence the shuffeling for randomization
    itemCounter         = 0
    locationCounter     = 0
    nameCounter         = 0
    finalSentence       = []
    for word in sentence.split(' '):
        # fill in a location
        if word == 'LOCATION':
            finalSentence.append( locations[locationCounter] )
            locationCounter += 1
        # or an item
        elif word == 'ITEM':
            finalSentence.append( items[itemCounter] )
            itemCounter += 1
        # is it a name?
        elif word == 'NAME':
            finalSentence.append( names[nameCounter] )
            nameCounter += 1
        # perhaps a location with a comma or dot?
        elif word[:-1] == 'LOCATION':
            finalSentence.append( locations[locationCounter] + word[-1])
            locationCounter += 1
        # or an item with a comma or dot or whatever
        elif word[:-1] == 'ITEM':
            finalSentence.append( items[itemCounter] + word[-1])
            itemCounter += 1
        # is it a namewith a comma, dot, whatever?
        elif word[:-1] == 'NAME':
            finalSentence.append( names[nameCounter] + word[-1] )
            nameCounter += 1
        # or else just the word
        else:
            finalSentence.append( word )
    # then make a sentence again out of the created list
    return ' '.join(finalSentence)



def fillInNew(sentence):
    item_indices = range(len(items))
    random.shuffle(item_indices)
    location_indices = range(len(locations))
    random.shuffle(location_indices)

    itemCounter         = 0
    locationCounter     = 0
    nameCounter         = 0
    finalSentence       = []
    explanation         = []
    for word in sentence.split(' '):
        # fill in a location
        if word == 'LOCATION':
            finalSentence.append( locations[int(location_indices[locationCounter])] )
            locationCounter += 1
        # or an item
        elif word == 'ITEM':
            finalSentence.append( items[int(item_indices[itemCounter])] )
            itemCounter += 1
        # is it a name?
        elif word == 'NAME':
            finalSentence.append( names[nameCounter] )
            nameCounter += 1
        # is it an item category?
        elif word == 'ITEM_CATEGORY':
            finalSentence.append( item_categories[int(item_indices[itemCounter])] )
            explanation.append("(")
            explanation.append( item_categories[int(item_indices[itemCounter])] )
            explanation.append("=")
            explanation.append( items[int(item_indices[itemCounter])] )
            explanation.append(")  ")
            itemCounter += 1
        # is it an location category?
        elif word == 'LOCATION_CATEGORY':
            finalSentence.append( location_categories[int(location_indices[locationCounter])] )
            explanation.append("(")
            explanation.append( location_categories[int(location_indices[locationCounter])] )
            explanation.append("=")
            explanation.append( items[int(location_indices[locationCounter])] )
            explanation.append(")  ")
            locationCounter += 1
        # perhaps a location with a comma or dot?
        elif word[:-1] == 'LOCATION':
            finalSentence.append( locations[int(location_indices[locationCounter])] + word[-1])
            locationCounter += 1
        # or an item with a comma or dot or whatever
        elif word[:-1] == 'ITEM':
            finalSentence.append( items[int(item_indices[itemCounter])] )
            itemCounter += 1
        # is it a namewith a comma, dot, whatever?
        elif word[:-1] == 'NAME':
            finalSentence.append( names[nameCounter] + word[-1] )
            nameCounter += 1
        # is it an item category?
        elif word[:-1] == 'ITEM_CATEGORY':
            finalSentence.append( item_categories[int(item_indices[itemCounter])] + word[-1])
            explanation.append("(")
            explanation.append( item_categories[int(item_indices[itemCounter])] )
            explanation.append("=")
            explanation.append( items[int(item_indices[itemCounter])] )
            explanation.append(")  ")
            itemCounter += 1
        # is it an location category?
        elif word[:-1] == 'LOCATION_CATEGORY':
            finalSentence.append( location_categories[int(location_indices[locationCounter])] + word[-1])
            explanation.append("(")
            explanation.append( location_categories[int(location_indices[locationCounter])] )
            explanation.append("=")
            explanation.append( locations[int(location_indices[locationCounter])] )
            explanation.append(")  ")
            locationCounter += 1
        # or else just the word
        else:
            finalSentence.append( word )
    # then make a sentence again out of the created list
    final_command = ' '.join(finalSentence)
    final_explanation = ' '.join(explanation)
    return final_command + "   " + final_explanation


# the tests are defined here

def testOne():
    print '\n'
    print fillInNew( random.choice(cat1Sentences) )
    print '\n\n'

# Category 2
def testTwo():
    print '\n'
    print fillInNew( random.choice(cat2Sentences) )
    print '\n\n'

# Category 3
def testThree():
    # print 'This is the situation for category 3, press enter for the question.\n\n'
    situation = random.choice( cat3Situations )
    print fillInNew( situation[0].split(':')[1] )
    # raw_input()
    print fillInNew( situation[1].split(':')[1] )
    print '\n\n'


#############################################  MAIN LOOP ####################################


# ask the user which test this program should generate
def mainLoop():
    answer = 'begin'
    while True:
        answer = raw_input('Which category do you want to do?   1, 2, 3 or q(uit)')
        if answer == 'q':
            print 'Exiting program.'
            sys.exit(1)
        elif answer == '1':
            print 'Category 1:\n',
            testOne()
        elif answer == '2':
            print 'Category 2:\n'
            testTwo()
        elif answer == '3':
            print 'Category 3:\n'
            testThree()
        else:
            print '\nNot a valid input, please try 1, 2, 3 or q(uit)\n'

if __name__ == "__main__":
    mainLoop()
