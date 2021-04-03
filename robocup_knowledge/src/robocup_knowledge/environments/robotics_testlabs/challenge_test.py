from __future__ import print_function

''' printing shortcuts '''
def printOk(sentence):
    print(prefix + bcolors.OKBLUE + sentence + bcolors.ENDC)

def printError(sentence):
    print(prefix + bcolors.FAIL + sentence + bcolors.ENDC)

def printWarning(sentence):
    print(prefix + bcolors.WARNING + sentence + bcolors.ENDC)

prefix = bcolors.HEADER + "[Challenge Test] " + bcolors.ENDC


''' waypoint used to test navigation '''
wp_test_nav = "wp_test_nav1"


''' point in the center of the living room, to filter location of humans '''
# room_center = {'x':1.365, 'y':0.978, 'z':0.0, 'frame_id':"map"}

''' entity to inspect '''
INSPECT_ENTITY_ID = "cabinet"
INSPECT_ROOM_ID = "kitchen"

'''Object types that can be recognized'''
obj_type_human=['human']
obj_type_drinks=['cola', 'fanta']
