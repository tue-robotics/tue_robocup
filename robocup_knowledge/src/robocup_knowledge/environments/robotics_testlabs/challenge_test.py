

''' printing shortcuts '''
def printOk(sentence):
    print prefix + bcolors.OKBLUE + sentence + bcolors.ENDC

def printError(sentence):
    print prefix + bcolors.FAIL + sentence + bcolors.ENDC

def printWarning(sentence):
    print prefix + bcolors.WARNING + sentence + bcolors.ENDC

prefix = bcolors.HEADER + "[Challenge Test] " + bcolors.ENDC


''' waypoint that the robot will visit to find people '''
waypoint_learning = "person_rec_learning"


''' point in the center of the living room, to filter location of humans '''
room_center = {'x':1.365, 'y':0.978, 'z':0.0, 'frame_id':"/map"}


'''Object types that can be recognized'''
obj_type_human=['human']
obj_type_drinks=['cola', 'fanta']
