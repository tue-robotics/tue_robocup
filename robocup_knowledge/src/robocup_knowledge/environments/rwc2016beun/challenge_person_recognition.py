''' colors from printing on screen '''
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

''' pose "enum" '''
class Pose:
    Standing = 0
    Sitting_down = 1

''' gender "enum" '''
class Gender:
    Male = 0
    Female = 1

''' printing shortcuts '''
def printOk(sentence):
    print prefix + bcolors.OKBLUE + sentence + bcolors.ENDC

def printError(sentence):
    print prefix + bcolors.FAIL + sentence + bcolors.ENDC

def printWarning(sentence):
    print prefix + bcolors.WARNING + sentence + bcolors.ENDC

prefix = bcolors.HEADER + "[Person Recognition] " + bcolors.ENDC


''' minimum number of faces found fo proceed with the challenge '''
min_faces_found = 2


''' determines if a face is discarded for being too close to another already tracked (in meters, as far as i know)'''
face_proximity_treshold = 0.2


''' threshold to consider a person standing up or sitting down, in meters '''
sitting_height_treshold = 0.8


''' waypoint that the robot will visit to find people '''
waypoint_learning = "person_rec_learning"
waypoint_living_room_1 = "person_rec_living_room_1"
waypoint_living_room_2 = "person_rec_living_room_2"
waypoint_living_room_3 = "person_rec_living_room_3"


''' point in the center of the living room, to filter location of humans '''
room_center = {'x':1.365, 'y':0.978, 'z':0.0, 'frame_id':"/map"}


'''Object types that can be recognized'''
object_types=['human']