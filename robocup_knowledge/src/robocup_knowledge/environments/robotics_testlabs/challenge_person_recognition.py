class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Pose:
    Standing = 0
    Sitting_down = 1


class Gender:
    Male = 0
    Female = 1

prefix = bcolors.HEADER + "[Person Recognition] " + bcolors.ENDC


def printOk(sentence):
    print prefix + bcolors.OKBLUE + sentence + bcolors.ENDC


def printError(sentence):
    print prefix + bcolors.FAIL + sentence + bcolors.ENDC


def printWarning(sentence):
    print prefix + bcolors.WARNING + sentence + bcolors.ENDC


min_faces_found = 2

waypoint_learning = "person_rec_learning"
waypoint_living_room_1 = "person_rec_living_room_1"
waypoint_living_room_2 = "person_rec_living_room_2"
waypoint_living_room_3 = "person_rec_living_room_3"
