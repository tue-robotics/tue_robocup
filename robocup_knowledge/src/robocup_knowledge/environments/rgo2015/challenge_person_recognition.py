# PERSON RECOGNITION RGO2015
from __future__ import print_function

names = ["Anna","Beth","Carmen","Jennifer","Jessica","Kimberly","Kristina","Laura","Mary","Sarah","Alfred","Charles","Daniel","James","John","Luis","Paul","Richard","Robert","Steve"]

locations = [ 'kitchen', 'livingroom', 'hall', 'bedroom', 'office']

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
    print(prefix + bcolors.OKBLUE + sentence + bcolors.ENDC)

def printError(sentence):
    print(prefix + bcolors.FAIL + sentence + bcolors.ENDC)

def printWarning(sentence):
    print(prefix + bcolors.WARNING + sentence + bcolors.ENDC)
