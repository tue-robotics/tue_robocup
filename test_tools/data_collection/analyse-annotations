#!/usr/bin/env python

# System
import os

# TU/e Robotics
from robocup_knowledge import knowledge_loader


"""
Counts the images in the subdirectories of
~/MEGA/data<ROBOT_ENV>/training_data/annotated. Both the verified and
unverified annotations are checked and a summary is printed to screen.
This contains:
- Per object that is present in the database: the amount of images present in the directory
- If images are not present in the database, a warning is printed
"""


# Colors from printing on screen
class BColors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def count_images(objects, path):
    """ Counts the images in the subdirectories of 'path'. The subdirectories are identified by the provided objects.
    The results are printed to screen

    :param objects: list with strings
    :param path: string indicating the path
    """
    # List the number of occurrences in each sub folder
    ustats = []  # Unverified
    for o in objects:
        p = os.path.join(path, o)

        # If path doesn't exist, we probably don't have any images
        if not os.path.exists(p):
            ustats.append((o, 0))
        else:
            ustats.append((o, len(os.listdir(p))))

    # Sort and print the results
    ustats.sort(key=lambda tup: tup[1], reverse=True)
    for s in ustats:
        if s[1] > 0:
            print "{}: {}".format(s[0], s[1])
        else:
            print BColors.WARNING + "{}: {}".format(s[0], s[1]) + BColors.ENDC

    # Sanity check: try to identify mismatches between object names and annotated images
    print BColors.BOLD + "\nPossible mismatches:" + BColors.ENDC
    print "Annotated but not in knowledge"
    for candidate in os.listdir(path):
        if candidate not in objects:
            print BColors.WARNING + candidate + BColors.ENDC

    print


if __name__ == "__main__":

    # Get the names of the objects in which we are interested
    common_knowledge = knowledge_loader.load_knowledge("common")
    objects = common_knowledge.object_names
    objects_set = set(objects)


    robot_env = os.environ.get("ROBOT_ENV")
    # get the names of the objects in output labels
    tensorflow_labels_path = os.path.join(os.path.expanduser("~"), "MEGA", "data", robot_env, "models", "tensorflow_ros", "output_labels.txt")
    with open(tensorflow_labels_path) as tensorflow_labels_file:
        raw_labels_lines = tensorflow_labels_file.readlines()
        tensorflow_labels = [line.strip() for line in raw_labels_lines]
        tensorflow_set = set(tensorflow_labels)

    print "objects that are present in objects_set but not in tensorflow_set\n"
    for o in objects_set.difference(tensorflow_set):
        print o
    print

    print "objects that are present in tensorflow_set but not in objects_set\n"
    for t in tensorflow_set.difference(objects_set):
        print t
    print

    # Get the path to the folder where images are stored
    path = os.path.join(os.path.expanduser("~"), "MEGA", "data", robot_env, "training_data", "annotated")

    # Count both verified and unverified
    for v in ["verified", "unverified"]:
        tpath = os.path.join(path, v)
        print BColors.HEADER + BColors.BOLD + v.upper() + BColors.ENDC + ':\n'
        count_images(objects=objects, path=tpath)
