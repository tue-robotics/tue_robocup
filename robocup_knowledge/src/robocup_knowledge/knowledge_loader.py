# Resolve the environment variable $ROBOT_ENV

import os, sys, imp
_robot_env = os.environ.get('ROBOT_ENV')

if not _robot_env:
    print "ROBOT_ENV environment variable is not set!"
    sys.exit()

# Look for the correct knowledge file
try:
    _knowledge_path = os.path.dirname(os.path.realpath(__file__)) + "/environments/%s.py" % _robot_env
    _env_knowledge = imp.load_source('environment_knowledge', _knowledge_path)
except:
    print "Knowledge file for environment '%s' (%s) does not exist!"%(_robot_env, _knowledge_path)
    sys.exit()

# The actual knowledge
try:
    rooms = _env_knowledge.rooms
    names = _env_knowledge.names
    drinks = _env_knowledge.drinks
except AttributeError as e:
    print "Invalid knowledge file for environment '%s' :"%_robot_env + str(e)
    sys.exit()
