# Resolve the environment variable $ROBOT_ENV

def load_knowledge(knowledge_item):
    import os, sys, imp
    _robot_env = os.environ.get('ROBOT_ENV')

    if not _robot_env:
        print "robocup_knowledge - load(): ROBOT_ENV environment variable is not set!"
        sys.exit()

    # Look for the correct knowledge file
    try:
        _knowledge_path = os.path.dirname(os.path.realpath(__file__)) + "/environments/%s/%s.py" % (_robot_env, knowledge_item)
        return imp.load_source('environment_knowledge', _knowledge_path)
    except:
        print "Knowledge item '%s' for environment '%s' does not exist at path '%s'!"%(knowledge_item, _robot_env, _knowledge_path)
        sys.exit()
