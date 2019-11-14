from __future__ import print_function

# Resolve the environment variable $ROBOT_ENV

def load_knowledge(knowledge_item, print_knowledge=False):
    import os, imp
    _robot_env = os.environ.get('ROBOT_ENV')

    if not _robot_env:
        raise KeyError("robocup_knowledge - load(): ROBOT_ENV environment variable is not set!")

    # Look for the correct knowledge file
    try:
        _knowledge_path = os.path.dirname(os.path.realpath(__file__)) + "/environments/%s/%s.py" % (_robot_env, knowledge_item)
        knowledge = imp.load_source(knowledge_item, _knowledge_path)

        knowledge_attrs = [attr for attr in dir(knowledge) if not callable(attr) and not attr.startswith("__")]

        if print_knowledge:
            print("=====================================")
            print("==          KNOWLEDGE              ==")
            print("=====================================")
            for attr in knowledge_attrs:
                print("==> %s = %s" % (attr, str(getattr(knowledge, attr))))
            print("=====================================")

        return knowledge

    except Exception as e:
        raise RuntimeError("Knowledge item '%s' for environment '%s' is incorrect at path '%s'! [Error = %s]"%(knowledge_item, _robot_env, _knowledge_path, e))
