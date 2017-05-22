import sys

def robot_constructor(robot_name=None):
    """Construct a robot by it's name. Choices are amigo, sergio, mockbot"""
    if not robot_name:
        robot_name = sys.argv[1]

    if robot_name == "amigo":
        import robot_skills.amigo
        return robot_skills.amigo.Amigo(wait_services=True)
    elif robot_name == "sergio":
        import robot_skills.sergio
        return robot_skills.sergio.Sergio(wait_services=True)
    elif robot_name == "mockbot":
        import robot_skills.mockbot
        return robot_skills.mockbot.Mockbot(wait_services=True)
    else:
        return None
