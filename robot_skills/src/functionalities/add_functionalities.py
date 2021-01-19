import rospy
import types

from .arm.guarded_motion import GuardedMotionFunc

def add_functionality(part, functionality):
    funcs = functionality.functions
    for funcname, func in funcs.items():
        setattr(part, funcname, types.MethodType(func, part))
    part.functionalities.append(functionality.name)

def add_functionalities(robot, known_functionalities=None):
    if known_functionalities is None:
        known_functionalities = [GuardedMotionFunc()] #TODO magically collect all functionalities in a certain location

    for func in known_functionalities:
        target_part_type = func.parttype
        available_parts = [part for part in robot.parts.values() if isinstance(part, target_part_type)]
        for part in available_parts:
            # check whether this part meets the requirements
            if func.check_requirements(part):
                add_functionality(part, func)
