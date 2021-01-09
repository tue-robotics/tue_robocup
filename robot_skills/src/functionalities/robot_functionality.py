import rospy

class RobotFunc():
    def __init__(self, name, parttype, functions):
        self._name = name,
        self.parttype = parttype
        self.functions = functions

    @property
    def name(self):
        return self._name

    @property
    def parttype(self):
        return self._parttype

    def check_requirements(self):
        raise NotImplementedError

    @property
    def functions(self):
        return self.functions


