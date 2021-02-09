class RobotFunc(object):
    def __init__(self, name, parttype, functions):
        self._name = name
        self._parttype = parttype
        self._functions = functions

    @property
    def name(self):
        return self._name

    @property
    def parttype(self):
        return self._parttype

    def check_requirements(self, part):
        raise NotImplementedError

    @property
    def functions(self):
        return self._functions


