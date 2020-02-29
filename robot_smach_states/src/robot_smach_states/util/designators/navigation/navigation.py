from __future__ import absolute_import

# System
import abc

# TU/e Robotics
from ..core import Designator


class NavigationConstraintsDesignator(Designator):
    __metaclass__ = abc.ABCMeta

    def __init__(self, name=None):
        super(NavigationConstraintsDesignator, self).__init__(resolve_type=tuple, name=name)

    @abc.abstractmethod
    def _resolve(self):
        pass

