# Robot skills
from robot_skills.util.kdl_conversions import pointMsgToKdlVector


class Shape(object):
    """ Represents shape properties """
    pass

    @property
    def convex_hull(self):
        return self._calc_convex_hull()

    def _calc_convex_hull(self):
        raise NotImplementedError("_calc_convex_hull must be implemented by subclasses. Class {cls} has no implementation".format(cls=self.__class__.__name__))

    @property
    def x_min(self):
        return self._calc_x_min()

    def _calc_x_min(self):
        raise NotImplementedError("_calc_x_min must be implemented by subclasses. Class {cls} has no implementation".format(cls=self.__class__.__name__))

    @property
    def x_max(self):
        return self._calc_x_max()

    def _calc_x_max(self):
        raise NotImplementedError("_calc_x_max must be implemented by subclasses. Class {cls} has no implementation".format(cls=self.__class__.__name__))

    @property
    def y_min(self):
        return self._calc_y_min()

    def _calc_y_min(self):
        raise NotImplementedError("_calc_y_min must be implemented by subclasses. Class {cls} has no implementation".format(cls=self.__class__.__name__))

    @property
    def y_max(self):
        return self._calc_y_max()

    def _calc_y_max(self):
        raise NotImplementedError("_calc_y_max must be implemented by subclasses. Class {cls} has no implementation".format(cls=self.__class__.__name__))

    @property
    def z_min(self):
        return self._calc_z_min()

    def _calc_z_min(self):
        raise NotImplementedError("_calc_z_min must be implemented by subclasses. Class {cls} has no implementation".format(cls=self.__class__.__name__))

    @property
    def z_max(self):
        return self._calc_z_max()

    def _calc_z_max(self):
        raise NotImplementedError("_calc_z_max must be implemented by subclasses. Class {cls} has no implementation".format(cls=self.__class__.__name__))


class RightPrism(Shape):
    """ Represents a right prism, i.e., a  prism in which the joining edges and faces are perpendicular to the base
    faces. This is typical for the shapes resulting from the convex hull, z min and z max of EntityInfo

    """
    def __init__(self, convex_hull, z_min, z_max):
        """ Constructor

        :param convex_hull: list with kdl Vectors representing the vertices of the convex hull. N.B.: these should only
         contain x and y values, the z-values are not relevant (the height information is contained in the z_min
         and z_max parameters).
        :param z_min: float minimum height [m] w.r.t. the center of the corresponding Entity
        :param z_max: float maximum height [m] w.r.t. the center of the corresponding Entity
        """
        super(RightPrism, self).__init__()
        self._convex_hull = convex_hull
        self._z_min = z_min
        self._z_max = z_max

    def _calc_x_max(self):
        return max(ch.x() for ch in self._convex_hull)

    def _calc_x_min(self):
        return min(ch.x() for ch in self._convex_hull)

    def _calc_y_max(self):
        return max(ch.y() for ch in self._convex_hull)

    def _calc_y_min(self):
        return min(ch.y() for ch in self._convex_hull)

    def _calc_z_max(self):
        return self._z_max

    def _calc_z_min(self):
        return self._z_min

    def _calc_convex_hull(self):
        return self._convex_hull



def shape_from_entity_info(e):
    """ Creates a shape from the convex_hull, z_min and z_max of the EntityInfo object. If no convex hull is present,
    an empty Shape object is returned

    :param e: ed_msgs.msg.EntityInfo
    :return: Shape
    """
    # Check if we have a convex hull
    if not e.convex_hull:
        return Shape()

    return RightPrism(convex_hull=[pointMsgToKdlVector(p) for p in e.convex_hull],
                      z_min=e.z_min,
                      z_max=e.z_max)
