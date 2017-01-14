# Robot skills
from robot_skills.util.kdl_conversions import point_msg_to_kdl_vector


class Shape(object):
    """ Represents shape properties """
    pass

    @property
    def bottom_z(self):
        return self._calc_bottom_z()

    def _calc_bottom_z(self):
        raise NotImplementedError("_calc_bottom_z must be implemented by subclasses")

    @property
    def top_z(self):
        return self._calc_bottom_z()

    def _calc_top_z(self):
        raise NotImplementedError("_calc_bottom_z must be implemented by subclasses")


class RightPrism(Shape):
    """ Represents a right prism, i.e., a  prism in which the joining edges and faces are perpendicular to the base
    faces. This is typical for the shapes resulting from the convex hull, z min and z max of EntityInfo

    """

    def _calc_top_z(self):
        return self._z_max

    def _calc_bottom_z(self):
        return self._z_min

    def __init__(self, convex_hull, z_min, z_max):
        """ Constructor

        :param convex_hull: list with kdl Vectors representing the vertices of the convex hull. N.B.: these should only
         contain x and y values, the z-values are not relevant (the height information is contained in the z_min
         and z_max parameters).
        :param z_min: float minimum height [m] # ToDo: w.r.t.???
        :param z_max: float maximum height [m] # ToDo: w.r.t.???
        """
        super(RightPrism, self).__init__()
        self._convex_hull = convex_hull
        self._z_min = z_min
        self._z_max = z_max


def shape_from_entity_info(e):
    """ Creates a shape from the convex_hull, z_min and z_max of the EntityInfo object. If no convex hull is present,
    an empty Shape object is returned

    :param e: ed.msg.EntityInfo
    :return: Shape
    """
    # Check if we have a convex hull
    if not e.convex_hull:
        return Shape()

    return RightPrism(convex_hull=[point_msg_to_kdl_vector(p) for p in e.convex_hull],
                      z_min=e.z_min,
                      z_max=e.z_max)
