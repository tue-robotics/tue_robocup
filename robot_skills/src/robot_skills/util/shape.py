# Robot skills
from robot_skills.util.kdl_conversions import point_msg_to_kdl_vector


class Shape(object):
    """ Represents shape properties """
    pass

    @property
    def convex_hull(self):
        return self._calc_convex_hull()

    def _calc_convex_hull(self):
        raise NotImplementedError("_calc_convex_hull must be implemented by subclasses")

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
    def __init__(self, convex_hull, bottom_z, top_z):
        """ Constructor

        :param convex_hull: list with kdl Vectors representing the vertices of the convex hull. N.B.: these should only
         contain x and y values, the z-values are not relevant (the height information is contained in the z_min
         and z_max parameters).
        :param bottom_z: float minimum height [m] w.r.t. the center of the corresponding Entity
        :param top_z: float maximum height [m] w.r.t. the center of the corresponding Entity
        """
        super(RightPrism, self).__init__()
        self._convex_hull = convex_hull
        self._bottom_z = bottom_z
        self._top_z = top_z

    def _calc_top_z(self):
        return self._top_z

    def _calc_bottom_z(self):
        return self._bottom_z

    def _calc_convex_hull(self):
        return self._convex_hull



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
