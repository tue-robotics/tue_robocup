# ROS
import PyKDL as kdl
from kdl_conversions import point_msg_to_kdl_vector
from numpy import abs


class Volume(object):
    """ Represents an volume of an entity

    Points are defined relative to the object they belong to
    """

    def __init__(self):
        """ Constructor

        """
        pass

    @property
    def center_point(self):
        """Get the center of the Volume"""
        return self._calc_center_point()

    def _calc_center_point(self):
        raise NotImplementedError("_calc_center_point must be implemented by subclasses. "
                                  "Class {cls} has no implementation".format(cls=self.__class__.__name__))

    def contains(self, point):
        """ Checks if the point is inside this volume

        :param point: kdl Vector w.r.t. the same frame as this volume
        :return: True if inside, False otherwise
        """
        raise NotImplementedError("contains must be implemented by subclasses. " 
                                  "Class {cls} has no implementation".format(cls=self.__class__.__name__))

    @property
    def size(self):
        return self._calc_size()

    def _calc_size(self):
        raise NotImplementedError("_calc_size must be implemented by subclasses. "
                                  "Class {cls} has no implementation".format(cls=self.__class__.__name__))


class BoxVolume(Volume):
    """ Represents a box shaped volume """

    def __init__(self, min_corner, max_corner):
        """ Constructor.

        Points are defined relative to the object they belong to

        :param min_corner: PyKDL.Vector with the minimum bounding box corner
        :param max_corner: PyKDL.Vector with the minimum bounding box corner
        """
        super(BoxVolume, self).__init__()

        assert isinstance(min_corner, kdl.Vector)
        assert isinstance(max_corner, kdl.Vector)

        self._min_corner = min_corner
        self._max_corner = max_corner

    def _calc_center_point(self):
        """Calculate where the center of the box is located

        >>> b = BoxVolume(kdl.Vector(0,0,0), kdl.Vector(1,1,1))
        >>> b.center_point
        [         0.5,         0.5,         0.5]
        """
        return kdl.Vector(0.5 * (self._min_corner.x() + self._max_corner.x()),
                          0.5 * (self._min_corner.y() + self._max_corner.y()),
                          0.5 * (self._min_corner.z() + self._max_corner.z()))

    def _calc_size(self):
        """Calculate the size of a volume
        >>> BoxVolume(kdl.Vector(0, 0, 0), kdl.Vector(1, 1, 1)).size
        1.0
        >>> BoxVolume(kdl.Vector(0, 0, 0), kdl.Vector(10, 10, 0.1)).size
        10.0
        >>> BoxVolume(kdl.Vector(0, 0, 0), kdl.Vector(1, 1, 10)).size
        10.0
        """
        size_x = abs(self._max_corner.x() - self._min_corner.x())
        size_y = abs(self._max_corner.y() - self._min_corner.y())
        size_z = abs(self._max_corner.z() - self._min_corner.z())
        return size_x * size_y * size_z

    @property
    def min_corner(self):
        return self._min_corner

    @property
    def max_corner(self):
        return self._max_corner

    @property
    def bottom_area(self):
        convex_hull = []
        convex_hull.append(kdl.Vector(self.min_corner.x(), self.min_corner.y(), self.min_corner.z()))  # 1
        convex_hull.append(kdl.Vector(self.max_corner.x(), self.min_corner.y(), self.min_corner.z()))  # 2
        convex_hull.append(kdl.Vector(self.max_corner.x(), self.max_corner.y(), self.min_corner.z()))  # 3
        convex_hull.append(kdl.Vector(self.min_corner.x(), self.max_corner.y(), self.min_corner.z()))  # 4
        return convex_hull

    def contains(self, point):
        """ Checks if the point is inside this volume

        :param point: kdl Vector w.r.t. the same frame as this volume
        :return: True if inside, False otherwise
        """
        return (self._min_corner.x() <= point.x() <= self._max_corner.x() and
                self._min_corner.y() <= point.y() <= self._max_corner.y() and
                self._min_corner.z() <= point.z() <= self._max_corner.z())

    def __repr__(self):
        return "BoxVolume(min_corner={}, max_corner={})".format(self.min_corner, self.max_corner)


class CompositeBoxVolume(Volume):
    """ Represents a composite box shaped volume """
    def __init__(self, boxes):
        """
        Constructor

        Points are defined relative to the object they belong to.

        :param boxes: list of tuples of two PyKDL.Vector. First one with the minimum bounding box corners,
                            second one with the maximum bounding box corners
        """
        super(CompositeBoxVolume, self).__init__()

        assert isinstance(boxes, list)
        assert len(boxes) > 0
        assert isinstance(boxes[0], tuple)

        self._min_corners = zip(*boxes)[0]
        self._max_corners = zip(*boxes)[1]

    def _calc_center_point(self):
        """Calculate where the center of the box is located

        >>> b = CompositeBoxVolume([(kdl.Vector(0,0,0), kdl.Vector(1,1,1))])
        >>> b.center_point
        [         0.5,         0.5,         0.5]
        """
        min_x = min([v.x() for v in self._min_corners])
        min_y = min([v.y() for v in self._min_corners])
        min_z = min([v.z() for v in self._min_corners])
        max_x = max([v.x() for v in self._max_corners])
        max_y = max([v.y() for v in self._max_corners])
        max_z = max([v.z() for v in self._max_corners])
        return kdl.Vector(0.5 * (min_x + max_x),
                          0.5 * (min_y + max_y),
                          0.5 * (min_z + max_z))

    @property
    def min_corner(self):
        min_x = min([v.x() for v in self._min_corners])
        min_y = min([v.y() for v in self._min_corners])
        min_z = min([v.z() for v in self._min_corners])
        return kdl.Vector(min_x, min_y, min_z)

    @property
    def max_corner(self):
        max_x = max([v.x() for v in self._max_corners])
        max_y = max([v.y() for v in self._max_corners])
        max_z = max([v.z() for v in self._max_corners])
        return kdl.Vector(max_x, max_y, max_z)

    @property
    def bottom_area(self):
        min_x = min([v.x() for v in self._min_corners])
        min_y = min([v.y() for v in self._min_corners])
        min_z = min([v.z() for v in self._min_corners])
        max_x = max([v.x() for v in self._max_corners])
        max_y = max([v.y() for v in self._max_corners])
        convex_hull = [kdl.Vector(min_x, min_y, min_z), kdl.Vector(max_x, min_y, min_z),
                       kdl.Vector(max_x, max_y, min_z), kdl.Vector(min_x, max_y, min_z)]
        return convex_hull

    def contains(self, point):
        """ Checks if the point is inside this volume

        :param point: kdl Vector w.r.t. the same frame as this volume
        :return: True if inside, False otherwise
        """
        for min_corner, max_corner in zip(self._min_corners, self._max_corners):
            if (min_corner.x() <= point.x() <= max_corner.x() and
                min_corner.y() <= point.y() <= max_corner.y() and
                min_corner.z() <= point.z() <= max_corner.z()):
                return True

        return False

    def __repr__(self):
        description = "CompositeBoxVolume:\n"
        for min_corner, max_corner in zip(self._min_corners, self._max_corners):
            description += "min_corner={}, max_corner={}\n".format(min_corner, max_corner)

        return description


class OffsetVolume(Volume):
    """ Represents a volume with a certain offset from the convex hull of the entity """
    def __init__(self, offset):
        """ Constructor

        :param offset: float with offset [m]
        """
        super(OffsetVolume, self).__init__()
        self._offset = offset

    def __repr__(self):
        return "OffsetVolume(offset={})".format(self._offset)


def volume_from_entity_volume_msg(msg):
    """ Creates a dict mapping strings to Volumes from the EntityInfo data dictionary

    :param msg: ed_msgs.msg.Volume
    :return: tuple of name and volume object
    """
    # Check if we have data and if it contains volumes
    if not msg:
        return None, None

    # Check if the volume has a name. Otherwise: skip
    if not msg.name:
        return None, None
    name = msg.name

    # Check if we have a shape
    if len(msg.subvolumes) == 1:
        subvolume = msg.subvolumes[0]
        if not subvolume.geometry.type == subvolume.geometry.BOX:
            return None, None

        center_point = point_msg_to_kdl_vector(subvolume.center_point.point)

        size = subvolume.geometry.dimensions
        size = kdl.Vector(size[0], size[1], size[2])

        min_corner = center_point - size / 2
        max_corner = center_point + size / 2

        return name, BoxVolume(min_corner, max_corner)
    else:
        min_corners = []
        max_corners = []
        for subvolume in msg.subvolumes:
            if not subvolume.geometry.type == subvolume.geometry.BOX:
                continue

            size = subvolume.geometry.dimensions
            size = kdl.Vector(size[0], size[1], size[2])

            center_point = point_msg_to_kdl_vector(subvolume.center_point)

            sub_min = center_point - size / 2
            sub_max = center_point + size / 2
            min_corners.append(sub_min)
            max_corners.append(sub_max)

        return name, CompositeBoxVolume(zip(min_corners, max_corners))


def volumes_from_entity_volumes_msg(msg):
    if not msg:
        return {}

    volumes = {}
    for v in msg:
        if not v.name:
            continue

        name, volume = volume_from_entity_volume_msg(v)
        if name and volume:
            volumes[name] = volume

    return volumes


if __name__ == "__main__":
    import doctest
    doctest.testmod()
