# ROS
import PyKDL as kdl


class Volume(object):
    """ Represents an area of an entity

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
        raise NotImplementedError("_get_center_point must be implemented by subclasses")

    def contains(self, point):
        """ Checks if the point is inside this volume
        :param point: kdl Vector w.r.t. the same frame as this volume
        :return: True if inside, False otherwise
        """
        raise NotImplementedError("contains must be implemented by subclasses")


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
        return self._min_corner.x() < point.x() < self._max_corner.x() and \
               self._min_corner.y() < point.y() < self._max_corner.y() and \
               self._min_corner.z() < point.z() < self._max_corner.z()

    def __repr__(self):
        return "BoxVolume(min_corner={}, max_corner={})".format(self.min_corner, self.max_corner)


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


def volumes_from_entity_info_data(data):
    """ Creates a dict mapping strings to Volumes from the EntityInfo data dictionary

    :param data: ed_msgs.msg.EntityInfo.data (string)
    :return: dict mapping strings to Volumes
    """
    # Check if we have data and if it contains areas
    if data is None or 'areas' not in data:
        return {}

    # Loop over all areas
    volumes = {}
    for a in data['areas']:

        # Check if the volume has a name. Otherwise: skip
        if 'name' not in a:
            continue
        name = a['name']

        # Check if this is an 'OffsetVolume'
        if 'offset' in a:
            volumes[name] = OffsetVolume(a['offset'])
            continue

        # Check if we have a shape
        if 'shape' in a:

            shapes = a['shape']

            # Check if this is a single shape
            if len(shapes) > 1:
                print "\nError [volumes_from_entity_info_data]: Cannot handle compound shapes yet...\n"
                continue
            shape = shapes[0]

            # Check if this one shape is a box
            if 'box' in shape:
                box = shape['box']
                mic = box['min']
                min_corner = kdl.Vector(mic['x'], mic['y'], mic['z'])
                mac = box['max']
                max_corner = kdl.Vector(mac['x'], mac['y'], mac['z'])
                volumes[name] = BoxVolume(min_corner=min_corner, max_corner=max_corner)
                continue

        # If we end up here, we don't know what to do with the area
        print "\nError [volumes_from_entity_info_data]: don't know what to do with {}\n".format(a)

    return volumes

if __name__ == "__main__":
    import doctest
    doctest.testmod()
