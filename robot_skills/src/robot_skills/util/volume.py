# ROS
import PyKDL as kdl


class Volume(object):
    """ Represents an area of an entity

    """
    def __init__(self):
        """ Constructor

        """
        pass


class BoxVolume(Volume):
    """ Represents a box shaped volume """
    def __init__(self, min_corner, max_corner):
        """ Constructor.

        :param min_corner: kdl Vector with the minimum bounding box corner
        :param max_corner: kdl Vector with the minimum bounding box corner
        """
        super(BoxVolume, self).__init__()
        self._min_corner = min_corner
        self._max_corner = max_corner


class OffsetVolume(Volume):
    """ Represents a volume with a certain offset from the convex hull of the entity """
    def __init__(self, offset):
        """ Constructor

        :param offset: float with offset [m]
        """
        super(OffsetVolume, self).__init__()
        self._offset = offset


class CompoundVolume(Volume):
    """ Represents a compound volume """
    # ToDo
    pass


def volumes_from_entity_info_data(data):
    """ Creates a dict mapping strings to Volumes from the EntityInfo data dictionary

    :param data: ed.msg.EntityInfo.data (string)
    :return: dict mapping strings to Volumes
    """
    # Check if the data contains areas
    if 'areas' not in data:
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
                mac = box['man']
                max_corner = kdl.Vector(mac['x'], mac['y'], mac['z'])
                volumes[name] = BoxVolume(min_corner=min_corner, max_corner=max_corner)
                continue

        # If we end up here, we don't know what to do with the area
        print "\nError [volumes_from_entity_info_data]: don't know what to do with {}\n".format(a)

    return volumes
