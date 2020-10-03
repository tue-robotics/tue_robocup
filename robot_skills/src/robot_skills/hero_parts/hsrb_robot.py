import importlib
import json
import weakref
from hsrb_interface import exceptions, robot, settings

settings._HSRB_SETTINGS = settings._HSRB_SETTINGS.replace("/hsrb/", "/hero/")
settings._SETTINGS = json.loads(settings._HSRB_SETTINGS)


def get(self, name, typ=None):
    """Get an item if available.

    The main difference with the original method is that all namespaces
    are removed from the settings.

    Args:
        name (str):   A name of ``Item`` to get.
        typ (Types):  A type of ``Item`` to get.

    Returns:
        Item: An instance with a specified name

    Raises:
        hsrb_interface.exceptions.ResourceNotFoundError
    """
    if typ is None:
        section, config = settings.get_entry_by_name(name)
        types = filter(lambda e: e.value == section, robot.ItemTypes)
        if types:
            typ = types[0]
        else:
            msg = "No such category ({0})".format(section)
            raise exceptions.ResourceNotFoundError(msg)
    key = (name, typ)
    if key in self._registry:
        return self._registry.get(key, None)
    else:
        config = settings.get_entry(typ.value, name)
        module_name, class_name = config["class"]
        module = importlib.import_module(".{0}".format(module_name),
                                         "hsrb_interface")
        print("Overriding settings: {}".format(module.settings))
        module.settings._HSRB_SETTINGS = module.settings._HSRB_SETTINGS.replace("/hsrb/", "/hero/")
        module.settings._SETTINGS = json.loads(module.settings._HSRB_SETTINGS)
        cls = getattr(module, class_name)
        obj = cls(name)
        self._registry[key] = obj
        return weakref.proxy(obj)


robot._ConnectionManager.get = get
Robot = robot.Robot

