import pickle
from collections import defaultdict
import pprint
import numpy as np

import matplotlib.pyplot as plt

from sklearn.cluster import KMeans


loaded = pickle.load(open('/home/loy/kmeans.pickle'))
# loaded is a list of dicts {'rgb':..., 'person_detection':..., 'map_ps':...}

xs = [person['map_ps'].point.x for person in loaded]
ys = [person['map_ps'].point.y for person in loaded]

plt.scatter(xs, ys, c='r')

people_pos = np.array([xs, ys]).T  # people_pos is a np.array of [(x, y)]

kmeans = KMeans(n_clusters=4, random_state=0)
kmeans.fit(people_pos)

# hashable_dict = tuple(loaded[0].items())
hashable_dicts = [tuple(d.items()) for d in loaded]  # A dict isn't hashable so can't be dict key. But a tuple can be, so we create ((k, v), (k, v), ...) tuple

# hashable_dicts2label maps elements of loaded to their laels
hashable_dicts2label = dict(zip(hashable_dicts,
                                kmeans.labels_))

label2hashable_dicts = defaultdict(list)

for hashable, label in sorted(hashable_dicts2label.items()):
    label2hashable_dicts[label].append(dict(hashable))  # And here we create a normal dict again
# label2hashable_dicts maps cluster labels to a list of {'rgb':..., 'person_detection':..., 'map_ps':...}

room_center = np.array([6, 0])

# Now we need to select wich element of the cluster is closest to the room_center
persons_closest_to_room_center = {}

# import pdb; pdb.set_trace()
for label, persons in label2hashable_dicts.items():
    position = np.array([person['map_ps'].point.x,
                         person['map_ps'].point.y])

    closest = sorted(persons, key=lambda person: np.hypot(*(np.array([person['map_ps'].point.x,
                                                                      person['map_ps'].point.y]) - room_center)))
    persons_closest_to_room_center[label] = closest[0]

# persons_closest_to_room_center is a map of label to a {'rgb':..., 'person_detection':..., 'map_ps':...}
# import pdb; pdb.set_trace()

xs2 = [person['map_ps'].point.x for person in persons_closest_to_room_center.values()]
ys2 = [person['map_ps'].point.y for person in persons_closest_to_room_center.values()]

plt.scatter(xs2, ys2, c='b')
plt.show()

locations = zip(xs2, ys2)
pprint.pprint(locations)

with open('/home/loy/kmeans_output.pickle', 'w') as dumpfile:
    pickle.dump(locations, dumpfile)