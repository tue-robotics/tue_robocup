import matplotlib.pyplot as plt
import pickle
import pprint
from collections import defaultdict
from sklearn.cluster import KMeans

import numpy as np


# ToDo: replace with better algorithm


def cluster_people(people_dicts, room_center, plot=False, n_clusters=4):
    xs = [person['map_ps'].point.x for person in people_dicts]
    ys = [person['map_ps'].point.y for person in people_dicts]

    if plot:
        plt.scatter(xs, ys, c='r')

    people_pos = np.array([xs, ys]).T  # people_pos is a np.array of [(x, y)]

    kmeans = KMeans(n_clusters=n_clusters, random_state=0)
    kmeans.fit(people_pos)

    # hashable_dict = tuple(people_dicts[0].items())
    # A dict isn't hashable so can't be dict key. But a tuple can be, so we create ((k, v), (k, v), ...) tuple
    hashable_dicts = [tuple(d.items()) for d in people_dicts]

    # hashable_dicts2label maps elements of people_dicts to their laels
    hashable_dicts2label = dict(zip(hashable_dicts,
                                    kmeans.labels_))

    label2hashable_dicts = defaultdict(list)

    for hashable, label in sorted(hashable_dicts2label.items()):
        label2hashable_dicts[label].append(dict(hashable))  # And here we create a normal dict again
    # label2hashable_dicts maps cluster labels to a list of {'rgb':..., 'person_detection':..., 'map_ps':...}

    # Now we need to select wich element of the cluster is closest to the room_center
    persons_closest_to_room_center = {}

    # import pdb; pdb.set_trace()
    for label, persons in label2hashable_dicts.items():
        # For each cluster, we want the detection that is closest to the cluster centroid/ kmeans.cluster_centers
        closest = sorted(persons, key=lambda _person: np.hypot(
            *(np.array([_person['map_ps'].point.x,
                        _person['map_ps'].point.y]) - kmeans.cluster_centers_[label])))
        persons_closest_to_room_center[label] = closest[0]

    # persons_closest_to_room_center is a map of label to a {'rgb':..., 'person_detection':..., 'map_ps':...}
    # import pdb; pdb.set_trace()

    xs2 = [person['map_ps'].point.x for person in persons_closest_to_room_center.values()]
    ys2 = [person['map_ps'].point.y for person in persons_closest_to_room_center.values()]

    if plot:
        plt.scatter(xs2, ys2, c='b')
        plt.show()

    # locations = zip(xs2, ys2)

    return persons_closest_to_room_center.values()


if __name__ == "__main__":
    import sys
    ppl_dicts = pickle.load(open(sys.argv[1]))
    # ppl_dicts is a list of dicts {'rgb':..., 'person_detection':..., 'map_ps':...}

    clustered_ppl = cluster_people(ppl_dicts, room_center=np.array([6, 0]), plot=True)

    _xs2 = [_person['map_ps'].point.x for _person in clustered_ppl]
    _ys2 = [_person['map_ps'].point.y for _person in clustered_ppl]
    locations = zip(_xs2, _ys2)
    pprint.pprint(locations)

    with open('/home/loy/kmeans_output.pickle', 'w') as dumpfile:
        pickle.dump(clustered_ppl, dumpfile)
