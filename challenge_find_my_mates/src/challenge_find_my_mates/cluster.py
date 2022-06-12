from __future__ import print_function

import pickle
from collections import defaultdict
import numpy as np

import matplotlib.pyplot as plt

from sklearn.cluster import KMeans


def cluster_people(people_dicts, room_center, plot=False):
    xs = [person['map_vs'].vector.x() for person in people_dicts]
    ys = [person['map_vs'].vector.y() for person in people_dicts]

    if plot:
        plt.scatter(xs, ys, c='r')

    people_pos = np.array([xs, ys]).T  # people_pos is a np.array of [(x, y)]

    kmeans = KMeans(n_clusters=4, random_state=0)
    kmeans.fit(people_pos)

    label2detection = defaultdict(list)

    for label, detection in zip(kmeans.labels_, people_dicts):
        label2detection[label].append(detection)

    # Now we need to select which element of the cluster is closest to the room_center
    persons_closest_to_room_center = {}

    # import pdb; pdb.set_trace()
    for label, persons in label2detection.items():
        # For each cluster, we want the detection that is closest to the cluster centroid/ kmeans.cluster_centers
        closest = sorted(persons, key=lambda person: np.hypot(*(np.array([person['map_vs'].vector.x(),
                                                                          person['map_vs'].vector.y()]) - kmeans.cluster_centers_[label])))
        persons_closest_to_room_center[label] = closest[0]

    # persons_closest_to_room_center is a map of label to a {'rgb':..., 'person_detection':..., 'map_vs':...}
    # import pdb; pdb.set_trace()

    xs2 = [person['map_vs'].vector.x() for person in persons_closest_to_room_center.values()]
    ys2 = [person['map_vs'].vector.y() for person in persons_closest_to_room_center.values()]

    if plot:
        plt.scatter(xs2, ys2, c='b')
        plt.show()

    # locations = zip(xs2, ys2)

    return persons_closest_to_room_center.values()


if __name__ == "__main__":
    import os.path
    import pprint
    import sys

    filename = os.path.expanduser(sys.argv[1])
    filename_base = os.path.splitext(filename)[0]

    with open(filename, 'br') as f:
        ppl_dicts = pickle.load(f)
    # ppl_dicts is a list of dicts {'rgb':..., 'person_detection':..., 'map_vs':...}

    clustered_ppl = cluster_people(ppl_dicts, room_center=np.array([6, 0]), plot=True)

    xs2 = [person['map_vs'].vector.x() for person in clustered_ppl]
    ys2 = [person['map_vs'].vector.y() for person in clustered_ppl]
    locations = zip(xs2, ys2)
    pprint.pprint(list(locations))

    with open(filename_base + '_kmeans_output.pickle', 'wb') as f:
        pickle.dump(list(clustered_ppl), f)
