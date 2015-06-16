#!/usr/bin/python

import struct
from PIL import Image
import scipy
import scipy.misc
import scipy.cluster

import matplotlib
import numpy as np
import operator

NUM_CLUSTERS = 5

def analyze(im):
    print 'reading image'

    im = im.resize((150, 150))      # optional, to reduce time
    ar = scipy.misc.fromimage(im)
    shape = ar.shape
    ar = ar.reshape(scipy.product(shape[:2]), shape[2])

    print 'finding clusters'
    codes, dist = scipy.cluster.vq.kmeans(ar, NUM_CLUSTERS)
    print 'cluster centres:\n', codes

    vecs, dist = scipy.cluster.vq.vq(ar, codes)         # assign codes
    counts, bins = scipy.histogram(vecs, len(codes))    # count occurrences

    index_max = scipy.argmax(counts)                    # find most frequent
    peak = codes[index_max]
    colour = ''.join(chr(c) for c in peak).encode('hex')
    print 'most frequent is %s (#%s)' % (peak, colour)

    most_frequent_colour = describe_color(colour)
    print "This is closest to %s"%most_frequent_colour

def describe_color(hexcolor):
    queryvector = hex2vector("#"+hexcolor)
    referencevector2name = {hex2vector(_hex):name for name,_hex in matplotlib.colors.cnames.iteritems()}
    distance2name = {vector_distance(queryvector, referencevector):name for referencevector, name in referencevector2name.iteritems()}
    closest_color = min(distance2name.items(), key=operator.itemgetter(0))
    
    print "Queried color is closest to {} with distance {}".format(closest_color[1], closest_color[0])

    import ipdb; ipdb.set_trace()
    return closest_color[1]

def vector_distance(a, b):
    """a, b are 3-tuples"""
    a = np.array(a)
    b = np.array(b)
    dist = np.linalg.norm(a-b)
    return dist

def hex2vector(hexcolor):
    return matplotlib.colors.hex2color(hexcolor)


if __name__ == "__main__":
    im = Image.open('image.jpg')
    analyze(im)