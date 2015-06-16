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

def replace_black_with_transpart(img):
    img = img.convert("RGBA")
    datas = img.getdata()

    newData = []
    for item in datas:
        if item[0] == 0 and item[1] == 0 and item[2] == 0:
            newData.append((255, 255, 255, 0))
        else:
            newData.append(item)

    img.putdata(newData)
    return img

def vector2hex(vector):
    colour = '#'+''.join(chr(c) for c in vector).encode('hex')
    return colour

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

    vector2count = dict(zip([tuple(vector) for vector in codes], counts))
    name2count = {describe_color(vector2hex(vector)):count for vector,count in vector2count.iteritems()}
    print name2count

    color_popularity_sorted = sorted(name2count.items(), key=operator.itemgetter(1), reverse=True)
    # import ipdb; ipdb.set_trace()

    if color_popularity_sorted[0][0] == "black":
        del color_popularity_sorted[0]

    return color_popularity_sorted[0][0]

    # index_max = scipy.argmax(counts)                    # find most frequent
    # peak = codes[index_max]
    # colour = vector2hex(peak)
    # print 'most frequent is %s (%s)' % (peak, colour)

    # most_frequent_colour = describe_color(colour)
    # print "This is closest to %s"%most_frequent_colour

def describe_color(hexcolor):
    queryvector = hex2vector(hexcolor)
    referencevector2name = {hex2vector(_hex):name for name,_hex in matplotlib.colors.cnames.iteritems()}
    distance2name = {vector_distance(queryvector, referencevector):name for referencevector, name in referencevector2name.iteritems()}
    closest_color = min(distance2name.items(), key=operator.itemgetter(0))
    
    # print "Queried color is closest to {} with distance {}".format(closest_color[1], closest_color[0])

    # import ipdb; ipdb.set_trace()
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
    most_frequent_colour = analyze(im)
    print most_frequent_colour
