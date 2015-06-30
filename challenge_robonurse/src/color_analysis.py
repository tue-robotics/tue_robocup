#!/usr/bin/python

"""This is a bit of magic :-)
Basically, the colors o the pixels in an image are clustered into NUM_CLUSTERS clusters using K-means (i.e. K == NUM_CLUSTERS).
With this, we know the most common RGB values (i.e. vector in 3D-space), but not yet an actual human-understandable color name for them.

To actually get a color name, we borrow color names from matplotlib.
This library has a mapping of colornames to RGB-values.
To give most common RGB values a name, we use the Euclidian distance between a color vector with a known name, and a cluster from k-means. """

import struct
from PIL import Image
import scipy
import scipy.misc
import scipy.cluster

import matplotlib
import numpy as np
import operator

NUM_CLUSTERS = 5


def replace_color(img, src, dst):
    # img = img.convert("RGBA")
    datas = img.getdata()

    newData = []
    for item in datas:
        if item[0] == src[0] and item[1] == src[1] and item[2] == src[2]:
            newData.append(dst) 
        else:
            newData.append(item)

    img.putdata(newData)
    return img

def vector2hex(vector):
    colour = '#'+''.join(chr(c) for c in vector).encode('hex')
    return colour

def hex2vector(hexcolor):
    return matplotlib.colors.hex2color(hexcolor)

name2hex = {"blue":"#0000FF",
            "green":"#008000",
            "red":"#FF0000",
            "blue-greenish":"#00FFFF", #"cyan"
            "fuchsia":"#FF00FF",
            "yellow":"#FFFF00",
            "black":"#000000",
            "white":"#FFFFFF",
            "brown":"#964B00",
            "pink":"#FFCBDB",
            "orange":"#FF7F00"} #matplotlib.colors.cnames
referencevector2name = {hex2vector(_hex):name for name,_hex in name2hex.iteritems()} #Create a dictionary/map from reference-color's 3D-vector to its colorname
# print referencevector2name
# import ipdb; ipdb.set_trace()

def analyze(im):
    # print 'reading image'

    im = im.resize((150, 150))      # optional, to reduce time

    im = replace_color(im, (255, 255, 255), (0,0,0)) #If it's fully white, replace it with black

    ar = scipy.misc.fromimage(im)
    shape = ar.shape
    ar = ar.reshape(scipy.product(shape[:2]), shape[2])

    # print 'finding clusters'
    codes, dist = scipy.cluster.vq.kmeans(ar, NUM_CLUSTERS)
    # print 'cluster centres:\n', codes

    vecs, dist = scipy.cluster.vq.vq(ar, codes)         # assign codes
    counts, bins = scipy.histogram(vecs, len(codes))    # count occurrences

    vector2count = dict(zip([tuple(vector) for vector in codes], counts))
    name2count = {describe_color(vector2hex(vector)):count for vector,count in vector2count.iteritems()}
    # print name2count

    color_popularity_sorted = sorted(name2count.items(), key=operator.itemgetter(1), reverse=True)
    # import ipdb; ipdb.set_trace()

    #If the most popular color is black, than we selected the background. 
    #   This must be discarded so we pick the most popular color in the object
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
    """For a hexadecimal color, e.g. #FF00FF, return the closest color name.
    E.g. 
    >>> describe_color("#FF0001")
    'red'
    >>> describe_color("#FFFFFE")
    'white'
    >>> describe_color("#008001")
    'green'
    >>> describe_color("#0000FE")
    'blue'"""
    queryvector = hex2vector(hexcolor) #Convert the queried color to a point in 3D space
    distance2name = {vector_distance(queryvector, referencevector):name for referencevector, name in referencevector2name.iteritems()} #Map the distance between a reference point and the queried point to colornames.

    closest_color = min(distance2name.items(), key=operator.itemgetter(0)) # Now get the (distance,colorname)-pair with the smallest distance
    
    # print "Queried color is closest to {} with distance {}".format(closest_color[1], closest_color[0])

    # import ipdb; ipdb.set_trace()
    return closest_color[1] #Of that pair, return only the colorname

def vector_distance(a, b):
    """a, b are 3-tuples"""
    a = np.array(a)
    b = np.array(b)
    dist = np.linalg.norm(a-b)
    return dist

if __name__ == "__main__":
    import doctest
    doctest.testmod()

    import sys
    try:
        filename = sys.argv[1]
        im = Image.open(filename)
        most_frequent_colour = analyze(im)
        print most_frequent_colour
    except Exception, e:
        print "You can specify a path to an image to analyse"
