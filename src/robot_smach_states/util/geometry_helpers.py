import math

import geometry_msgs.msg as gm

from ed.msg import EntityInfo

def isLeftOfLine(p, l):
	""" Checks whether point p is left of line l
		p: geometry_msgs.Point
		l: array of size two of geometry_msgs.Point. 
		Note that only x and y are evaluated
	""" 		
	A = l[0]
	B = l[1]
	if ( (B.x - A.x) * (p.y - A.y) - (B.y - A.y) * (p.x - A.x) ) > 0:
		return True
	else:
		return False
	#position = sign( (Bx-Ax)*(Y-Ay) - (By-Ay)*(X-Ax) )

def isPointInsideHull(p, chull):
	""" Checks whether point p is inside the convex hull ch
		p: geometry_msgs.Point
		ch: array of geometry_msgs.Point. Note that the order is supposed to be anti-clockwise
		Note that only p.x and p.y are evaluated
	"""

	''' Copy list '''
	ch = list(chull)

	''' Loop over lines of chull '''
	for i in xrange(len(ch) - 1):

		''' Check whether the point is left of the line '''
		if not isLeftOfLine(p, [ch[i], ch[i+1]]):
			return False

	return True

def onTopOff(e1, e2, ht=0.1):
	""" Checks whether entity e1 is on top of entity e2 
		e1: entity e1
		e2: entity e2
		ht: height threshold: the bottom of e1 and the top of e2 need to be within ht m
	"""
	''' First: check if e2 actually has a convex hull '''
	if len(e2.convex_hull) == 0:
		print 'Error, entity {0} has no convex hull'.format(e2.id)
		return False

	''' Second: check if center point of e1 is within convex hull of e2 '''
	if not isPointInsideHull(e1.center_point, e2.convex_hull):
		return False

	if math.fabs(e1.z_min-e2.z_max) > ht:
		return False

	return True

