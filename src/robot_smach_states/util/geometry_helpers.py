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


