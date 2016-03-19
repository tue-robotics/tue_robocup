#!/usr/bin/env python

import roslib
import rospy

from ed.srv import Query

from xmlrpclib import ServerProxy

def cyan(text):
    return text

class SyncClient():

    def __init__(self, server_ip,port):
        self.srv = rospy.Service('~query', Query, self.srv_callback)
        self.sp  = ServerProxy("http://%s:%d"%(server_ip,port))

    def srv_callback(self, req):
        rospy.loginfo("Request: [%s]"%cyan(req))

        # RPC request to server
        result = self.sp.get(req.ids, req.properties, req.since_revision)
        if not result:
            rospy.loginfo("Sync result: [%s]"%cyan("None"))
        else:
            rospy.loginfo("Sync client: synced data of size [%2f kb] with revision [%d]"%(len(str(result['human_readable'])) / 1000.0,result['new_revision']))
        return result

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('sync_client')
        if rospy.has_param('~ip'):
            ip = rospy.get_param('~ip')
            port = rospy.get_param('~port')
            client = SyncClient(ip, port)
            rospy.loginfo("Sync client that pushes changes initialized [connecting to server on ip %s]"%ip)
            rospy.spin()
        else:
            rospy.logerr("Sync client: no server ip set; please specify the local 'ip' parameter")
    except rospy.ROSInterruptException:
        pass
