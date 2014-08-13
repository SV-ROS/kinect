#!/usr/bin/env python

import sys
import rospy
from kinect_challenge.srv import *

def MSBMIOClient(action,landmarkID,runID):
    rospy.wait_for_service('MSBMIO')
    try:
        req = rospy.ServiceProxy('MSBMIO', MSBMIO)
        resp = req(action,landmarkID,runID)
        return resp.Status
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s map|start|end landmarkID runID"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        action = sys.argv[1]
        landmarkID= sys.argv[2]
        runID= sys.argv[3]
    else:
        print usage()
        sys.exit(1)
    print "Requesting Action=%s with LandmarkID=%s runID=%s"%(action,landmarkID,runID)
    print "Response = %s"%(MSBMIOClient(action,landmarkID,runID))