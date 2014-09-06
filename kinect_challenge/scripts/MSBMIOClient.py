#!/usr/bin/env python

#######################
#
#   Kinect Challenge MSBMIOClient helper module (Microsoft Benchmark IO Service Client)
#
#   Python API for the the MSBMIOSvs interface to the Microsoft Benchmark computer for the Kinect Challenge.
#
#   Import this module and call MSBMIOClient method to send a request to the MSBM PC
#
#  Author: Ralph Gnauck
#  License: BSD
#
#
#   Created for the SV ROS entry in the 2014 IROS Microsoft Kinect Challenge Competition
#
#######################
import sys
import rospy
from kinect_challenge.srv import *

# send message to MSBM PC
# see service MSBMIOSvs Class for description of action, landmarkID, runID and returned responses
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


# can just run it from the command line
# pass action, landmarkID and RunID as arguments
# sends a message and prints the response, then exist
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