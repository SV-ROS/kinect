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
# see service MSBMIOSvs Class for description of action, landmark_id, run_id and returned responses
def MSBMIOClient(action, landmark_id, run_id):
    rospy.wait_for_service('MSBMIO')
    try:
        req = rospy.ServiceProxy('MSBMIO', MSBMIO)
        resp = req(action, landmark_id, run_id)
        return resp.status
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s map|start|end landmark_id run_id"%sys.argv[0]


# can just run it from the command line
# pass action, landmark_id and RunID as arguments
# sends a message and prints the response, then exist
if __name__ == "__main__":
    if len(sys.argv) == 4:
        action = sys.argv[1]
        landmark_id= sys.argv[2]
        run_id= sys.argv[3]
    else:
        print usage()
        sys.exit(1)
    print "Requesting action=%s with landmark_id=%s run_id=%s"%(action, landmark_id, run_id)
    print "Response = %s"%(MSBMIOClient(action, landmark_id, run_id))