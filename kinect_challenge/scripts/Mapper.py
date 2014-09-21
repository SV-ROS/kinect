#!/usr/bin/env python


#######################
#
#   Kinect Challenge Mapper Node
#
#   Manages the mapping phase for the Kinect Challenge.
#
#   Manages the creation of the waypoint.wp file (and backups)
#   Handles saving of the grid 2d MAP and rtabmap.db files
#
#   Sends message to MSBM PC when a landmark is created
#
#   Creates Markers for visualization in Rviz when a waypoint is recorded.
#
#   Generates voice synth. feedback to assist operator.
#
#   Waypoints locations are saved based on the current pose defined in the /map to /base_footprint transform at the time of recording the WP.
#    The pose is forced to be in the 2d map plane
#
#   Uses joystick button clicks to trigger the waypoint creation and map saving
#
#   Author: Ralph Gnauck
#   License: BSD
#
#
#   Created for the SV ROS entry in the 2014 IROS Microsoft Kinect Challenge Competition
#
#######################


import rospy
import tf
from sensor_msgs.msg import Joy

import geometry_msgs.msg
import shutil

import math
import os
import time
import sys

import subprocess

import MSBMIOClient
import SayText
import os
from tf.transformations import *
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty

###############################################
#
#   Class for the Mapper.py Ros node
# 
#
class Mapper():
    #
    # Constructor
    #
    #   mapPath: path where the map and waypoint file will be saved
    #
    #   waypointFile: name of the waypoint file default = 'waypoint.wp'
    #
    #   lmButtonIdx:   joystick button index for button to trigger landmark creation
    #   msButtonIdx:   joystick button index for button to trigger saving of the map and rtabmap.db
    #
    
    def __init__(self,mapPath,waypointFile, lmButtonIdx, msButtonIdx):
    
        self.mapPath=mapPath
        waypointFile=os.path.join(mapPath,waypointFile)

        # backup old waypoint file if one already exists (adds timestamp to file name)
        if os.path.exists(waypointFile):
            bkFile =  waypointFile + ".%s.bak" % time.ctime()
            bkFile = bkFile.replace(":","_")
            print "Backing up old waypoint file '%s' to '%s'" % ( waypointFile,bkFile)
            shutil.move(waypointFile,bkFile)

        # save parmas
        self.waypointFile =  waypointFile
        self.lmButtonIdx=lmButtonIdx
        self.msButtonIdx=msButtonIdx

        # create publisher for visualization markers for waypoints
        self.marker_pub = rospy.Publisher("visualization_marker",Marker)

        # we listen to the joystick buttons
        rospy.Subscriber("joy",Joy,self.JoyCallback)

        # need the transform from  /map to /base_footprint
        self.listener = tf.TransformListener()

        
        self.rate = rospy.Rate(10.0)
        
        self.tf = None  # not seen a transform yet
        self.waypointNum =0 # start from waypoint 0
        
        self.vocalizer = SayText.SayText()  # set up text to speech for user feedback
        
    # speak a message
    def say(self,msg):
        self.vocalizer.say(msg)

    # main run loop
    def Run(self):
        
        self.say("Mapper Ready")

        while not rospy.is_shutdown():
            transformOK =False
            try:
                # check for transform
                (trans,rot) = self.listener.lookupTransform('map','base_footprint',rospy.Time(0))

                # clean up pose, force to be on and parallel to map plane
                trans=(trans[0],trans[1],0) # z=0

                # roll and pitch = 0
                (r,p,y) = euler_from_quaternion(rot)
                qt = quaternion_from_euler(0,0,y)
                rot=(qt[0],qt[1],qt[2],qt[3])
                
                transformOK =True
                
            except (tf.LookupException,tf.ConnectivityException, tf.ExtrapolationException):
                pass # ignore these exceptions
                
            if transformOK: # got good transform so save it in case we save a waypoint
                self.tf = (trans,rot)
            
            self.rate.sleep()
        
        return 0

    # create a visualization marker arrow at the tf location (in /map frame) and with id num
    def makeArrowMarker(self,tf,num):
        return self._makeMarker(tf,num)

    # create a text marker at offset above(0.575m)  tf(in /map frame)  with id num+100 and text = str(num)
    def makeTextMarker(self,tf,num):
        
        marker = self._makeMarker(tf,num) # create an arrow first
        
        marker.type = marker.type= marker.TEXT_VIEW_FACING # now change it to text
        marker.id = num+100 # update the ID
        marker.pose.position.z = marker.pose.position.z +0.075 # move it above the arrow
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.text=str(num)  # set the text string
        return marker

    # create a visualization marker arrow  at offset above(0.5m) tf (in /map frame) location and with id num
    def _makeMarker(self,tf,num): 
    
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "kinect_challenge"
        marker.id = num
        
        marker.type= marker.ARROW
        marker.action = marker.ADD
        
        marker.pose.position.x = tf[0][0]
        marker.pose.position.y = tf[0][1]
        marker.pose.position.z = 0.5


        
        marker.pose.orientation.x  = tf[1][0]
        marker.pose.orientation.y  = tf[1][1]
        marker.pose.orientation.z  = tf[1][2]
        marker.pose.orientation.w  = tf[1][3]
           
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.3
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration()
        
        return marker

    # handle joystick buttons
    def JoyCallback(self,msg):

        # is lmButtonIdx a  valid buttonID
        if self.lmButtonIdx >= 0 and self.lmButtonIdx < len(msg.buttons):
            keyState = msg.buttons[self.lmButtonIdx] # ok so return button state
        else:
            keyState = -1 # ignore this button

        if keyState==1:  # Save landmark button pressed
            self.say("Save Waypoint button pressed") 
            rospy.loginfo( "Saving landmark %d" % self.waypointNum)
            #print "Got Joy Message: %s" % str(msg)
            
            if self.tf != None: # do we have a valid transform to use
                # send message to MSBM PC 
                sts = MSBMIOClient.MSBMIOClient("map",str(self.waypointNum),"0")
                rospy.loginfo(  "MSBMIO returned %s" % sts)
                
                if sts =="OK": # Landmark accepted by MSBM so save in the waypoint file and create a visualization marker
                    self.marker_pub.publish(self.makeArrowMarker(self.tf,self.waypointNum))
                    self.marker_pub.publish(self.makeTextMarker(self.tf,self.waypointNum))
                    
                    rospy.loginfo(  "Saving waypoint %d: %s, %s" %(self.waypointNum,str(self.tf[0]),str(self.tf[1])))

                    # save waypoint pose in file
                    with open(self.waypointFile,'a') as wpfh:
                        wpfh.write("%d: %s\n" % (self.waypointNum,str(self.tf)))

                    self.say("Waypoint %d saved ok" % self.waypointNum)

                    # increment waypoint number for next waypoint
                    self.waypointNum = self.waypointNum+1

                
                else:
                    rospy.loginfo(  "MS Benchmark Failed to record waypoint, please try again")
                    self.say("Waypoint Failed" )
            else:
                rospy.logwarn("Can't save landmark, No Transform Yet")
                self.say("Waypoint Failed" ) 

        # is save map button idx ok
        if self.msButtonIdx >= 0 and self.msButtonIdx < len(msg.buttons):
            keyState = msg.buttons[self.msButtonIdx] # get state of save map button
        else:
            keyState = -1

        if keyState==1: # save map button pressed
            self.saveMap()
            

    def saveMap(self):
        self.say("Saving map" )


        # Put rtabmap in localization mode so it does not continue to update map after we save it
        rtmsp = rospy.ServiceProxy('rtabmap/set_mode_localization',Empty())
        rtmsp()
        
        
        # create timestamp for map file
        timeStamp = "%s" % time.ctime().replace(":","_").replace(" ","_")
        mapFile = "map_%s" % timeStamp
        dbFile = "kc_map_%s.db" % timeStamp

        # save map file using map_server
        rospy.loginfo( "Saving map to file '%s'" % mapFile)
        sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (self.mapPath,mapFile),shell=True)

        # Save a default copy just named "map" also
        if sts == 0:
            sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (self.mapPath,"map"),shell=True)

        # make backup of the rtabmap DB
        dbPath = os.environ["HOME"]
        if sts == 0:
            sts = subprocess.call('cp "%s/.ros/rtabmap.db" "%s/.ros/%s"' % (dbPath,dbPath,dbFile),shell=True)

        rospy.loginfo( "Save Map returned sts %d" % sts)

        # let user know what happened
        if sts==0:
            self.say("Map Saved ok" )
        else:
            self.say("Save map failed. Status = %d" % sts)

        
def usage():

    print "Invalid arguments, Usage:"
    return "%s" %sys.argv[0]


# main function
# get params and start the node
if __name__ == "__main__":

    sts = 0
    
    rospy.loginfo(  "Running Mapper")
    try:
    
        rospy.init_node('mapper')
        
        mapPath = ""
        waypointFile = ""
        
        lmButtonIdx = -1
        msButtonIdx = -1
                

        if sts ==0:
            if mapPath == "":
                mapPath = rospy.get_param("Mapper/MapPath","")
                rospy.loginfo(  "Found param mapPath='%s'" % mapPath)
                
            mapPath = mapPath.strip()
            
            if mapPath == "":
                rospy.loginfo(  "Error: No mapPath specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "map and waypoints will be saved in: '%s'" % mapPath)
                 
        if sts ==0:
            if waypointFile == "":
                waypointFile = rospy.get_param("Mapper/WaypointFile","")
                rospy.loginfo(  "Found param Waypointfile='%s'" % waypointFile)
                
            waypointFile = waypointFile.strip()
            
            if waypointFile == "":
                rospy.loginfo(  "Error: No waypoints file specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "Waypoints will be saved to: '%s'" % waypointFile)
 
        if sts ==0:
            if lmButtonIdx == -1:
                lmButtonIdx = int(rospy.get_param("Mapper/lmButtonIdx","-1"))
                
           
            if lmButtonIdx == -1:
                rospy.loginfo(  "No lmButtonIdx specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "Using lmButtonIdx %d" % lmButtonIdx)

        if sts ==0:
            if msButtonIdx == -1:
                msButtonIdx = int(rospy.get_param("Mapper/msButtonIdx","-1"))
                
           
            if msButtonIdx == -1:
                rospy.loginfo(  "No msButtonIdx specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "Using msButtonIdx %d" % msButtonIdx)
                
        #if sts ==0:
        # all ok so start the node
        mapper = Mapper(mapPath,waypointFile,lmButtonIdx,msButtonIdx)
        sts = mapper.Run()

    except Exception as ex:
        rospy.loginfo(  "Mapper Crashed with exception: %s" % str(ex))
        sts = -1
        
    finally:
        rospy.loginfo(  "Mapper Finished")
        sys.exit(sts)











