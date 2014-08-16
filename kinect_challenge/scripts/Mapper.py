#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
import tf
import geometry_msgs.msg
import shutil

import math
import os
import time
import sys

import subprocess

import MSBMIOClient

from visualization_msgs.msg import Marker

class Mapper():
    def __init__(self,mapPath,waypointFile, lmButtonIdx, msButtonIdx):
    
        self.mapPath=mapPath
        waypointFile=os.path.join(mapPath,waypointFile)
    
        if os.path.exists(waypointFile):
            bkFile =  waypointFile + ".%s.bak" % time.ctime()
            bkFile = bkFile.replace(":","_")
            print "Backing up old waypoint file '%s' to '%s'" % ( waypointFile,bkFile)
            shutil.move(waypointFile,bkFile)
            
        self.waypointFile =  waypointFile
        self.lmButtonIdx=lmButtonIdx
        self.msButtonIdx=msButtonIdx
        
        self.marker_pub = rospy.Publisher("visualization_marker",Marker)
       
        rospy.Subscriber("joy",Joy,self.JoyCallback)
        
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10.0)
        self.tf = None
        self.waypointNum =0
        
        

    def Run(self):
        while not rospy.is_shutdown():
            transformOK =False
            try:
                (trans,rot) = self.listener.lookupTransform('map','base_link',rospy.Time(0))
                transformOK =True
            except (tf.LookupException,tf.ConnectivityException, tf.ExtrapolationException):
                continue
                
            if transformOK:
                self.tf = (trans,rot)
                #print "got transform: %s, %s" %(str(trans),str(rot))
                #angular = 4 * math.atan2(trans[1],trans[0])
                #linear = 0.5 * math.sqrt(trans[0] **2 + trans[1] **2)
            
            self.rate.sleep()
        
        return 0
        
    def makeArrowMarker(self,tf,num):
        return self._makeMarker(tf,num)
             
    def makeTextMarker(self,tf,num):
        marker = self._makeMarker(tf,num)
        marker.type = marker.type= marker.TEXT_VIEW_FACING
        marker.id = num+100
        marker.pose.position.z = tf[0][2]+0.575
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.text=str(num)
        return marker
        
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
        marker.pose.position.z = tf[0][2]+0.5
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
    
    def JoyCallback(self,msg):
        keyState = msg.buttons[self.lmButtonIdx]
        if keyState==1:
            rospy.loginfo( "Saving landmark %d" % self.waypointNum)
            #print "Got Joy Message: %s" % str(msg)
            if self.tf != None:
            
                sts = MSBMIOClient.MSBMIOClient("map",str(self.waypointNum),"0")
                rospy.loginfo(  "MSBMIO returned %s" % sts)
                if sts =="OK":
                    self.marker_pub.publish(self.makeArrowMarker(self.tf,self.waypointNum))
                    self.marker_pub.publish(self.makeTextMarker(self.tf,self.waypointNum))
                    
                    rospy.loginfo(  "Saving waypoint %d: %s, %s" %(self.waypointNum,str(self.tf[0]),str(self.tf[1])))
                    with open(self.waypointFile,'a') as wpfh:
                        wpfh.write("%d: %s\n" % (self.waypointNum,str(self.tf)))
                        
                    self.waypointNum = self.waypointNum+1
                
                else:
                    rospy.loginfo(  "MS Benchmark Failed to record waypoint, please try again")
            else:
                rospy.logwarn("Can't save landmark, No Transform Yet")
                
        keyState = msg.buttons[self.msButtonIdx]
        if keyState==1:
            mapFile = "map_%s" % time.ctime().replace(":","_")
            rospy.loginfo( "Saving map to file '%s'" % mapFile)
            sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (self.mapPath,mapFile),shell=True)
            # Save a default copy just named "map" also
            sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (self.mapPath,"map"),shell=True)
            rospy.loginfo( "Save Map returned sts %d" % sts)
        
def usage():

    print "Invalid arguments, Usage:"
    return "%s mapPath waypointFile lmButtonIdx msButtonIdx" %sys.argv[0]

if __name__ == "__main__":

    sts = 0
    
    rospy.loginfo(  "Running Mapper")
    try:
    
        rospy.init_node('mapper')
        
        mapPath = ""
        waypointFile = ""
        
        lmButtonIdx = -1
        msButtonIdx = -1
                
        nargs = len(sys.argv)
        argOffset = 0
        
        if nargs > 1:
            #print sys.argv[1]
        
            if sys.argv[1].startswith('__name:='):
                argOffset = 2
                
        if sts ==0:
            if nargs >= 2+argOffset:
                mapPath = sys.argv[1+argOffset]

            if mapPath == "":
                mapPath = rospy.get_param("/Mapper/MapPath","")
                rospy.loginfo(  "Found param mapPath='%s'" % mapPath)
                
            mapPath = mapPath.strip()
            
            if mapPath == "":
                rospy.loginfo(  "Error: No mapPath specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "map and waypoints will be saved in: '%s'" % mapPath)
                 
        if sts ==0:
            if nargs >= 3+argOffset:
                waypointFile = sys.argv[2+argOffset]

            if waypointFile == "":
                waypointFile = rospy.get_param("/Mapper/WaypointFile","")
                rospy.loginfo(  "Found param Waypointfile='%s'" % waypointFile)
                
            waypointFile = waypointFile.strip()
            
            if waypointFile == "":
                rospy.loginfo(  "Error: No waypoints file specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "Waypoints will be saved to: '%s'" % waypointFile)
 
        if sts ==0:
            if nargs >= 4+argOffset:
                lmButtonIdx = int(sys.argv[3+argOffset])

            if lmButtonIdx == -1:
                lmButtonIdx = int(rospy.get_param("/Mapper/lmButtonIdx","-1"))
                
           
            if lmButtonIdx == -1:
                rospy.loginfo(  "No lmButtonIdx specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "Using lmButtonIdx %d" % lmButtonIdx)

        if sts ==0:
            if nargs >= 5+argOffset:
                msButtonIdx = int(sys.argv[4+argOffset])

            if msButtonIdx == -1:
                msButtonIdx = int(rospy.get_param("/Mapper/msButtonIdx","-1"))
                
           
            if msButtonIdx == -1:
                rospy.loginfo(  "No msButtonIdx specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "Using msButtonIdx %d" % msButtonIdx)
                
        if sts ==0:
                
            mapper = Mapper(mapPath,waypointFile,lmButtonIdx,msButtonIdx)
            sts = mapper.Run()

    except Exception as ex:
        rospy.loginfo(  "Mapper Crashed with exception: %s" % str(ex))
        sts = -1
        
    finally:
        rospy.loginfo(  "Mapper Finished")
        sys.exit(sts)











