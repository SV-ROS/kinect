#!/usr/bin/env python

import rospy
import actionlib

import tf
import geometry_msgs.msg
import shutil

import math
import os
import time
import sys
import re
import subprocess

import MSBMIOClient

from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

class Navigator():
    def __init__(self,mapPath,waypointFile,waypointTour):
    
        self.mapPath=mapPath
        waypointFile=os.path.join(mapPath,waypointFile)
        waypointTour=os.path.join(mapPath,waypointTour)

        self.waypointFile =  waypointFile
        self.waypointTour =  waypointTour
        
        self.marker_pub = rospy.Publisher("visualization_marker",Marker)
        self.ac = actionlib.SimpleActionClient("move_base" ,MoveBaseAction)
       
        while not self.ac.wait_for_server(rospy.Duration(5.0)):
            print "Waiting for Move Base Server"
           
        self.runID = int(time.time()*1000)
        
        self.rate = rospy.Rate(10.0)
 
        self.tourIdx =0
        
        self.waypoints = self.loadWaypoints(self.waypointFile)
        self.waypointTour = self.loadWaypointTour(self.waypointTour)
        
    def setGoal(self,idx):
        self.goal = MoveBaseGoal()
        
        tgt = self.waypointTour[idx][1]
        tf = self.waypoints[tgt]
        
        self.goal.target_pose.pose.position.x = tf[0][0]
        self.goal.target_pose.pose.position.y = tf[0][1]
        self.goal.target_pose.pose.position.z = tf[0][2]
        
        self.goal.target_pose.pose.orientation.x  = tf[1][0]
        self.goal.target_pose.pose.orientation.y  = tf[1][1]
        self.goal.target_pose.pose.orientation.z  = tf[1][2]
        self.goal.target_pose.pose.orientation.w  = tf[1][3]
        self.goal.target_pose.header.frame_id="map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        self.ac.send_goal(self.goal)
        
        return tgt
        
    def loadWaypointTour(self,waypointTour):
        tour = []

        with open(waypointTour,'r') as wptfh:
            wptxt = wptfh.readlines()
            
        for t in wptxt:
            if t.strip() != "":
                wpf,wpt = t.split(",")
                tour.append((int(wpf),int(wpt)))
            
        print "Loaded Tour Info:\n%s" % tour
        return tour

    def loadWaypoints(self,waypointFile):
    
        waypoints = []
        
        with open(waypointFile,'r') as wpfh:
            wptxt = wpfh.readlines()
            

        rex = re.compile("([0-9]+):\s*\(\((-?[0-9.]*),\s*(-?[0-9.]*),\s*(-?[0-9.]*)\)\s*,\s*\((-?[0-9.]*),\s*(-?[0-9.]*),\s*(-?[0-9.]*),\s*(-?[0-9.]*)\)\)")
        
        for wp in wptxt:
            #print "matching text '%s'" % wp.strip()
            if wp.strip() != "":
                match = rex.match(wp.strip())
                if match <> None:
                    waypoints.append(( ( float(match.group(2)),float(match.group(3)),float(match.group(4)) ),  ( float(match.group(5)), float(match.group(6)), float(match.group(7)), float(match.group(8))) ))
                    
                else:
                    print "invalid waypoint file, cannot parse text '%s'" % wp
    
        print "Found Waypoints %s" % waypoints
            
        return waypoints
    
    def Run(self):
        
        state =0
        wpIdx =self.waypointTour[self.tourIdx][0]
        goalTime = 300/5
        time.sleep(15)
        while not rospy.is_shutdown():
        
            if state==0:
                rospy.loginfo("Sending start %d" % wpIdx)
                sts=MSBMIOClient.MSBMIOClient("start",str(wpIdx),"%d" % self.runID)
                time.sleep(15)
                if sts =="OK":
                    rospy.loginfo("Sending goal %d" % self.waypointTour[self.tourIdx][1])
                    wpIdx=self.setGoal(self.tourIdx)
                    goalTime = 300/5
                    
                    state =1
                
            if state == 1:
                rospy.loginfo("waiting reach goal" )
                goalStatus = self.ac.wait_for_result(rospy.Duration(5))
                if goalStatus == True:
                    state = 10
                else:
                    goalTime--1;
                    if goalTime < 0:
                        state = 20
                        
                 
            if state ==10:
                rospy.loginfo("reached goal, send notify" )
                sts = MSBMIOClient.MSBMIOClient("end",str(wpIdx),"%d" % self.runID)
                if sts =="OK":
                    self.tourIdx= self.tourIdx+1
                    if self.tourIdx >= len( self.waypointTour):
                        self.tourIdx = 0
                    state = 0
                    rospy.loginfo("goal achieved, start next segment" )
                    
                else:
                    rospy.loginfo( "failed to record arrival, retry")
                    state = 0
                    
            if state==20:
                rospy.loginfo( "Failed to reach goal")
                state= 100
                
            if state ==100:
                pass
                
            self.createMarkers()
            self.rate.sleep()
        
        return 0
        
    def createMarkers(self):
        wpIdx =0
        for tf in self.waypoints:
            #print "Creating marker @ %s %d" % (str(tf),wpIdx)
            marker=self.makeArrowMarker(tf,wpIdx)
            self.marker_pub.publish(marker)
            marker=self.makeTextMarker(tf,wpIdx)
            self.marker_pub.publish(marker)
            wpIdx=wpIdx +1
        
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
    
    
        
def usage():

    print "Invalid arguments, Usage:"
    return "%s mapPath waypointFile waypointTour" %sys.argv[0]

if __name__ == "__main__":

    sts = 0
    
    print "Running Navigator"
    try:
    
        rospy.init_node('navigator')
        
        mapPath = ""
        waypointFile = ""
        waypointTour = ""
        
        nargs = len(sys.argv)
        argOffset = 0
        
        if nargs > 1:
            print sys.argv[1]
        
            if sys.argv[1].startswith('__name:='):
                argOffset = 2
                
        if sts ==0:
            if nargs >= 2+argOffset:
                mapPath = sys.argv[1+argOffset]

            if mapPath == "":
                mapPath = rospy.get_param("/Navigator/MapPath","")
                print "Found param mapPath='%s'" % mapPath
                
            mapPath = mapPath.strip()
            
            if mapPath == "":
                print "Error: No mapPath specified"
                print usage()
                sts= 1
            else:
                print "waypoints will be read from: '%s'" % mapPath
                 
        if sts ==0:
            if nargs >= 3+argOffset:
                waypointFile = sys.argv[2+argOffset]

            if waypointFile == "":
                waypointFile = rospy.get_param("/Navigator/WaypointFile","")
                print "Found param Waypointfile='%s'" % waypointFile
                
            waypointFile = waypointFile.strip()
            
            if waypointFile == "":
                print "Error: No waypoints file specified"
                print usage()
                sts= 1
            else:
                print "Waypoints file: '%s'" % waypointFile
 
        if sts ==0:
            if nargs >= 4+argOffset:
                waypointTour = sys.argv[3+argOffset]

            if waypointTour == "":
                waypointTour = rospy.get_param("/Navigator/WaypointTour","")
                print "Found param waypointTour='%s'" % waypointTour
                
            waypointTour = waypointTour.strip()
            
            if waypointTour == "":
                print "Error: No waypointTour file specified"
                print usage()
                sts= 1
            else:
                print "waypointTour file: '%s'" % waypointFile
 
 
        if sts ==0:
                
            nav = Navigator(mapPath,waypointFile,waypointTour)
            sts = nav.Run()

    except Exception as ex:
        print "Navigator Crashed with exception: %s" % str(ex)
        sts = -1
        
    finally:
        print "Navigator Finished"
        sys.exit(sts)











