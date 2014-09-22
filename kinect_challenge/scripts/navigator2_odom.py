#!/usr/bin/env python

#######################
#
#   Kinect Challenge Navigator Node
#
#   Manages the navigation phase for the Kinect Challenge.
#
#   Runs a state machine to control the autonomous navigation phase
#
#   Steps through the tour list and drives each leg starting from one landmark and moving to the next.
#   At the end of the tour it goes back to the start and loops continuously
#   Sends the start and end messages to the MSBM PC at the start and end of a leg.
#
#   Uses the waypoints.wp and tour.wpt files to define the tour
#
#   see code for file format descriptions
#
#   Author: Ralph Gnauck
#   License: BSD
#
#
#   Created for the SV ROS entry in the 2014 IROS Microsoft Kinect Challenge Competition
#
#######################

import rospy
import actionlib

import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist
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
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from random import random

import SayText

####################################################
#
#   Class for Navigator Ros Node
#

class Navigator():
    #
    # Constructor
    #
    #   mapPath:  path for the waypoint and tour files
    #   waypointFile: name of waypoint file
    #   waypointTour: name of tour file
    #
    #   publishes arrow marks for waypoint visualization in rviz
    #
    #   uses action lib to direct move_base to handle the navigation from waypoint to waypoint
    #
    def __init__(self,mapPath,waypointFile,waypointTour):

        # Save params
        self.mapPath=mapPath
        waypointFile=os.path.join(mapPath,waypointFile)
        waypointTour=os.path.join(mapPath,waypointTour)

        self.waypointFile =  waypointFile
        self.waypointTour =  waypointTour

        # setup waypoint marker publisher
        self.marker_pub = rospy.Publisher("visualization_marker",Marker)

        # create action client
        self.ac = actionlib.SimpleActionClient("move_base" ,MoveBaseAction)

        # wait for move_base server to be ready
        while not self.ac.wait_for_server(rospy.Duration(5.0)) and not rospy.is_shutdown():
            rospy.loginfo(  "Waiting for Move Base Server")
        
        # variables to determine if the robot is stuck
        self.odom_timer = 5
        self.odom_start = rospy.Time.now()
        self.stuck = False
            
        # subscribe to the /rosout topic so we can detect when the robot is stuck
        rospy.Subscriber("/p3dx/odom", Odometry, self.odomCallback)

        # publisher for Twist commands
        self.cmd_vel_pub = rospy.Publisher("/p3dx/cmd_vel" ,Twist)

        # MSBM expects a runID during navigation, use current time in seconds past epoch for each run
        self.runID = int(time.time()*1000)

        self.rate = rospy.Rate(10.0)

        self.tourIdx =0 # start at first leg of the tour
        
        self.max_tries = 3 # number of attempts to replan a path to a waypoint
        self.n_tries = 0

        # read the waypoints and tour files
        self.waypoints = self.loadWaypoints(self.waypointFile)
        self.waypointTour = self.loadWaypointTour(self.waypointTour)
        


        
        self.vocaliser = SayText.SayText()

    # say stuff
    def say(self,msg):
        self.vocaliser.say(msg)

    # send goal to move_base
    # idx is line number in tour file, goal set to waypoint of 'to' location in tour file at line idx.
    # assumes we are currently at waypoint of 'from' location in tour file at line idx
    def setGoal(self,idx):
        
        self.goal = MoveBaseGoal()

        # get waypoint pose
        tgt = self.waypointTour[idx][1]
        tf = self.waypoints[tgt]

        # create goal pose
        self.goal.target_pose.pose.position.x = tf[0][0]
        self.goal.target_pose.pose.position.y = tf[0][1]
        self.goal.target_pose.pose.position.z = tf[0][2]

        self.goal.target_pose.pose.orientation.x  = tf[1][0]
        self.goal.target_pose.pose.orientation.y  = tf[1][1]
        self.goal.target_pose.pose.orientation.z  = tf[1][2]
        self.goal.target_pose.pose.orientation.w  = tf[1][3]
        
        self.goal.target_pose.header.frame_id="map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # set the goal
        self.ac.send_goal(self.goal)

        # return the target waypoint number
        return tgt

    # read the tour file
    # 
    # each line has two comma separated integers,  fromWP, toWP
    #
    #  fills in tour list with tuples of (to,from) pairs and returns it
    #
    def loadWaypointTour(self,waypointTour):
        tour = []

        with open(waypointTour,'r') as wptfh:
            wptxt = wptfh.readlines()

        for t in wptxt:
            if t.strip() != "":
                wpf,wpt = t.split(",")
                tour.append((int(wpf),int(wpt)))

        rospy.loginfo(  "Loaded Tour Info:\n%s" % tour)
        return tour


    # read waypoint files
    #
    # each line has one waypoint in format:
    #
    #   wpIdx : ((lx,ly,lz),(ox,oy,oz,ow))
    #
    #   wpIdx: waypoint index(id)
    #
    #   lx,ly,lz:     x,y,z of waypoint location in map frame
    #   ox,oy,oz,ow:  orientation of pose of waypoint(quaternion components) in map frame
    #
    def loadWaypoints(self,waypointFile):

        waypoints = []

        with open(waypointFile,'r') as wpfh:
            wptxt = wpfh.readlines()

        # reg ex to parse the fields of the line
        rex = re.compile("([0-9]+):\s*\(\((-?[0-9.]*),\s*(-?[0-9.]*),\s*(-?[0-9.]*)\)\s*,\s*\((-?[0-9.]*),\s*(-?[0-9.]*),\s*(-?[0-9.]*),\s*(-?[0-9.]*)\)\)")

        for wp in wptxt:
            #print "matching text '%s'" % wp.strip()
            if wp.strip() != "":
                match = rex.match(wp.strip())
                if match <> None:
                    waypoints.append(( ( float(match.group(2)),float(match.group(3)),float(match.group(4)) ),  ( float(match.group(5)), float(match.group(6)), float(match.group(7)), float(match.group(8))) ))

                else:
                    rospy.loginfo(  "Invalid waypoint file, cannot parse text '%s'" % wp)

        rospy.loginfo(  "Found Waypoints %s" % waypoints)

        return waypoints

    # main loop
    def Run(self):

        state =0 # initial state of state machine

        # get starting node from first tour leg
        wpIdx =self.waypointTour[self.tourIdx][0]
        
        goalTime = 300/5  # allow 300 seconds (in 5 second chunks to complete the leg)
        #time.sleep(15)    # allow time to localize and drive to start node !!! TODO: make this triggered by Joystick button

        # thunderbirds are 'GO'
        self.say("Navigator Ready")

        while not rospy.is_shutdown():
            
            # publish the waypoint markers
            self.createMarkers()

            if state == 0: # Send start node and start a leg 
            
                rospy.loginfo("Sending start %d" % wpIdx)

                # send message to MSBM indicating we are starting a leg from 'to' node (starts the clock)
                sts=MSBMIOClient.MSBMIOClient("start",str(wpIdx),"%d" % self.runID) # always returns OK
                
                self.say("Starting from waypoint %d"%wpIdx)
                time.sleep(5)
                
                if sts =="OK": # should always be ok on start leg

                    
                    rospy.loginfo("Sending goal %d" % self.waypointTour[self.tourIdx][1])

                    # tell move base the end waypoint and set it as current goal
                    wpIdx=self.setGoal(self.tourIdx)
                    
                    self.say("Setting goal waypoint %d"%wpIdx)

                    # setup leg timer
                    goalTime = 300/5
                    state =1  # next state


            if state == 1: # monitor goal progress
            
                rospy.loginfo("waiting to reach goal" )
                    
                goalStatus = self.ac.wait_for_result(rospy.Duration(5)) # wait for 5 seconds to see if we reach the goal yet

                if goalStatus == True:  
                    state = 10 # we got there or aborted
                else:
                    goalTime = goalTime-1 # 5 seconds is up, count down main loop timer
                    if goalTime < 0: # If total time out finished we failed to get there, goto error state
                        state = 20
                        
                if self.stuck:
                    rospy.loginfo("Robot is stuck.  Will try backing up and turning...")
                     
                    # cancel the current waypoint goal
                    self.ac.cancel_all_goals()
                    self.ac.wait_for_result(rospy.Duration(15))
 
                    # back up a bit assuming we've run into something going forward
                    cmd_vel = Twist()
                    cmd_vel.linear.x = -0.05
                                          
                    escape_interval = 3.0
                    tick = 0.2
                    clock = 0.0
                                         
                    while clock < escape_interval:
                        self.cmd_vel_pub.publish(cmd_vel)
                        rospy.sleep(tick)
                        clock += tick
                      
                    self.cmd_vel_pub.publish(Twist())
 
                    # now rotate randomly to the left or right
                    cmd_vel = Twist()
                    cmd_vel.angular.z = math.copysign(0.4, random() - 1.0)
                     
                    escape_interval = 3.0
                    tick = 0.2
                    clock = 0.0
                                         
                    while clock < escape_interval:
                        self.cmd_vel_pub.publish(cmd_vel)
                        rospy.sleep(tick)
                        clock += tick
                      
                    self.cmd_vel_pub.publish(Twist())
 
                    # assume we got unstuck and try the goal again
                    self.stuck = False
                    state = 0
                    continue


            if state == 10:  # we finished the goal
            
                goalSts = self.ac.get_state() # get actual completion status
                print "GOAL STATUS:", str(goalSts)
                
                if goalSts == GoalStatus.ABORTED:
                    self.n_tries += 1
                    if self.n_tries >= self.max_tries:
                        rospy.loginfo("Giving up.")
                        self.n_tries = 0
                        state = 20
                    else:
                        rospy.loginfo("Goal aborted.")
                        rospy.loginfo("Attempt " + str(self.n_tries + 1) + " of " + str(self.max_tries))
                        state = 0
                
                elif goalSts == GoalStatus.SUCCEEDED:  # did we really get there
                                        
                    rospy.loginfo("Reached goal, sending notify" )

                    # notify MSBM we are at target waypoint
                    sts = MSBMIOClient.MSBMIOClient("end",str(wpIdx),"%d" % self.runID)
                    
                    if sts =="OK":  # MSBM accepted waypoint
                        rospy.loginfo("Goal achieved, start next segment" )
                        self.say("Reached goal waypoint %d"%wpIdx)
                        self.setNextWaypoint() # get next leg
                        state = 0 # go back to send next start notification
                        

                    else: # MSBM did not like us, try again
                        rospy.loginfo( "Failed to record arrival, retry")
                        self.say("Failed to notify benchmark that goal waypoint %d reached. Will retry"%wpIdx)
                       
                        state = 0

                else: # move base failed to get us there
                    rospy.loginfo( "Failed to reach goal %d. Status '%d'" % (wpIdx,goalSts))
                    self.say("Failed to reach goal waypoint %d. Status is %d" %(wpIdx,goalSts))
                    state = 100 # skip to next wp

                    
            if state == 20: # did not reach the goal in time, give up and move to next WP
                self.ac.cancel_goal()
                rospy.loginfo( "Failed to reach goal %d in designated time." % wpIdx)
                self.say("Failed to reached goal waypoint %d in designated time"%wpIdx)
                state = 100

                
            if state == 100: # skip to next WP
                rospy.loginfo( "Trying next goal")
                self.say( "Trying next goal")
                self.setNextWaypoint()# skip WP and try next one
                state = 0
                

            self.rate.sleep()

        return 0

    # get next leg of the tour
    # validates that the to and from waypoints are valid waypoints in the waypoints list
    def setNextWaypoint(self):

        # get next leg, loop to start once we go past the end
        self.tourIdx= self.tourIdx+1
        if self.tourIdx >= len( self.waypointTour):
            self.tourIdx = 0

        # get num waypoints
        numWP = len(self.waypoints)

        # get leg and to and from  wp
        ts=self.waypointTour[self.tourIdx]
        fromWP = ts[0]
        toWP = ts[1]

        # check from wp is valid
        if fromWP >= numWP :
            msg = "Invalid from waypoint %d in tour step %d, resetting tour to beginning" % (fromWP,self.tourIdx)
            rospy.loginfo(msg)
            self.say(msg)
            self.tourIdx = 0

        # check to wp is valid
        if toWP >= numWP :
            msg = "Invalid to waypoint %d in tour step %d, resetting tour to beginning" % (toWP,self.tourIdx)
            rospy.loginfo(msg)
            self.say(msg)
            self.tourIdx = 0

    # create the visualization markers for the waypoints and publish them
    def createMarkers(self):
        wpIdx =0
        for tf in self.waypoints:
            #print "Creating marker @ %s %d" % (str(tf),wpIdx)
            marker=self.makeArrowMarker(tf,wpIdx)
            self.marker_pub.publish(marker)
            marker=self.makeTextMarker(tf,wpIdx)
            self.marker_pub.publish(marker)
            wpIdx=wpIdx +1

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

    def odomCallback(self, msg):
        now = rospy.Time.now()
    
        rospy.loginfo(abs(msg.twist.twist.linear.x))
        if abs(msg.twist.twist.linear.x) < 0.05:
            if (now - self.odom_start) >= rospy.Duration(self.odom_timer):
                rospy.loginfo("STUCK!!!!!!!!!!!!!!!!")
                self.stuck = True
                return
        else:
            self.odom_start = now  
            rospy.loginfo("NOT!!!!!!!!!!!!!!!!")
            self.stuck = False

def usage():

    rospy.init_node( "Invalid arguments, Usage:")
    return "%s" %sys.argv[0]

# main function
#
# reads the params and creates the Navigator node
if __name__ == "__main__":

    sts = 0

    rospy.loginfo(  "Running Navigator")
    try:

        rospy.init_node('navigator')

        mapPath = ""
        waypointFile = ""
        waypointTour = ""


        if sts ==0:


            if mapPath == "":
                mapPath = rospy.get_param("/Navigator/MapPath","")
            rospy.loginfo(  "Found param mapPath='%s'" % mapPath)

            mapPath = mapPath.strip()

            if mapPath == "":
                rospy.loginfo(  "Error: No mapPath specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "waypoints will be read from: '%s'" % mapPath)

        if sts ==0:

            if waypointFile == "":
                waypointFile = rospy.get_param("/Navigator/WaypointFile","")
            rospy.loginfo(  "Found param Waypointfile='%s'" % waypointFile)

            waypointFile = waypointFile.strip()

            if waypointFile == "":
                rospy.loginfo(  "Error: No waypoints file specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "Waypoints file: '%s'" % waypointFile)

        if sts ==0:

            if waypointTour == "":
                waypointTour = rospy.get_param("/Navigator/WaypointTour","")
                
            rospy.loginfo(  "Found param waypointTour='%s'" % waypointTour)

            waypointTour = waypointTour.strip()

            if waypointTour == "":
                rospy.loginfo(  "Error: No waypointTour file specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo(  "waypointTour file: '%s'" % waypointFile)


        if sts ==0:
            # run the navigator node
            nav = Navigator(mapPath,waypointFile,waypointTour)
            sts = nav.Run()

    except Exception as ex:
        rospy.loginfo(  "Navigator Crashed with exception: %s" % str(ex))
        sts = -1

    finally:
        rospy.loginfo(  "Navigator Finished")
        
    sys.exit(sts)
