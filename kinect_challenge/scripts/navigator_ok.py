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
#   Created for the SV ROS entry in the 2014 IROS Microsoft Kinect Challenge Competition
#
#######################

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

import msbmio_client

from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty

import say_text

####################################################
#
#   Class for Navigator Ros Node
#

class Navigator():
    #
    # Constructor
    #
    #   map_path:  path for the waypoint and tour files
    #   waypoint_file: name of waypoint file
    #   waypoint_tour: name of tour file
    #
    #   publishes arrow marks for waypoint visualization in rviz
    #
    #   uses action lib to direct move_base to handle the navigation from waypoint to waypoint
    #
    def __init__(self,map_path,waypoint_file,waypoint_tour):

        # Save params
        self.map_path = map_path
        waypoint_file = os.path.join(map_path, waypoint_file)
        waypoint_tour = os.path.join(map_path, waypoint_tour)

        self.waypoint_file =  waypoint_file
        self.waypoint_tour =  waypoint_tour

        # setup waypoint marker publisher
        self.marker_pub = rospy.Publisher("visualization_marker", Marker)

        # create action client
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait for move_base server to be ready
        while not self.ac.wait_for_server(rospy.Duration(5.0)) and not rospy.is_shutdown():
            rospy.loginfo( "Waiting for Move Base Server")


        # MSBM expects a runID during navigation, use current time in seconds past epoch for each run
        self.runID = int(time.time() * 1000)

        self.rate = rospy.Rate(10.0)

        self.tour_index = 0 # start at first leg of the tour

        # read the waypoints and tour files
        self.waypoints = self.load_waypoints(self.waypoint_file)
        self.waypoint_tour = self.load_waypoint_tour(self.waypoint_tour)

        
        self.vocaliser = say_text.SayText()

    # say stuff
    def say(self,msg):
        self.vocaliser.say(msg)

    # send goal to move_base
    # idx is line number in tour file, goal set to waypoint of 'to' location in tour file at line idx.
    # assumes we are currently at waypoint of 'from' location in tour file at line idx
    def set_goal(self, index):
        
        self.goal = MoveBaseGoal()

        # get waypoint pose
        target_waypoint_number = self.waypoint_tour[index][1]
        tf = self.waypoints[target_waypoint_number]

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
        return target_waypoint_number

    # read the tour file
    # 
    # each line has two comma separated integers,  fromWP, toWP
    #
    #  fills in tour list with tuples of (to,from) pairs and returns it
    #
    def load_waypoint_tour(self, waypoint_tour):
        tour = []

        with open(waypoint_tour,'r') as wptfh:
            wptxt = wptfh.readlines()

        for t in wptxt:
            if t.strip() != "":
                wpf,wpt = t.split(",")
                tour.append((int(wpf),int(wpt)))

        rospy.loginfo("Loaded Tour Info:\n%s" % tour)
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
    def load_waypoints(self, waypoint_file):

        waypoints = []

        with open(waypoint_file,'r') as wpfh:
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
    def run(self):

        state =0 # initial state of state machine

        # get starting node from first tour leg
        target_waypoint_number =self.waypoint_tour[self.tour_index][0]
        
        goal_time = 300/5  # allow 300 seconds (in 5 second chunks to complete the leg)
        time.sleep(15)    # allow time to localize and drive to start node !!! TODO: make this triggered by Joystick button

        # thunderbirds are 'GO'
        self.say("Navigator Ready")

        while not rospy.is_shutdown():
            
            # publish the waypoint markers
            self.create_markers()

            if state == 0: # Send start node and start a leg 
            
                rospy.loginfo("Sending start %d" % target_waypoint_number)

                # send message to MSBM indicating we are starting a leg from 'to' node (starts the clock)
                sts=msbmio_client.MSBMIOClient("start",str(target_waypoint_number),"%d" % self.runID) # always returns OK
                
                self.say("Starting from waypoint %d"%target_waypoint_number)
                #time.sleep(5)
                if sts =="OK": # should always be ok on start leg

                    
                    rospy.loginfo("Sending goal %d" % self.waypoint_tour[self.tour_index][1])

                    # tell move base the end waypoint and set it as current goal
                    target_waypoint_number=self.set_goal(self.tour_index)
                    
                    self.say("Setting goal waypoint %d"%target_waypoint_number)

                    # setup leg timer
                    goal_time = 300/5
                    state = 1  # next state


            if state == 1: # monitor goal progress
            
                rospy.loginfo("waiting to reach goal" )
                goal_status = self.ac.wait_for_result(rospy.Duration(5)) # wait for 5 seconds to see if we reach the goal yet
                if goal_status == True:  
                    state = 10 # we got there or aborted
                else:
                    goal_time = goal_time - 1 # 5 seconds is up, count down main loop timer
                    if goal_time < 0: # If total time out finished we failed to get there, goto error state
                        state = 20


            if state == 10:  # we finished the goal
            
                goal_state = self.ac.get_state() # get actual completion status
                print str(goal_state)
                
                if goal_state == GoalStatus.SUCCEEDED:  # did we really get there
                    
                    rospy.loginfo("Reached goal, sending notify" )

                    # notify MSBM we are at target waypoint
                    sts = msbmio_client.MSBMIOClient("end",str(target_waypoint_number),"%d" % self.runID)
                    
                    if sts =="OK":  # MSBM accepted waypoint
                        rospy.loginfo("Goal achieved, start next segment")
                        self.say("Reached goal waypoint %d"%target_waypoint_number)
                        self.set_next_waypoint() # get next leg
                        state = 0 # go back to send next start notification
                        

                    else: # MSBM did not like us, try again
                        rospy.loginfo( "Failed to record arrival, retry")
                        self.say("Failed to notify benchmark that goal waypoint %d reached. Will retry"%target_waypoint_number)
                       
                        state = 0

                else: # move base failed to get us there
                    rospy.loginfo( "Failed to reach goal %d. Status '%d'" % (target_waypoint_number,goal_state))
                    self.say("Failed to reach goal waypoint %d. Status is %d" %(target_waypoint_number,goal_state))
                    state = 100 # skip to next wp

                    
            if state == 20: # did not reach the goal in time, give up and move to next WP
                self.ac.cancel_goal()
                rospy.loginfo( "Failed to reach goal %d in designated time." % target_waypoint_number)
                self.say("Failed to reached goal waypoint %d in designated time"%target_waypoint_number)
                state = 100

                
            if state == 100: # skip to next WP
                rospy.loginfo( "Trying next goal")
                self.say( "Trying next goal")
                self.set_next_waypoint()# skip WP and try next one
                state = 0
                

            self.rate.sleep()

        return 0

    # get next leg of the tour
    # validates that the to and from waypoints are valid waypoints in the waypoints list
    def set_next_waypoint(self):

        # get next leg, loop to start once we go past the end
        self.tour_index = self.tour_index + 1
        if self.tour_index >= len(self.waypoint_tour):
            self.tour_index = 0

        # get num waypoints
        num_waypoints = len(self.waypoints)

        # get leg and to and from  wp
        tour_sequence = self.waypoint_tour[self.tour_index]
        from_waypoint = tour_sequence[0]
        to_waypoint = tour_sequence[1]

        # check from wp is valid
        if from_waypoint >= num_waypoints :
            msg = "Invalid from waypoint %d in tour step %d, resetting tour to beginning" % (from_waypoint,self.tour_index)
            rospy.loginfo(msg)
            self.say(msg)
            self.tour_index = 0

        # check to wp is valid
        if to_waypoint >= num_waypoints :
            msg = "Invalid to waypoint %d in tour step %d, resetting tour to beginning" % (to_waypoint,self.tour_index)
            rospy.loginfo(msg)
            self.say(msg)
            self.tour_index = 0

    # create the visualization markers for the waypoints and publish them
    def create_markers(self):
        waypoint_index = 0
        for tf in self.waypoints:
            #print "Creating marker @ %s %d" % (str(tf),waypoint_index)
            marker=self.make_arrow_marker(tf, waypoint_index)
            self.marker_pub.publish(marker)
            marker=self.make_text_marker(tf, waypoint_index)
            self.marker_pub.publish(marker)
            waypoint_index = waypoint_index + 1

    # create a visualization marker arrow at the tf location (in /map frame) and with id num
    def make_arrow_marker(self, tf, num):
        return self._makeMarker(tf, num)
                
    # create a text marker at offset above(0.575m)  tf(in /map frame)  with id num+100 and text = str(num)
    def make_text_marker(self, tf, num):

        marker = self._makeMarker(tf, num) # create an arrow first

        marker.type = marker.type= marker.TEXT_VIEW_FACING # now change it to text
        marker.id = num + 100 # update the ID
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
    def _makeMarker(self, tf, num):

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


def usage():

    rospy.init_node("Invalid arguments, Usage:")
    return "%s" %sys.argv[0]

# main function
#
# reads the params and creates the Navigator node
if __name__ == "__main__":

    sts = 0

    rospy.loginfo("Running Navigator")
    try:

        rospy.init_node('navigator')

        map_path = ""
        waypoint_file = ""
        waypoint_tour = ""


        if sts ==0:

            if map_path == "":
                map_path = rospy.get_param("~map_path","")
            rospy.loginfo("Found param map_path='%s'" % map_path)

            map_path = map_path.strip()

            if map_path == "":
                rospy.loginfo("Error: No map_path specified")
                print usage()
                sts= 1
            else:
                rospy.loginfo("Waypoints will be read from: '%s'" % map_path)

        if sts ==0:

            if waypoint_file == "":
                waypoint_file = rospy.get_param("~waypoint_file","")
            rospy.loginfo("Found param Waypointfile='%s'" % waypoint_file)

            waypoint_file = waypoint_file.strip()

            if waypoint_file == "":
                rospy.loginfo("Error: No waypoints file specified")
                print usage()
                sts = 1
            else:
                rospy.loginfo("Waypoints file: '%s'" % waypoint_file)

        if sts == 0:

            if waypoint_tour == "":
                waypoint_tour = rospy.get_param("~waypoint_tour","")
                
            rospy.loginfo("Found param waypoint_tour='%s'" % waypoint_tour)

            waypoint_tour = waypoint_tour.strip()

            if waypoint_tour == "":
                rospy.loginfo("Error: No waypoint_tour file specified")
                print usage()
                sts = 1
            else:
                rospy.loginfo("Waypoint_tour file: '%s'" % waypoint_file)


        if sts == 0:
            # run the navigator node
            nav = Navigator(map_path, waypoint_file, waypoint_tour)
            sts = nav.run()

    except Exception as ex:
        rospy.loginfo("Navigator Crashed with exception: %s" % str(ex))
        sts = -1

    finally:
        rospy.loginfo("Navigator Finished")
        
    sys.exit(sts)











