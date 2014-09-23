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

import msbmio_client
import say_text
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
    #   map_path: path where the map and waypoint file will be saved
    #
    #   waypoint_file: name of the waypoint file default = 'waypoint.wp'
    #
    #   lm_button_index:   joystick button index for button to trigger landmark creation
    #   ms_button_index:   joystick button index for button to trigger saving of the map and rtabmap.db
    #
    
    def __init__(self, map_path, waypoint_file, lm_button_index, ms_button_index):
    
        self.map_path = map_path
        waypoint_file=os.path.join(map_path, waypoint_file)

        # backup old waypoint file if one already exists (adds timestamp to file name)
        if os.path.exists(waypoint_file):
            backup_file =  waypoint_file + ".%s.bak" % time.ctime()
            backup_file = backup_file.replace(":","_")
            print "Backing up old waypoint file '%s' to '%s'" % ( waypoint_file,backup_file)
            shutil.move(waypoint_file, backup_file)

        # save parmas
        self.waypoint_file =  waypoint_file
        self.lm_button_index = lm_button_index
        self.ms_button_index = ms_button_index

        # create publisher for visualization markers for waypoints
        self.marker_pub = rospy.Publisher("visualization_marker", Marker)

        # we listen to the joystick buttons
        rospy.Subscriber("joy", Joy, self.joy_callback)

        # need the transform from  /map to /base_footprint
        self.listener = tf.TransformListener()
        
        self.rate = rospy.Rate(10.0)
        
        self.tf = None  # not seen a transform yet
        self.waypoint_num = 0 # start from waypoint 0
        
        self.vocalizer = say_text.SayText()  # set up text to speech for user feedback
        
        # give tf a chance to queue up some transforms
        rospy.sleep(3)
        
    # speak a message
    def say(self, msg):
        self.vocalizer.say(msg)

    # main run loop
    def run(self):
        
        self.say("Mapper Ready")

        while not rospy.is_shutdown():
            transform_ok = False
            try:
                # check for transform
                #self.listener.waitForTransform('map', 'base_footprint', rospy.Time(), rospy.Duration(0.5))
                (trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))

                # clean up pose, force to be on and parallel to map plane
                trans = (trans[0], trans[1], 0) # z=0

                # roll and pitch = 0
                (r, p, y) = euler_from_quaternion(rot)
                qt = quaternion_from_euler(0, 0, y)
                rot=(qt[0], qt[1], qt[2], qt[3])
                
                transform_ok = True
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass # ignore these exceptions
                
            if transform_ok: # got good transform so save it in case we save a waypoint
                self.tf = (trans, rot)
            
            self.rate.sleep()
        
        return 0

    # create a visualization marker arrow at the tf location (in /map frame) and with id num
    def make_arrow_marker(self, tf, num):
        return self._make_marker(tf,num)

    # create a text marker at offset above(0.575m)  tf(in /map frame)  with id num+100 and text = str(num)
    def make_text_marker(self,tf,num):
        
        marker = self._make_marker(tf,num) # create an arrow first
        
        marker.type = marker.type= marker.TEXT_VIEW_FACING # now change it to text
        marker.id = num + 100 # update the ID
        marker.pose.position.z = marker.pose.position.z + 0.075 # move it above the arrow
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.text=str(num)  # set the text string
        return marker

    # create a visualization marker arrow  at offset above(0.5m) tf (in /map frame) location and with id num
    def _make_marker(self,tf,num): 
    
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
    def joy_callback(self,msg):

        # is lm_button_index a  valid buttonID
        if self.lm_button_index >= 0 and self.lm_button_index < len(msg.buttons):
            keyState = msg.buttons[self.lm_button_index] # ok so return button state
        else:
            keyState = -1 # ignore this button

        if keyState == 1:  # Save landmark button pressed
            self.say("Save Waypoint button pressed") 
            rospy.loginfo("Saving landmark %d" % self.waypoint_num)
            #print "Got Joy Message: %s" % str(msg)
            
            if self.tf != None: # do we have a valid transform to use
                # send message to MSBM PC 
                sts = msbmio_client.MSBMIOClient("map", str(self.waypoint_num), "0")
                rospy.loginfo("MSBMIO returned %s" % sts)
                
                if sts == "OK": # Landmark accepted by MSBM so save in the waypoint file and create a visualization marker
                    self.marker_pub.publish(self.make_arrow_marker(self.tf, self.waypoint_num))
                    self.marker_pub.publish(self.make_text_marker(self.tf, self.waypoint_num))
                    
                    rospy.loginfo("Saving waypoint %d: %s, %s" %(self.waypoint_num,str(self.tf[0]), str(self.tf[1])))

                    # save waypoint pose in file
                    with open(self.waypoint_file,'a') as wpfh:
                        wpfh.write("%d: %s\n" % (self.waypoint_num, str(self.tf)))

                    self.say("Waypoint %d saved ok" % self.waypoint_num)

                    # increment waypoint number for next waypoint
                    self.waypoint_num = self.waypoint_num + 1
                
                else:
                    rospy.loginfo("MS Benchmark Failed to record waypoint, please try again")
                    self.say("Waypoint Failed" )
            else:
                rospy.logwarn("Can't save landmark, No Transform Yet")
                self.say("Waypoint Failed" ) 

        # is save map button idx ok
        if self.ms_button_index >= 0 and self.ms_button_index < len(msg.buttons):
            keyState = msg.buttons[self.ms_button_index] # get state of save map button
        else:
            keyState = -1

        if keyState==1: # save map button pressed
            self.save_map()
            

    def save_map(self):
        self.say("Saving map")

        # Put rtabmap in localization mode so it does not continue to update map after we save it
        rtabmap_localization_mode = rospy.ServiceProxy('rtabmap/set_mode_localization',Empty())
        rtabmap_localization_mode()
        
        # create timestamp for map file
        time_stamp = "%s" % time.ctime().replace(":","_").replace(" ","_")
        map_file = "map_%s" % time_stamp
        db_file = "kc_map_%s.db" % time_stamp

        # save map file using map_server
        rospy.loginfo( "Saving map to file '%s'" % map_file)
        sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (self.map_path, map_file), shell=True)

        # Save a default copy just named "map" also
        if sts == 0:
            sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (self.map_path, "map"), shell=True)

        # make backup of the rtabmap DB
        dbPath = os.environ["HOME"]
        if sts == 0:
            sts = subprocess.call('cp "%s/.ros/rtabmap.db" "%s/.ros/%s"' % (dbPath, dbPath, db_file), shell=True)

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
    
    rospy.loginfo("Running Mapper")
    try:
    
        rospy.init_node('mapper')
        
        map_path = ""
        waypoint_file = ""
        
        lm_button_index = -1
        ms_button_index = -1

        if sts == 0:
            if map_path == "":
                map_path = rospy.get_param("~map_path","")
                rospy.loginfo("Found param map_path='%s'" % map_path)
                
            map_path = map_path.strip()
            
            if map_path == "":
                rospy.loginfo("Error: No map_path specified")
                print usage()
                sts = 1
            else:
                rospy.loginfo("Map and waypoints will be saved in: '%s'" % map_path)
                 
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
                rospy.loginfo("Waypoints will be saved to: '%s'" % waypoint_file)
 
        if sts == 0:
            if lm_button_index == -1:
                lm_button_index = int(rospy.get_param("~lm_button_index", "-1"))
                
           
            if lm_button_index == -1:
                rospy.loginfo("No lm_button_index specified")
                print usage()
                sts = 1
            else:
                rospy.loginfo("Using lm_button_index %d" % lm_button_index)

        if sts == 0:
            if ms_button_index == -1:
                ms_button_index = int(rospy.get_param("~ms_button_index","-1"))
                
           
            if ms_button_index == -1:
                rospy.loginfo("No ms_button_index specified")
                print usage()
                sts = 1
            else:
                rospy.loginfo("Using ms_button_index %d" % ms_button_index)
                
        #if sts ==0:
        # all ok so start the node
        mapper = Mapper(map_path, waypoint_file, lm_button_index, ms_button_index)
        sts = mapper.run()

    except Exception as ex:
        rospy.loginfo("Mapper Crashed with exception: %s" % str(ex))
        sts = -1
        
    finally:
        rospy.loginfo("Mapper Finished")
        sys.exit(sts)











