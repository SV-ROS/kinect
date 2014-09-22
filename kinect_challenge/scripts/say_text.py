#!/usr/bin/env python

#######################
#
#   Kinect Challenge text to speech Node
#
#   simple python API to the ROS text to speech service.
#
#   sends SoundRequest messages on the "robotsound" topic
#
#  Author: Ralph Gnauck
#  License: BSD
#
#
#   Created for the SV ROS entry in the 2014 IROS Microsoft Kinect Challenge Competition
#
#######################
import rospy
import time

from sound_play.msg import SoundRequest

class SayText:

  # Constructor establishes publisher to messages
  def __init__(self):
	self.sound_pub = rospy.Publisher("robotsound", SoundRequest)

  # play a text to speech message once
  def say(self, text_msg):
	msg = SoundRequest()
	msg.sound = SoundRequest.SAY
	msg.command = SoundRequest.PLAY_START
	msg.arg = text_msg

	self.sound_pub.publish(msg)
	time.sleep(2)
