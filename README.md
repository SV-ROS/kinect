kinect
#

Kinect Challenge 2014 

This repo contains the following ROS packages
Package kinect               - Provides launchers for gazebo Simulation
Package kinect_challenge     - Provides Nodes for interaction with the Microsoft Benchmark App, and nodes and launchers to run the mapping and navigation phases


kinect:  Simulation and navigation in gazebo
##
Provides launchers for Simulation and Navigation in Gazebo.

Simulation
###

Start the simulation

    roslaunch kinect turtlebot.launch

Then, to manually drive the robot around you can use

    roslaunch kobuki_keyop keyop.launch

Then, to see what the robot sees (camera image, pointcloud, etc.) you can start

    roslaunch turtlebot_rviz_launchers view_robot.launch
    
Navigation demo
###

To try navigation in the room model run `roslaunch kinect amcl_demo.launch`. It starts gazebo and all the other pieces.


kinect_challenge:  Competition Environment
##

This package provides scripts and launch files for the competition environment for both the mapping and navigation phases
It provides three Nodes:
MSBMIOSvs    - A service for communicating with the Microsoft Benchmark Computer, a simulation mode is available so you 
              don't need to run the Microsoft app to test the mapping and navigation. (see configuration below)

Mapper.py    - A Node that runs the mapping phase
Navigator.py - A Node that runs the navigation phase

Basic launch instructions.
###

Mapping:
####

If running the Microsoft app start it on the windows computer first, else configure it for sim mode, (see below)

To run mapping run `roslaunch kinect_challenge kc_map.launch`.  It should launch everything you need for mapping (see configuration below)
Drive around to create the map, then visit each waypoint and press 'A' on the joy stick to record the location of the waypoint.
At the end press 'X' to save the map and waypoint locations.

After Mapping configure the maps/tour.wpt file and run Navigation (see below)

Note If using the actual MS App it expects there to be 8 waypoints recorded during mapping and used during navigation.

Navigation:
####

If running the Microsoft app start it on the windows computer first, else configure it for sim mode, (see below)

To run navigation run `roslaunch kinect_challenge kc_nav.launch`.  It should launch everything you need for navigation (see configuration below)
The robot should be placed close to the first waypoint at the start, when the launch starts the robot should drive from waypoint to waypoint in the order 
defined in the .../maps/tour.wpt file (see configuration below)

Note: orientation matters, the robot will try to match the direction it was facing when the waypoint was recorded.

The Navigation Node generates the RunID sent to the microsoft app using an integer based on the current time when the node is launched.

Configuration:
###
The launch files are in kinect_challenge\launch and can be modified to fit your robot.

Modify the kc_map.launch and kc_nav.launch to suit your robot (see comments in the files to add your base launch entries)
You may need to modify the teleop launch if you are not using a Microsoft XBox 360 controller

For mapping the Joystick buttons used can be configured with the lmButtonIdx and msButtonIdx parameters passed to the Mapper.py node

Microsoft Benchmark Communications
###
The MSBMIOSvs node uses a couple of parameters to configure to IP address and port
host_IP   - the ip address of the windows PC running the Microsoft app. (must be in XXX.XXX.XXX.XXX format, host name not supported)
port      - the port to talk to the MS app on, default = 7576

When the MS App is running it expects to see the Fiducial near the center of its camera view when the robot is at a waypoint.
It will time out with an error if it cant find the image in a short time.

Simulating Microsoft App
if you enter "sim" for the host_IP instead of a valid IP address the node will simulate the Microsoft app and just alway return an "OK" status

Data Files
### 
The package uses a waypoints.wp and a tour.wpt data file, they are saved in the maps folder.

waypoint.wp:
This file is created during mapping and records the index and location of each waypoint in map coordinates.
The navigator reads this file to determine the location of the waypoints.

tour.wpt:
You must create this file, it is a list of 'from' and 'to' indexes for the waypoints to be navigated during navigation.
Each line has two numbers, the index of the starting waypoint and he index of the destination waypoint.
The last line should go from the last waypoint visited back to the first way point to make a circular tour.
Each line should start from the waypointIdx that as the destination index on the previous line.

0 <toIdx>
<toIdx> <nextToIdx>
...
<lastToIdx> 0

The robot should start at index 0

See the Microsoft Rules for details of what makes a valid tour.

To Do:
###

Handle failure to find a way point.
Allow restarting from an intermediate location
Allow the runID to be manually set before launching Navigation

