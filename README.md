# kinect


Kinect Challenge 2014 

This repo contains the following ROS packages
 - Package kinect               - Provides launchers for gazebo Simulation
 - Package kinect_challenge     - Provides Nodes for interaction with the Microsoft Benchmark App, and nodes and launchers to run the mapping, waypoint creation and navigation phases


## kinect:  Simulation and navigation in gazebo

Provides launchers for Simulation and Navigation in Gazebo.

### Simulation


Start the simulation

    roslaunch kinect turtlebot.launch

Then, to manually drive the robot around you can use

    roslaunch kobuki_keyop keyop.launch

Then, to see what the robot sees (camera image, pointcloud, etc.) you can start

    roslaunch turtlebot_rviz_launchers view_robot.launch
    
### Navigation demo


To try navigation in the room model run `roslaunch kinect amcl_demo.launch`. It starts gazebo and all the other pieces.


## kinect_challenge:  Competition Environment


This package provides scripts and launch files for the competition environment for the mapping, waypoint creation and navigation phases
It provides these Nodes:
 - msbmio_svs.py    - A service for communicating with the Microsoft Benchmark Computer, a simulation mode is available so you 
              don't need to run the Microsoft app to test the mapping and navigation. (see configuration below).
 - mapper.py    - A Node that runs the mapping and waypoint creation phases.
 - navigator.py - A Node that runs the navigation phase.
 - say_text.py   - A node to interface with ROS sound_paly for voice synth.

It supports the real Pinoner P3DX robot.

It also needs:
   - rtabmap installed
   - ROSARIA installed
   - and some other ros nodes, (sound_play)

    sorry dependencies are not fully up to date.

### Maps.

The 2D maps created during the Challenge are now in the kinect_challenge/ChallengeMap folder:
there are three copies of the map:
- map_challenge.pgm      (the origional map created on the robot during mapping phase)
- map_remapped.pgm       (the reworked map after manualy adjusting loop closures in rtabmap and regenerating the 2D and 3D maps)
- map_final.pgm          (the final map we used for navigation phase, this has some manual edits to remove noise and close the open external holes in the map)

map_final.yaml is the associated yaml file for the final version of the map.
the waypoint.wp and tour.wpt fiels are also in the folder.

Rename any of the three maps to just map.pgm and map.yaml (update the internal name in map.yaml) to use any of these maps.

The 3D rtabmap.db files are two big (~217MB) to put into github, we will try to find an alternative repository for them soon.


### Basic launch instructions.
Running the challenge scenarios is done in three phases, mapping, waypoint creation and navigation.

#### Mapping:

To run the mapping phase launch

 `roslaunch kinect_challenge kc_p3dx_map.launch`

It should launch everything you need for mapping (see configuration below)

Drive around using teloep to create the map (you will not be creating the waypoints at this time) .

At the end press 'X' to save the 2D map.

Use ^C to end the launch file when done.

The final rtabmap.db of the 3D map is not fully saved until the launch file has fully terminated.


#### Waypoint creation:
If running the Microsoft app start it on the windows computer first, else configure it for sim mode, (see below)

To run the waypoint creation phase launch

 `roslaunch kinect_challenge kc_p3dx_mwp.launch`
 
It should launch everything you need for waypoint creation (see configuration below)
 
Start driving using teleop to get the robot localized.

Press 'A' on the joy stick to record the location of each waypoint.

Use ^C to end the launch file when done.

After all waypoints are saved configure the maps/tour.wpt file used in the Navigation Phase (see below).

Note If using the actual MS App it expects there to be 8 waypoints recorded during mapping and used during navigation.

#### Navigation:

If running the Microsoft app start it on the windows computer first, else configure it for sim mode, (see below)

To run navigation run

 `roslaunch kinect_challenge kc_p3dx_nav.launch`

It should launch everything you need for navigation (see configuration below)

Start driving using teleop to get the robot localized, (you have about 15 seconds before the autonomous navigation begins).

The robot should be placed close to the first waypoint at the start, after the launch starts the robot should drive from waypoint to waypoint in the order 
defined in the .../maps/tour.wpt file (see configuration below)

Note: orientation matters, the robot will try to match the direction it was facing when the waypoint was recorded.

The Navigation Node generates the RunID sent to the microsoft app using an integer based on the current time when the node is launched.

During navigation if the robot gets stuck and can't make progress towards the current goal, it will chose another goal, try to drive towards that for about three seconds
and then try driving to the original goal again.

NOTE: In waypoint creation and Navigation Phases you can display the 3D point cloud in rviz, after the robot is localized click the 'Download Map' check-box in the RTABMAP
plug-in and it should be display the point cloud.

### Configuration:

The launch files are in kinect_challenge\launch and can be modified to fit your robot.

Modify the kc_p3dx_map.launch, kc_p3dx_mwp.launch and kc_p3dx_nav.launch to suit your robot (see comments in the files to add your base launch entries)
You may need to modify the teleop launch if you are not using a Microsoft XBox 360 controller

For mapping the Joystick buttons used can be configured with the lmButtonIdx and msButtonIdx parameters passed to the Mapper.py node

### Microsoft Benchmark Communications

The MSBMIOSvs node uses a couple of parameters to configure to IP address and port
 - host_IP   - the ip address of the windows PC running the Microsoft app. (must be in XXX.XXX.XXX.XXX format, host name not supported)
 - port      - the port to talk to the MS app on, default = 7576

When the MS App is running it expects to see the Fiducial near the center of its camera view when the robot is at a waypoint.
It will time out with an error if it cant find the image in a short time.

Simulating Microsoft App

If you enter "sim" for the host_IP instead of a valid IP address the node will simulate the Microsoft app and just alway return an "OK" status

### Data Files
 
The package uses a waypoints.wp and a tour.wpt data file, they are saved in the maps folder.

 - waypoint.wp:
This file is created during mapping and records the index and location of each waypoint in map coordinates.
The navigator reads this file to determine the location of the waypoints.

 - tour.wpt:
You must create this file, it is a list of 'from' and 'to' indexes for the waypoints to be navigated during navigation.
Each line has two numbers, the index of the starting waypoint and the index of the destination waypoint.
The last line should go from the last waypoint visited back to the first way point to make a circular tour.
Each line should start from the waypointIdx that as the destination index on the previous line.

```
0 <toIdx>
<toIdx> <nextToIdx>
...
<lastToIdx> 0
```

The robot should start at index 0

See the Microsoft Rules for details of what makes a valid tour.

### To Do:
 - Handle failure to find a way point.
 - Allow restarting from an intermediate location
 - Allow the run_id to be manually set before launching Navigation

