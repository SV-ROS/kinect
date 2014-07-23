kinect
======

Kinect Challenge 2014  

Simulation
----------

Start the simulation

    roslaunch kinect turtlebot.launch

Then, to manually drive the robot around you can use

    roslaunch kobuki_keyop keyop.launch

Then, to see what the robot sees (camera image, pointcloud, etc.) you can start

    roslaunch turtlebot_rviz_launchers view_robot.launch
    
Navigation demo
---------------

To try navigation in the room model run `roslaunch kinect amcl_demo.launch`. It starts gazebo and all the other pieces.
