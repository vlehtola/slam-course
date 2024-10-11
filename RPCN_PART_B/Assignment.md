# Lidar-inertial sensor fusion in 2D
v.v.lehtola@utwente.nl

Use the supplied docker file to build a ready virtual environment with all the needed libraries and packages. See README file. Documentation about scan matching 2D lidar data and using supplementing IMU data can be found at [cartographer wiki](http://wiki.ros.org/cartographer) 
and [ros wiki](http://wiki.ros.org/laser_scan_matcher) .

## Report

Figure 1: Take a snapshot of the cartographer result in rviz, when the whole trajectory is visible. 	

Discuss in your report:
-	What are the relations between keyframes (lidar lecture 1), poses seen in rviz, and submaps (cartographer documentation)?
-	What are the conditions for creating submaps in cartographer?
-	How are the submaps used?
-	Compared to the 1st loop, is the number of submaps created higher, lower, or the same during the 2nd and the 3rd loop? Why/ Why not?
-	Think and play with the parameters: What would be the smallest number of submaps for running the part B exercise successfully? Hint: there needs to be enough overlap between them for the SLAM algorithm.
-	How is the loop closing done?

Figure 2: Plot the following trajectories in one figure and add to your report:
-	IMU integration trajectory (group work part a)
-	2D lidar trajectory
-	Lidar-imu fusion trajectory (see the my_robot.lua file options to enable IMU data usage)

Discuss the results briefly.
