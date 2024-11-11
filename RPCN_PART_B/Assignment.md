# Lidar-inertial sensor fusion in 2D
v.v.lehtola@utwente.nl

Use the supplied docker file to build a ready virtual environment with all the needed libraries and packages. See README file. Documentation about scan matching 2D lidar data and using supplementing IMU data can be found at [cartographer wiki](http://wiki.ros.org/cartographer) 
and [ros wiki](http://wiki.ros.org/laser_scan_matcher) 

## Report

The report has a **page limit of 3 pages** and it must contain:

Figure 1: Take a snapshot of your best cartographer result in rviz, when the whole trajectory is visible. Do this after tuning the parameters.

Discuss briefly in your report:
-	What are the relations between keyframes (lidar lecture 1), poses seen in rviz, and submaps (cartographer documentation)?
-	What are the conditions for creating submaps in cartographer?
-	How are the submaps used?
-	When returning from the farthest point, is the number of submaps created higher, lower, or the same than during going there? Why/ Why not?
-	Think and play with the parameters: What would be the smallest number of submaps for running the part B exercise successfully? Hint: there needs to be enough overlap between them for the SLAM algorithm.
-	How is the loop closing done?

Figure 2(*): Plot the following trajectories in one figure:
-	IMU integration trajectory (group work part A2)
-	2D lidar trajectory after parameter tuning (=content of Figure 1)
-	Lidar-imu fusion trajectory (see the my_robot.lua file options to enable IMU data usage)

Discuss the results briefly. Why is the drift different? Were there any fast rotations? If yes, how does it show? If not, what would have happened if there were?

## CBL

This section concerns only MROB students with ASAI or HSAI specialization.

The benefit of lidar(s) depends on the environment, and the task at hand. Consider their applicability (and that of the cameras!) to your CBL system. Instead of (*), i.e. Figure 2 and the related discussion, include the following into your CBL report (=not to `exercise#B` report):

*  How can one or multiple lidars/cameras benefit your CBL robot design, and how should they be placed?
*  What should be the quality of these sensors in terms of error and noise? Will these be low-cost? Briefly consider also the energy usage.
*  Briefly describe the methodology which would be used to calibrate these sensors.
