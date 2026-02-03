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

## PPD - Assignment B: Environmental Perception

Note: The following questions guide your thinking. If a technology (LiDAR, cameras, etc.) 
is not applicable to your robot design, state this explicitly and justify why alternative 
approaches are sufficient. You will be graded on engineering reasoning, not on using 
specific sensors. Both using and not using a particular sensor can be correct choices 
depending on your system requirements.

Instead of the groupwork exercise (*), include the following in your CBL report 
(=not in `exercise#B` report):

* Identify your robot's environmental perception requirements (obstacle detection, 
  semantic understanding, operating conditions: lighting, weather, occlusions)
  
* Justify your chosen perception approach

* **If using LiDAR:** Describe type (2D/3D, mechanical/solid-state), placement, 
  quality requirements (range, accuracy, angular resolution), and calibration methodology

* **If using camera(s):** Describe type (mono/stereo, RGB/depth, thermal), placement, 
  quality requirements (resolution, frame rate, lighting sensitivity), and calibration 
  methodology (intrinsic/extrinsic)

* **If not using LiDAR or cameras:** Explain your chosen sensors (ultrasonic, IR 
  proximity, tactile, structured light, etc.), how these meet your requirements, and 
  what tradeoffs you considered

These PPD report points may be written after the cognition challenge and the design of the system has been agreed upon within the group.  
The base idea should be there at the 2/3 mandatory Q&A session.
