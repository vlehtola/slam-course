
Use this assignment to refine your analytical and innovative thinking by exploring A-LOAM with NTU VIRAL Dataset and suggesting improvements

# Multi-Sensor Data Fusion using A-LOAM

In this assignment, students will:

 - Map and analyze key components from the A-LOAM paper to the A-LOAM codebase provided.
 - Perform a feasibility study and evaluate the impact of low-cost sensing to the map quality and algorithm robustness.
 - Get hands-on experience with multi-line 3D lidar data and working with an open SLAM dataset (NTU VIRAL dataset).

## Task 1: Mapping Key Components of the Paper to the Codebase
**Objective:** Identify and map specific components of algorithms discussed in the LOAM paper to corresponding functions or files in the provided A-LOAM codebase. A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time). A-LOAM is intended as good learning material for SLAM beginners.

Instructions: Find how and where following tasks are achieved
- Scan Registration: How the lidar scan points are matched.
- Odometry Estimation: Estimating the relative motion between consecutive scans.
- Mapping: How the map is incrementally built and refined.
- Sensor Fusion: Merging different sensor data together
- Error metrics: Is there any optimization happening to improve the results? What are the error metrics 

Document your findings briefly by comparing the logic 
written in the paper with quotes of relevant code snippets, 
including filenames and line numbers.

## Task 2: Feasibility study on low-cost sensing 
**Objective:** Evaluate the impact of reduced lidar sensing quality on SLAM performance.

Many robotic systems are preferably low cost. Here, the task is to test that how much the lidar ranging ability can be
reduced before it significantly affects the A-LOAM functionality for the given application (=drone localization and mapping).
Hence, you will determine the feasibility limit for the lidar scanners in terms of scan range.
This means that lower cost lidars may be used to do the job.

Instructions: 
 - Modify the maximum range of lidar data in A-LOAM. To do this, you need to adjust how the lidar point cloud is processed in the code.
 - Following the logic how `double MINIMUM_RANGE = 0.1;` is implemented, implement a maximum range **R**.

 - In file `scanRegistration.cpp`, look for the line to be modified:

         if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;

- Compile the modified code using the docker package.
- Run A-LOAM using the NTU-VIRAL data, and your modified code.
- Find two relatively close values for **R** between 0.1 m 
and 100 m, one for which the method is still working, and another one with which significant problems arise. Report these values.
- Based on your analysis in Task 1, discuss which part(s) of the method is first to break when
**R** is reduced.

Document your findings.

## Report

The **page limit for the report is 3 pages**, and it must contain the following:
 1. Five small sections matching the points listed in Task 1, use e.g. quotes from the paper and quotes from the code 
 2. Two values for the maximum range of the lidar **R**, with two snapshots from rviz showing the method working and not working. (Hint: visualize and annotate scan match errors in registered point clouds)
 3. Discuss briefly which part(s) of the algorithm fail first when **R** is low (*).

## CBL

This section concerns only MROB students with ASAI or HSAI specialization.

Instead of (*), include the following into your CBL report:

- Select and name a sensor fusion strategy (complementary, competitive, and cooperative) and provide valid arguments how this strategy benefits the design of your CBL system.
- See the CBL rubric to fulfil the remaining tasks.

