# Pedestrian dead reckoning 

Calculate a trajectory by integrating the IMU data over time. Perform the IMU integration in 2D and/or 3D. See README how to access the data through docker.

## Report

The report has a page limit of 3 pages and it must contain:

Figure#1: Plot the 2D X-Y trajectory with the IMU calibration from task A1. Integrate the trajectory from data that was extracted from `rosbag`. 

Figure#2: Plot the 2D X-Y trajectory without IMU calibration, for comparison purposes. 

Discuss: Is the result better with calibration? 
According to the calibrated data, did the IMU arrive in the same place from where it started? Estimate the magnitude of drift. In other words, measure the Cartesian distance between the start and end points of the trajectory. To what extent did the calibration remove the drift? If there is still drift, why?

**Note:** Pay attention to the plotting scale so that the labels of the axes are properly shown.

**Note:** Remember to calibrate the IMU data first with the bias and scale factor values obtained from `exercise#A1` before plotting Figure#2. Re-mention these values briefly. 

Steps: 

Part A2
1. Run the rosbag inside the given docker container, and store rosbag imu data in .txt or any other preferable format for easier access.
2. Calibration: eliminate known imu bias and scale factor errors using the result from `exercise#A1`
3. Mechanization: integrate to obtain the orientation, velocity, and position with respect to time. Express the positions in the first time frame to obtain the trajectory.
4. Plot the trajectory.
5. Compare results by integrating without calibration.
6. Discuss.

## CBL

See description in `exercise#A1`.
