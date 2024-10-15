# Pedestrian dead reckoning 

Calculate a trajectory by integrating the IMU data over time. Perform the IMU integration in 2D and/or 3D. See README how to access the data through docker.

## Report
Figure: Plot the trajectory. Pay attention to the scale and to that the labels of the axes are properly shown.

Discuss: 
Did the IMU arrive on the same place than it started from, according to the data? Estimate the magnitude of drift. If there is drift, why?

Remember to calibrate the IMU first with the bias and scale factor values obtained from the previous exercise. Mention these values, if not mentioned before. To what extent did the calibration remove the drift?


(what kindof trajectory needs to be plotted? How integrate imu calib parameters?)

Integrate IMU calib folder with rpcn A exercise...
Steps:
Calculate scale factor and biase in class
store rosbag data in .txt or any preferable format.
plot the results
integrate imu bias and scale factor to store data using data_analysis.py
plot the results
