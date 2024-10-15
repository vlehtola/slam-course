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
Part 1a
1. Record the IMU data with MT Manager 2.0 in class
2. Open and store in .txt file
3. Utilize data_analyse python file to get average imu values
4. estimate scale factor and bias via equations
Part 2b
1. run rosbag and store rosbag imu data in .txt or any other preferable format.
2. plot the results
3. integrate imu bias and scale factor to store data using data_analysis.py
4. plot the results
5. compare results
