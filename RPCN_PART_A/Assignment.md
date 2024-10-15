# Pedestrian dead reckoning 

Calculate a trajectory by integrating the IMU data over time. Perform the IMU integration in 2D and/or 3D. See README how to access the data through docker.

## Report
Figure#1: Plot the 2D X-Y trajectory with the recorded data (that was stored in the form of .txt via running `rosbag`). 

Figure#2: Plot the 2D X-Y trajectory with the refined recorded data (that was biased and scale-corrected)

**Note:** Pay attention to the plotting scale so that the labels of the axes are properly shown.

Discuss: 
According to the data, did the IMU arrive in the same place from where it started? Estimate the magnitude of drift. If there is drift, why?

**Note:** Remember to calibrate the IMU data first with the bias and scale factor values obtained from `exercise#A1` before plotting Figure#2. Mention these values if they have not been mentioned before. To what extent did the calibration remove the drift?


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
