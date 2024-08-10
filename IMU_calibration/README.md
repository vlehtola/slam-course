# IMU calibration with the IMU data
v.v.lehtola@utwente.nl

## Objective
The objective of this practical is the rudimentary IMU calibration. We study accelerometers and gyroscopes and their bias and scale factor errors.

## Experiment
The experiment shall be done together in the class as follows. Align the IMU in the way that one of its axes is aligned with gravity. Record the IMU acceleration and angular rate of this axis for a while, e.g. 1 min. This allows for averaging the acceleration and angular rate over time. 
Then turn the IMU upside down in the way that the gravity is in the opposite direction. Record another time series and average the acceleration and angular rate. 
Repeat this procedure for two other axes (=up and down for all three axes).

Now turn off and turn on the IMU and repeat the whole procedure multiple times. Keep a measurement notebook on how the experiment went.

## Data preparation
This step may be done jointly in class. To open the proprietary data file from e.g. XSens MTi-300, either use mtb_to_text_python310 package or download the “MT Software Suite” from: https://www.movella.com/support/software-documentation  and install it.
Then go to the MT Manager folder and open and read MT Manager documentation. 

## Report
Table 1: calculated bias and scale factor error of acceleration and angular rate from the following equations introduced at the lecture 

![IMU calibration equations](https://github.com/vlehtola/slam-course/blob/main/IMU_calibration/eqs_imu_calib.PNG "Equations")

Calculate bias offset and scale factor error for each run. In addition, estimate run-to-run bias and scale factor instability. Can we estimate other error sources? Is yes, please do so.
Briefly discuss the results. Recommend calibration parameters for b_a,S_a,b_g,S_g that should be used for the IMU (=bias offset and scale factor error). For example, highlight these in the table and explain the
highlighting in the text. Use these to calibrate the IMU data.
