# IMU calibration with the IMU data
v.v.lehtola@utwente.nl

## Objective
The objective of this practical is the rudimentary IMU calibration. We study accelerometers and gyroscopes and their bias and scale factor errors.

## Experiment
The experiment shall be done together in the class as follows. Align the IMU in the way that one of its axes is aligned with gravity. Record the IMU acceleration and angular rate of this axis for a while, e.g. 1 min. This allows for averaging the acceleration and angular rate over time. 
Then turn the IMU upside down so that the gravity is in the opposite direction. Record another time series and average the acceleration and angular rate. 
Repeat this procedure for two other axes (=up and down for all three axes).

Now turn off and turn on the IMU and repeat the whole procedure multiple times. Keep a measurement notebook on how the experiment went.

## Data preparation
This step may be done jointly in class. To open the proprietary data file from e.g. XSens MTi-300, either use mtb_to_text_python310 package or download the “MT Software Suite” from: https://www.movella.com/support/software-documentation  and install it.
Then go to the MT Manager folder and open and read MT Manager documentation. 

## Data analysis

Calculate bias and scale factor error for acceleration and angular rate from the following equations introduced at the lecture, for each axis 

![IMU calibration equations](https://github.com/vlehtola/slam-course/blob/main/RPCN_PART_A1/eqs_imu_calib.PNG "Equations")


## Report

The report has a **page limit of 2 pages** and it must contain:

Table 1: Report calculated bias and scale factor error values in Table 1, for each axis.

Table 2 ( * ): In addition, estimate run-to-run bias and scale factor instability. Can we model or estimate other error sources? Is yes, please do so. If no, why not? (*: CBL students, see below)

Briefly discuss the results. Recommend calibration parameters for b_a,S_a,b_g,S_g that should be used for the IMU (=bias offset and scale factor error). For example, highlight these in the table 1 and explain the highlighting in the text. Use these to calibrate the IMU data for the followup task.

Steps:

Part A1
1. Record the IMU data with MT Manager 2.0 in class
2. Open and store in .txt file
3. Utilize e.g. data_analyse python file to get average values for the specific force, f_up and f_down
4. Estimate scale factor and bias via equations
5. Report obtained values in Table 1 and Table 2(*) and discuss them.


## CBL

This section concerns only MROB students with ASAI or HSAI specialization.

IMU(s) can be used in any environment, so we expect that these are considered for your CBL system. Instead of (*), i.e. calculating run-to-run bias and scale factor instability, include the following into your CBL report (=not to `exercise#A1` report):

*  How can one or multiple IMUs benefit your CBL robot design, and how should they be placed?
*  What should be the quality of the IMU(s) in terms of accelerometer and gyroscope drift?
*  Briefly describe methodology is used to calibrate the IMU(s), with respect to e.g. the biases

These CBL report points may be written after the design of the system has been agreed upon within the group. The base idea should be there at the 2/3 mandatory Q&A session.
