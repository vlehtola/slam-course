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
3. Utilize e.g. a python code to get average values for the specific force, f_up and f_down
4. Estimate scale factor and bias via equations
5. Report obtained values in Table 1 and Table 2(*) and discuss them.


## PPD

This section concerns only MROB students with ASAI or HSAI specialization. Note: The following questions are designed to guide your thinking. If a technology (IMU, LiDAR, etc.) is not applicable to your robot design (applies to A,B,C), state this explicitly and justify why alternative approaches are sufficient. You will be graded on engineering reasoning, not on using specific sensors.  Both using and not using a particular sensor can be correct choices depending on your system requirements.

Instead of (*), i.e. calculating run-to-run bias and scale factor instability, include the following into your CBL report (=not to `exercise#A1` report):

* Identify your robot's localization and motion sensing requirements (accuracy, environment, dynamics)
* Justify your chosen sensor approach
* If using IMU(s): Describe number/placement, quality requirements (drift, noise), and calibration methodology
* If not using IMU(s): Explain your chosen sensors (wheel encoders, LiDAR, GNSS, visual odometry, etc. explained later in the course), how these meet your requirements, and what tradeoffs you considered

These PPD report points may be written after the cognition challenge and the design of the system has been agreed upon within the group. The base idea should be there at the 2/3 mandatory Q&A session.
