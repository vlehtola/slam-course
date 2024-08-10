
This assignment is under development.

# Multi-sensor fusion
This assignment is about multi-sensor fusion. The backpack has 3 hokuyo lidars.

![Tape measures from the multi-sensor backpack](https://github.com/vlehtola/slam-course/blob/main/hokuyo%20backpack.jpg "Multi-sensor backpack")

Dockerfile is in the part B folder.

The multi-lidar fusion can be done basically in two fashions:
- fuse the lidar data before cartographer
- use the trajectory from the cartographer to register the data of other lidars

# Fuse the lidar data before cartographer

Cartographer provides one input ros topic called 'scan' in the 2D SLAM mode. How could the data from two topics (=two lidars) be merged into one?

# Registering the data of other lidars

Registration of lidar data and analysis of a 3D point cloud. If horizontal lidar is s2, other lidars are s1 and s0 (check this). 
Read the data (msgs for /scan1 and /scan0) from the bag file, see the following 
[link for instructions](http://wiki.ros.org/rosbag/Tutorials/reading%20msgs%20from%20a%20bag%20file)

The data is in the form of ranges, and there is also the information about the starting angle and the angles between the beams. These form a cylinder coordinate system c1 in the sensor frame. Convert these points p_c1=(r,θ,z=0) into a XYZ system in the sensor frame p_(s_1 ). Use translation t_c1^s1=(0,0,0). You can use the python code snippet below to get started.

```python
from rosbag import Bag
import numpy as np

scan0={}
scan1={}

for topic, msg, t in Bag('demo_part_b.bag'):
  if topic == '/scan0':
        print( msg.header.seq)
        x=[]
        y=[]
        index=0 
        for i in msg.ranges:
            x.append(i*np.sin(msg.angle_min+msg.angle_increment*index))
            y.append(i*np.cos(msg.angle_min+msg.angle_increment*index))
            index+=1
        scan0[msg.header.seq] = x,y
```

Next, register the data into the body frame b with a 4x4 homogeneous transformation matrix H_(s_1)^b. Here, the body frame b is equal to the sensor frame s2. You may assume that all rotations are 90 degrees.

At this point, you should have all the data in the body frame. But the body frame is moving and we need to register all the data into the world frame, which in this case is the first scan frame (or pose). 

Use the trajectory that you extracted for part b (either lidar or lidar-inertial fusion) to register the data from the two non-horizontal lidars (s0 and s1) into the first frame f1. Augment the trajectory data with time stamps as follows (this is rough way and we do it for the sake of the exercise only): read the time from the first and the last message in the bag file. Using these two times, interpolate timestamps for each trajectory pose.

For each scan, read the timestamp and find the trajectory frame (or pose) with the closest matching timestamp. Use this frame (and calculate the platform orientation in 2D from the two nearest neighboring frame positions of the trajectory if not otherwise available).
Consider that you are constructing a chain of transformations from the second frame to the first frame H_(s_2)^(s_1 ), from the the third to the second H_(s_3)^(s_2 ), and so on to the frame N that you are handling to register all the data
H_(f_N)^(f_1 )=〖H_(f_2)^(f_1 ) H〗_(f_3)^(f_2 )…H_(f_N)^(f_(N-1) ).

The result is a 3D point cloud. Please take a snapshot of the point cloud and include it into your report. An easy way to do this is to install cloudCompare (www.cloudcompare.org ) and open the point cloud therein for visualization. Discuss briefly how you could detect objects such as doors from the data (those are big objects). Discuss briefly how the assumption that we have a 2D trajectory affects the results and the quality of the point cloud.

(Optional for 2 extra points in case you missed some points from previous assignments; you may use one extra page): Convert the point cloud into a 3D occupancy grid (see the updated lecture slides on RPCN applications). Show a figure. What is a suitable voxel size? Give approximate estimations of the dimensions of the test field. How high is the ceiling? How precise is your estimate? Discuss briefly what robotic applications could benefit from lidar-inertial perception systems.

