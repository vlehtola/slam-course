
Use this assignment to refine your analytical and innovative thinking by exploring A-LOAM with NTU VIRAL Dataset and suggesting improvements

# NTUVIRAL DATASET with A-LOAM

In this assignment, students will:

 - Map and analyze key components from the A-LOAM paper to the A-LOAM codebase provided.
 - Suggest modifications or improvements based on their understanding of both the paper and the provided dataset (NTU VIRAL).

## Task 1: Mapping Key Components of the Paper to the Codebase
**Objective:** Identify and map specific components of algorithms discussed in the A-LOAM paper to corresponding functions or files in the provided A-LOAM codebase.

Instructions: Find how and wehere following tasks are achieved
- Scan Registration: How the lidar scan points are matched.
- Odometry Estimation: Estimating the relative motion between consecutive scans.
- Mapping: How the map is incrementally built and refined.
- Sensor Fusion: Merging different sensor data together
- Error metrics: Is there any optimization happening to improve the results? What are the error metrics 

Document your finding in the word file with screenshot of relevant coding pieces or use jupyter/Ipython Notebook (.ipynb) to formulate both text and code together.



## Task 2: Suggesting Improvements or Modifications
**Objective:** Propose changes to the existing algorithmic formulation or codebase, your analysis, or potential limitations in the implementation.

Instructions: Based on your analysis in Tasks 1, suggest one or more improvements. 
- Efficiency improvements: Can the code be optimized for faster processing (e.g., multithreading, efficient data structures)?
- Algorithmic improvements: Suggest modifications to improve accuracy, robustness, or scalability. For example, could you use RANSAC for better outlier rejection in scan registration?
- Feature enhancements: Are there any features that could be added? (e.g., loop closure, better IMU integration, or improvements to the pose graph optimization).

Write a brief proposal or pseudo-code explaining the proposed changes and how they might improve the performance or accuracy.

## Task 4 (Optional): Implement a Small Improvement
**Objective:** Implement one of your suggested improvements in the A-LOAM code.
