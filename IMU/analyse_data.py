# -*- coding: utf-8 -*-
"""
Created on Mon Dec 18 09:21:13 2023

Calculate the bias for acc and gyr for grading the assignments.
NOTE! One needs to use a data import script before this one. And that has been tested in windows only! (pyd file is a dll file)

@author: ville
"""

import os
import numpy as np

number_of_files = 24                            # adjust and include only sensible files. file order is critical!
directory_path = ""
vertical_axis = ['acc_x','acc_y','acc_z']       # field experiment measurement order, calibration box
vertical_axis_gyr = ['gyr_x','gyr_y','gyr_z']   # field experiment measurement order, the same than for acc
# Earths rotation ωe is 360/(24 ∗ 3600) = 0.0042deg/s. Latitude of UT campus is 52.22◦ = 0.9114rad. 
gravity_scaling = 2* 9.80665
gyro_scaling = 2* 0.0042 * np.sin(0.9114) #*(2*np.pi) / 360

def calculate_averages_with_null_handling(directory):
    all_averages = {}
    for i in range(0, number_of_files):
        filename = os.path.join(directory, f"measurement_{i:03}.txt")
        if os.path.exists(filename):
            with open(filename, 'r') as file:
                lines = file.readlines()
                header = lines[0]
                topics = header.split(",")
                lines = lines[1:]  # Skip header line
                if not lines:
                    continue

                sums = [0] * len(lines[0].split(','))
                counts = [0] * len(lines[0].split(','))

                # Sum up the values in each column, handling 'null'
                for line in lines:
                    values = line.split(',')
                    if len(values) != len(sums):
                        print(f"Error: inconsistent number of columns in {filename}")
                        break
                    for j, value in enumerate(values):
                        if value.strip().lower() != "null":
                            sums[j] += float(value)
                            counts[j] += 1

                # Calculate averages for this file, avoiding division by zero
                averages = {f"{topics[j]}": sums[j] / counts[j] if counts[j] > 0 else None for j in range(len(sums))}
                all_averages[f"measurement_{i:03}.txt"] = averages

    return all_averages

def mean_averages(avg1, avg2):
    """
    mean the averages of one file from another.

    :param avg1: Dictionary of averages for the first file.
    :param avg2: Dictionary of averages for the second file.
    :return: A dictionary with the differences.
    """
    differences = {}
    for key in avg1:
        if key in avg2:
            # Subtract averages if both columns exist in both files
            differences[key] = (avg2[key] + avg1[key]) / 2.0 if avg2[key] is not None and avg1[key] is not None else None
        else:
            # If a column is not present in both, we can't calculate the difference
            differences[key] = None
    return differences

def substract_averages(avg1, avg2):
    """
    substract the averages of one file from another.

    :param avg1: Dictionary of averages for the first file.
    :param avg2: Dictionary of averages for the second file.
    :return: A dictionary with the differences.
    """
    differences = {}
    for key in avg1:
        if key in avg2:
            # Subtract averages if both columns exist in both files
            differences[key] = max(avg2[key] - avg1[key], avg1[key] - avg2[key]) if avg2[key] is not None and avg1[key] is not None else None
        else:
            # If a column is not present in both, we can't calculate the difference
            differences[key] = None
    return differences



# Example usage

if __name__ == '__main__':
    results = {}
    count = 0
    all_averages = calculate_averages_with_null_handling(directory_path)
    for i in range(1, number_of_files,2):  # skip 2
        count +=1
        mean_val = mean_averages(all_averages[f"measurement_{i:03}.txt"], all_averages[f"measurement_{i-1:03}.txt"])
        difference_val = substract_averages(all_averages[f"measurement_{i-1:03}.txt"], all_averages[f"measurement_{i:03}.txt"])
        #print(f"Sum measurement_{i:03}.txt and measurement_{i-1:03}.txt:", differences)
        index = int( (i-1)%6/2 )
        key_acc = f"{vertical_axis[index]}"
        key_gyr = f"{vertical_axis_gyr[index]}"
        results[f"{i-1:03}_{i:03}_{key_acc}"] = [ mean_val[key_acc], (difference_val[key_acc] - gravity_scaling) / gravity_scaling]
        results[f"{i-1:03}_{i:03}_{key_gyr}"] = [ mean_val[key_gyr], (difference_val[key_gyr] - gyro_scaling) / gyro_scaling ]
        print(results)
        
    sums = {'acc_x': 0, 'acc_y': 0, 'acc_z': 0, 'gyr_x': 0, 'gyr_y': 0, 'gyr_z': 0, 'acc_sx': 0, 'acc_sy': 0, 'acc_sz': 0, 'gyr_sx': 0, 'gyr_sy': 0, 'gyr_sz': 0}
    counts = {'acc_x': 0, 'acc_y': 0, 'acc_z': 0, 'gyr_x': 0, 'gyr_y': 0, 'gyr_z': 0, 'acc_sx': 0, 'acc_sy': 0, 'acc_sz': 0, 'gyr_sx': 0, 'gyr_sy': 0, 'gyr_sz': 0}

    # Sum values for each axis and sensor type
    for key, value in results.items():
        if 'acc_x' in key:
            sums['acc_x'] += value[0]
            sums['acc_sx'] += value[1]
            counts['acc_x'] += 1
            counts['acc_sx'] += 1
        elif 'acc_y' in key:
            sums['acc_y'] += value[0]
            sums['acc_sy'] += value[1]
            counts['acc_y'] += 1
            counts['acc_sy'] += 1
        elif 'acc_z' in key:
            sums['acc_z'] += value[0]
            sums['acc_sz'] += value[1]
            counts['acc_z'] += 1
            counts['acc_sz'] += 1
        elif 'gyr_x' in key:
            sums['gyr_x'] += value[0]
            sums['gyr_sx'] += value[1]
            counts['gyr_x'] += 1
            counts['gyr_sx'] += 1
        elif 'gyr_y' in key:
            sums['gyr_y'] += value[0]
            sums['gyr_sy'] += value[1]
            counts['gyr_y'] += 1
            counts['gyr_sy'] += 1
        elif 'gyr_z' in key:
            sums['gyr_z'] += value[0]
            sums['gyr_sz'] += value[1]
            counts['gyr_z'] += 1
            counts['gyr_sz'] += 1
    
    # Calculate averages
    averages = {axis: sums[axis] / counts[axis] for axis in sums}
    
    print(averages)
    
# Note: Replace "path_to_the_directory_containing_the_files" with the actual directory path.
