# -*- coding: utf-8 -*-
"""
Created on Mon Dec 18 09:21:13 2023

Calculate the bias for acc and gyr for grading the assignments.
NOTE! One needs to use a data import script before this one. And that has been tested in windows only! (pyd file is a dll file)

@author: ville
"""

import os
import numpy as np
import glob

# Set constants
number_of_files = 24  # Adjust and include only sensible files. File order is critical!
directory_path = os.getcwd()
vertical_axis = ['Acc_X', 'Acc_Y', 'Acc_Z']  # Field experiment measurement order, calibration box
vertical_axis_gyr = ['Gyr_X', 'Gyr_Y', 'Gyr_Z']  # Field experiment measurement order, same as for acc
gravity_scaling = 2 * 9.80665
gyro_scaling = 2 * 0.0042 * np.sin(0.9114)  # Adjust for latitude

def calculate_averages_with_null_handling(directory):
    all_averages = {}
    pattern = os.path.join(directory, "MT_*.txt")
    matching_files = glob.glob(pattern)

    for filename in matching_files:
        if os.path.exists(filename):
            with open(filename, 'r') as file:
                lines = file.readlines()

                # Skip lines starting with `//` until we find the actual header
                header = None
                for line in lines:
                    if not line.startswith("//"):  # Skip comment lines
                        header = line.strip().split("\t")  # Assuming tab-separated columns
                        break

                if header is None:
                    print(f"No valid header found in {filename}, skipping this file.")
                    continue

                print(f"Processing file: {filename}")
                print(f"Header: {header}")

                # Check column indices for sensor data (only process required sensor columns)
                sensor_columns = ['Acc_X', 'Acc_Y', 'Acc_Z', 'Gyr_X', 'Gyr_Y', 'Gyr_Z']
                sensor_indices = {col: header.index(col) for col in sensor_columns if col in header}

                # Log missing columns if they aren't in the header
                missing_columns = [col for col in sensor_columns if col not in sensor_indices]
                if missing_columns:
                    print(f"Missing sensor columns in {filename}: {missing_columns}")

                if not sensor_indices:
                    print(f"No sensor columns found in {filename}, skipping this file.")
                    continue

                # Initialize sums and counts for each sensor column
                sums = {col: 0 for col in sensor_columns}
                counts = {col: 0 for col in sensor_columns}

                # Process the lines, skipping lines that match column names (PacketCounter, Acc_X, etc.)
                for line in lines[1:]:
                    values = line.strip().split("\t")

                    # Skip any line that contains the column names again
                    if values == header:
                        print(f"Skipping line with column names in {filename}")
                        continue

                    if line.startswith("//"):
                        continue  # Skip any additional comment lines

                    # Skip lines that don't have enough columns for sensor data
                    if len(values) < max(sensor_indices.values()) + 1:
                        print(f"Warning: Inconsistent column count in {filename}, skipping line: {line}")
                        continue

                    # Process sensor data only (ignore other columns like latitude, longitude, etc.)
                    for col, idx in sensor_indices.items():
                        try:
                            value = values[idx].strip()
                            if value and is_numeric(value):  # Non-empty and numeric value
                                sums[col] += float(value)
                                counts[col] += 1
                            else:
                                print(f"Missing or non-numeric value for {col} in file {filename}, line: {line}")
                        except (ValueError, IndexError):
                            # Skip if the value can't be converted to float or index is out of range
                            print(f"Non-numeric or missing value encountered in file {filename}, column {col}: {value}")

                # Compute averages for the sensor columns
                averages = {col: (sums[col] / counts[col] if counts[col] > 0 else None) for col in sensor_columns}
                all_averages[os.path.basename(filename)] = averages

    return all_averages


def mean_averages(dict1, dict2):
    # Calculates mean averages for corresponding keys in dict1 and dict2
    result = {}
    for key in dict1:
        val1 = dict1.get(key, None)
        val2 = dict2.get(key, None)
        if val1 is not None and val2 is not None:
            result[key] = (val1 + val2) / 2
        elif val1 is not None:
            result[key] = val1
        elif val2 is not None:
            result[key] = val2
        else:
            result[key] = None
    return result


def substract_averages(dict1, dict2):
    # Subtracts values, handling None cases
    result = {}
    for key in dict1:
        val1 = dict1.get(key, None)
        val2 = dict2.get(key, None)
        if val1 is not None and val2 is not None:
            result[key] = val1 - val2
        elif val1 is not None and val2 is None:
            result[key] = val1  # If one is None, treat None as 0
        elif val1 is None and val2 is not None:
            result[key] = -val2  # If the first is None, negate the second
        else:
            result[key] = None  # Both are None, result should be None
    return result


def is_numeric(value):
    try:
        float(value)
        return True
    except ValueError:
        return False


if __name__ == '__main__':
    results = {}
    count = 0
    
    # Use glob to dynamically find existing files that match the pattern
    files = sorted(glob.glob(os.path.join(directory_path, "MT_*_*.txt")))
    
    # Check if at least two files are present
    if len(files) < 2:
        print("Not enough files to process.")
        exit(1)
        
    all_averages = calculate_averages_with_null_handling(directory_path)

    # Iterate over the files, skipping by 2
    for i in range(1, len(files), 2):
        file_1 = files[i-1]
        file_2 = files[i]
        
        # Ensure both files exist in the computed averages
        if os.path.basename(file_1) in all_averages and os.path.basename(file_2) in all_averages:
            count += 1
            mean_val = mean_averages(all_averages[os.path.basename(file_1)], all_averages[os.path.basename(file_2)])
            difference_val = substract_averages(all_averages[os.path.basename(file_1)], all_averages[os.path.basename(file_2)])

            index = (i // 2) % 3
            key_acc = vertical_axis[index]
            key_gyr = vertical_axis_gyr[index]
            
            print("index")
            print(index)

                        # Process all sensor columns without index cycling
            for key_acc, key_gyr in zip(vertical_axis, vertical_axis_gyr):
                # Ensure keys are present and values are numeric before processing
                if key_acc in mean_val and key_gyr in mean_val:
                    if is_numeric(mean_val[key_acc]) and is_numeric(difference_val[key_acc]):
                        results[f"{os.path.basename(file_1)}_{os.path.basename(file_2)}_{key_acc}"] = [
                            mean_val[key_acc],
                            (difference_val[key_acc] - gravity_scaling) / gravity_scaling
                        ]
                    if is_numeric(mean_val[key_gyr]) and is_numeric(difference_val[key_gyr]):
                        results[f"{os.path.basename(file_1)}_{os.path.basename(file_2)}_{key_gyr}"] = [
                            mean_val[key_gyr],
                            (difference_val[key_gyr] - gyro_scaling) / gyro_scaling
                        ]
                else:
                    # Log warning if key is missing
                    if key_acc not in mean_val:
                        print(f"Warning: {key_acc} missing from file {file_1} or {file_2}")
                    if key_gyr not in mean_val:
                        print(f"Warning: {key_gyr} missing from file {file_1} or {file_2}")
        else:
            print(f"Warning: One or both files missing: {file_1}, {file_2}")
    

    # Initialize sums and counts for averaging over all results
    sums = {
        'Acc_X': 0, 'Acc_Y': 0, 'Acc_Z': 0,
        'Gyr_X': 0, 'Gyr_Y': 0, 'Gyr_Z': 0,
        'Acc_SX': 0, 'Acc_SY': 0, 'Acc_SZ': 0,
        'Gyr_SX': 0, 'Gyr_SY': 0, 'Gyr_SZ': 0
    }
    counts = {axis: 0 for axis in sums}

    # Sum values for each axis and sensor type
    for key, value in results.items():
        if 'Acc_X' in key:
            sums['Acc_X'] += value[0]
            sums['Acc_SX'] += value[1]
            counts['Acc_X'] += 1
            counts['Acc_SX'] += 1
        elif 'Acc_Y' in key:
            sums['Acc_Y'] += value[0]
            sums['Acc_SY'] += value[1]
            counts['Acc_Y'] += 1
            counts['Acc_SY'] += 1
        elif 'Acc_Z' in key:
            sums['Acc_Z'] += value[0]
            sums['Acc_SZ'] += value[1]
            counts['Acc_Z'] += 1
            counts['Acc_SZ'] += 1
        elif 'Gyr_X' in key:
            sums['Gyr_X'] += value[0]
            sums['Gyr_SX'] += value[1]
            counts['Gyr_X'] += 1
            counts['Gyr_SX'] += 1
        elif 'Gyr_Y' in key:
            sums['Gyr_Y'] += value[0]
            sums['Gyr_SY'] += value[1]
            counts['Gyr_Y'] += 1
            counts['Gyr_SY'] += 1
        elif 'Gyr_Z' in key:
            sums['Gyr_Z'] += value[0]
            sums['Gyr_SZ'] += value[1]
            counts['Gyr_Z'] += 1
            counts['Gyr_SZ'] += 1
    
    # Calculate overall averages, handle division by zero
    averages = {axis: sums[axis] / counts[axis] if counts[axis] > 0 else None for axis in sums}
    
    print("Final Averages:", averages)

