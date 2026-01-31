#!/usr/bin/env python3
"""
HDOP Range Analyzer
Analyzes the HDOP distribution in your GPS bag file to determine proper thresholds
"""

import rosbag2_py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os
import rclpy

def analyze_hdop_range(bag_path):
    """Analyze HDOP range and distribution in GPS bag file"""
    
    if not rclpy.ok():
        rclpy.init()
    
    bag_path = os.path.abspath(os.path.expanduser(bag_path))
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    msg_type = get_message('gps_driver/msg/Customgps')
    hdop_data = []
    timestamps = []
    
    print("Reading GPS messages to analyze HDOP...")
    message_count = 0
    
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name == '/gps':
            try:
                msg = deserialize_message(data, msg_type)
                hdop_data.append(msg.hdop)
                timestamps.append(timestamp * 1e-9)
                message_count += 1
                
            except Exception as e:
                continue
    
    reader.close()
    
    if not hdop_data:
        print("No HDOP data found!")
        return
    
    # Convert to arrays for analysis
    hdop_array = np.array(hdop_data)
    time_array = np.array(timestamps)
    time_rel = time_array - time_array[0]
    
    print(f"\n=== HDOP ANALYSIS RESULTS ===")
    print(f"Total messages: {len(hdop_data)}")
    print(f"HDOP Range: {hdop_array.min():.2f} to {hdop_array.max():.2f}")
    print(f"HDOP Mean: {hdop_array.mean():.2f}")
    print(f"HDOP Median: {np.median(hdop_array):.2f}")
    print(f"HDOP Std: {hdop_array.std():.2f}")
    
    # Percentile analysis
    print(f"\nHDOP Percentiles:")
    percentiles = [10, 25, 33, 50, 67, 75, 90]
    for p in percentiles:
        value = np.percentile(hdop_array, p)
        print(f"  {p}th percentile: {value:.2f}")
    
    # Distribution analysis
    excellent_count = np.sum(hdop_array < 2.0)
    good_count = np.sum((hdop_array >= 2.0) & (hdop_array < 5.0))
    poor_count = np.sum(hdop_array >= 5.0)
    
    print(f"\nHDOP Quality Distribution:")
    print(f"  Excellent (<2.0): {excellent_count} messages ({excellent_count/len(hdop_data)*100:.1f}%)")
    print(f"  Good (2.0-5.0): {good_count} messages ({good_count/len(hdop_data)*100:.1f}%)")
    print(f"  Poor (≥5.0): {poor_count} messages ({poor_count/len(hdop_data)*100:.1f}%)")
    
    # Suggested thresholds based on data distribution
    print(f"\n=== SUGGESTED THRESHOLDS FOR YOUR DATA ===")
    
    # Use 33rd and 67th percentiles for natural divisions
    threshold_low = np.percentile(hdop_array, 33)
    threshold_high = np.percentile(hdop_array, 67)
    
    print(f"Based on your data distribution:")
    print(f"  OPEN (good signal): HDOP < {threshold_low:.1f}")
    print(f"  MOVING (fair signal): {threshold_low:.1f} ≤ HDOP < {threshold_high:.1f}")
    print(f"  OCCLUDED (poor signal): HDOP ≥ {threshold_high:.1f}")
    
    # Alternative thresholds
    print(f"\nAlternative thresholds (if above doesn't work):")
    median_val = np.median(hdop_array)
    print(f"  OPEN: HDOP < {median_val:.1f}")
    print(f"  OCCLUDED: HDOP ≥ {median_val:.1f}")
    
    # Create visualization
    plt.figure(figsize=(15, 10))
    
    # Plot 1: HDOP over time
    plt.subplot(2, 2, 1)
    plt.plot(time_rel, hdop_array, 'b-', alpha=0.7, linewidth=1)
    plt.axhline(y=threshold_low, color='green', linestyle='--', label=f'Open threshold ({threshold_low:.1f})')
    plt.axhline(y=threshold_high, color='red', linestyle='--', label=f'Occluded threshold ({threshold_high:.1f})')
    plt.xlabel('Time (s)')
    plt.ylabel('HDOP')
    plt.title('HDOP vs Time')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 2: HDOP histogram
    plt.subplot(2, 2, 2)
    plt.hist(hdop_array, bins=50, alpha=0.7, edgecolor='black')
    plt.axvline(x=threshold_low, color='green', linestyle='--', label=f'Open ({threshold_low:.1f})')
    plt.axvline(x=threshold_high, color='red', linestyle='--', label=f'Occluded ({threshold_high:.1f})')
    plt.xlabel('HDOP Value')
    plt.ylabel('Frequency')
    plt.title('HDOP Distribution')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 3: Cumulative distribution
    plt.subplot(2, 2, 3)
    sorted_hdop = np.sort(hdop_array)
    cumulative = np.arange(1, len(sorted_hdop) + 1) / len(sorted_hdop)
    plt.plot(sorted_hdop, cumulative, 'b-', linewidth=2)
    plt.axvline(x=threshold_low, color='green', linestyle='--', label=f'33rd percentile')
    plt.axvline(x=threshold_high, color='red', linestyle='--', label=f'67th percentile')
    plt.xlabel('HDOP Value')
    plt.ylabel('Cumulative Probability')
    plt.title('HDOP Cumulative Distribution')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 4: Box plot
    plt.subplot(2, 2, 4)
    plt.boxplot(hdop_array, vert=True)
    plt.ylabel('HDOP')
    plt.title('HDOP Box Plot')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('hdop_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()
    print(f"\nSaved HDOP analysis plot: hdop_analysis.png")
    
    return threshold_low, threshold_high

def main():
    """Main function"""
    
    bag_path = '/home/xingyue/EECE5554/gnss/data/open_area_data'
    
    print("HDOP Range Analyzer")
    print("="*50)
    print(f"Analyzing: {bag_path}")
    
    try:
        thresholds = analyze_hdop_range(bag_path)
        
        if thresholds:
            threshold_low, threshold_high = thresholds
            
            print(f"\n" + "="*50)
            print("RECOMMENDATION:")
            print("="*50)
            print("Update your gps_data_separator.py with these thresholds:")
            print(f"    HDOP_POOR_THRESHOLD = {threshold_high:.1f}      # HDOP ≥ {threshold_high:.1f} = occluded")
            print(f"    HDOP_GOOD_THRESHOLD = {threshold_low:.1f}      # HDOP < {threshold_low:.1f} = open")
            print("    MOVEMENT_THRESHOLD = 0.05      # > 5cm = moving")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
