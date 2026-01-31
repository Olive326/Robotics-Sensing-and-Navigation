#!/usr/bin/env python3

import rosbag2_py
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os
import rclpy
from scipy import stats

def read_bag_file(bag_path):
    """Read GPS messages from MCAP rosbag file"""
    
    if not rclpy.ok():
        rclpy.init()
    
    bag_path = os.path.abspath(os.path.expanduser(bag_path))
    
    if not os.path.isdir(bag_path):
        raise FileNotFoundError(f"Bag directory not found: {bag_path}")
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    msg_type = get_message('gps_driver/msg/Customgps')
    gps_data = []
    
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name == '/gps':
            try:
                msg = deserialize_message(data, msg_type)
                gps_data.append({
                    'timestamp': timestamp * 1e-9,
                    'utm_easting': msg.utm_easting,
                    'utm_northing': msg.utm_northing,
                    'altitude': msg.altitude,
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'hdop': msg.hdop
                })
            except Exception:
                continue
    
    reader.close()
    return pd.DataFrame(gps_data)

def calculate_centroid_and_center_data(df):
    """Calculate centroid and center the data"""
    centroid_east = df['utm_easting'].mean()
    centroid_north = df['utm_northing'].mean()
    
    df_centered = df.copy()
    df_centered['easting_centered'] = df['utm_easting'] - centroid_east
    df_centered['northing_centered'] = df['utm_northing'] - centroid_north
    
    return df_centered, (centroid_east, centroid_north)

def euclidean_distance_from_centroid(df_centered):
    """Calculate Euclidean distance from centroid"""
    return np.sqrt(df_centered['easting_centered']**2 + df_centered['northing_centered']**2)

def create_all_plots():
    # Try to read all datasets
    datasets = {}
    centroids = {}
    
    base_path = '/home/xingyue/EECE5554_old/gnss/preprocessed_data'
    
    # Check for available datasets
    for name, folder in [('open', 'open_data'), ('occluded', 'occluded_data'), ('moving', 'moving_data')]:
        path = os.path.join(base_path, folder)
        if os.path.exists(path):
            try:
                print(f"Loading {name} dataset...")
                datasets[name] = read_bag_file(path)
                datasets[name], centroids[name] = calculate_centroid_and_center_data(datasets[name])
                print(f"  Loaded {len(datasets[name])} points")
            except Exception as e:
                print(f"  Failed to load {name}: {e}")
    
    if not datasets:
        print("No datasets found!")
        return
    
    print(f"\nAvailable datasets: {list(datasets.keys())}")
    
    # Plot 1: Stationary N/E scatter plot
    plt.figure(figsize=(12, 8))
    colors = ['blue', 'red', 'green']
    markers = ['o', '^', 's']
    
    stationary_data = {k: v for k, v in datasets.items() if k in ['open', 'occluded']}
    
    if stationary_data:
        for i, (name, data) in enumerate(stationary_data.items()):
            plt.scatter(data['easting_centered'], data['northing_centered'], 
                       alpha=0.6, s=2, label=f'{name.capitalize()} area', 
                       color=colors[i], marker=markers[i])
        
        # Add centroid information
        text_info = []
        for name, (east, north) in centroids.items():
            if name in stationary_data:
                text_info.append(f'{name.capitalize()} Centroid: E={east:.2f}m, N={north:.2f}m')
        
        plt.xlabel('Easting - Centroid (m)')
        plt.ylabel('Northing - Centroid (m)')
        plt.title('Stationary GPS Data: Northing vs Easting (Centered)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # Add centroid text
        for i, info in enumerate(text_info):
            plt.text(0.02, 0.98-i*0.04, info, transform=plt.gca().transAxes, 
                    verticalalignment='top', bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        plt.savefig('1_stationary_scatter.png', dpi=300, bbox_inches='tight')
        plt.show()
        print("Saved: 1_stationary_scatter.png")
    
    # Plot 2: Stationary altitude vs time
    if stationary_data:
        plt.figure(figsize=(12, 6))
        
        for i, (name, data) in enumerate(stationary_data.items()):
            time_rel = data['timestamp'] - data['timestamp'].iloc[0]
            plt.scatter(time_rel, data['altitude'], alpha=0.6, s=2, 
                       label=f'{name.capitalize()} area', color=colors[i], marker=markers[i])
        
        plt.xlabel('Time (s)')
        plt.ylabel('Altitude (m)')
        plt.title('Stationary GPS Data: Altitude vs Time')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.savefig('2_stationary_altitude.png', dpi=300, bbox_inches='tight')
        plt.show()
        print("Saved: 2_stationary_altitude.png")
    
    # Plot 3: Euclidean distance histograms
    if stationary_data:
        fig, axes = plt.subplots(1, len(stationary_data), figsize=(6*len(stationary_data), 5))
        if len(stationary_data) == 1:
            axes = [axes]
        
        for i, (name, data) in enumerate(stationary_data.items()):
            distances = euclidean_distance_from_centroid(data)
            
            axes[i].hist(distances, bins=30, alpha=0.7, edgecolor='black', color=colors[i])
            axes[i].set_xlabel('Distance from Centroid (m)')
            axes[i].set_ylabel('Frequency')
            axes[i].set_title(f'{name.capitalize()} Area - Distance Distribution')
            axes[i].grid(True, alpha=0.3)
            
            # Add statistics
            axes[i].text(0.7, 0.8, f'Mean: {distances.mean():.2f}m\nStd: {distances.std():.2f}m', 
                        transform=axes[i].transAxes, verticalalignment='top',
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        plt.tight_layout()
        plt.savefig('3_distance_histograms.png', dpi=300, bbox_inches='tight')
        plt.show()
        print("Saved: 3_distance_histograms.png")
    
    # Plot 4: Moving data N/E with line of best fit
    if 'moving' in datasets:
        moving_data = datasets['moving']
        
        plt.figure(figsize=(10, 8))
        plt.scatter(moving_data['easting_centered'], moving_data['northing_centered'], 
                   alpha=0.6, s=3, c=range(len(moving_data)), cmap='viridis', label='GPS points')
        
        # Line of best fit
        slope, intercept, r_value, p_value, std_err = stats.linregress(
            moving_data['easting_centered'], moving_data['northing_centered'])
        
        line_x = np.array([moving_data['easting_centered'].min(), moving_data['easting_centered'].max()])
        line_y = slope * line_x + intercept
        plt.plot(line_x, line_y, 'r-', linewidth=2, 
                label=f'Best fit: y={slope:.3f}x+{intercept:.3f} (R²={r_value**2:.3f})')
        
        plt.xlabel('Easting - Centroid (m)')
        plt.ylabel('Northing - Centroid (m)')
        plt.title('Moving GPS Data: Northing vs Easting with Best Fit Line')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.colorbar(label='Time sequence')
        plt.axis('equal')
        
        # Add centroid info
        east, north = centroids['moving']
        plt.text(0.02, 0.98, f'Centroid: E={east:.2f}m, N={north:.2f}m', 
                transform=plt.gca().transAxes, verticalalignment='top',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        plt.savefig('4_moving_scatter.png', dpi=300, bbox_inches='tight')
        plt.show()
        print("Saved: 4_moving_scatter.png")
    else:
        print("No moving dataset found - skipping moving data scatter plot")
    
    # Plot 5: Moving data altitude vs time
    if 'moving' in datasets:
        moving_data = datasets['moving']
        time_rel = moving_data['timestamp'] - moving_data['timestamp'].iloc[0]
        
        plt.figure(figsize=(12, 6))
        plt.plot(time_rel, moving_data['altitude'], 'b-', linewidth=1, alpha=0.7, label='Altitude')
        plt.scatter(time_rel, moving_data['altitude'], s=1, alpha=0.5, c='blue')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Altitude (m)')
        plt.title('Moving GPS Data: Altitude vs Time')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Add statistics
        plt.text(0.7, 0.8, f'Alt Range: {moving_data["altitude"].min():.1f} - {moving_data["altitude"].max():.1f}m\n'
                          f'Alt Std: {moving_data["altitude"].std():.2f}m', 
                transform=plt.gca().transAxes, verticalalignment='top',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        plt.savefig('5_moving_altitude.png', dpi=300, bbox_inches='tight')
        plt.show()
        print("Saved: 5_moving_altitude.png")
    else:
        print("No moving dataset found - skipping moving altitude plot")
    
    print("\nAnalysis complete! Upload the generated PNG files to Canvas.")

if __name__ == '__main__':
    create_all_plots()
