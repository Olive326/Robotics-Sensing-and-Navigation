#!/usr/bin/env python3
"""
GPS Data Separator
Separates a single GPS bag file into three separate MCAP bag directories:
- occluded_data/
- moving_data/ 
- open_data/
Each containing their respective GPS messages in MCAP format
"""

import rosbag2_py
import pandas as pd
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import os
import rclpy

def read_and_analyze_bag(bag_path):
    """Read GPS messages and analyze for segmentation"""
    
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
    
    print("Reading and analyzing GPS messages...")
    messages = []  # Store all messages with metadata
    message_count = 0
    
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name == '/gps':
            try:
                msg = deserialize_message(data, msg_type)
                
                # Store message with all info needed for segmentation
                message_data = {
                    'topic_name': topic_name,
                    'raw_data': data,  # Keep original serialized data
                    'timestamp': timestamp,
                    'msg': msg,
                    'hdop': msg.hdop,
                    'utm_easting': msg.utm_easting,
                    'utm_northing': msg.utm_northing,
                    'time_rel': (timestamp * 1e-9)
                }
                
                messages.append(message_data)
                message_count += 1
                
                if message_count % 500 == 0:
                    print(f"  Processed {message_count} messages...")
                    
            except Exception as e:
                print(f"Error parsing message {message_count}: {e}")
                continue
    
    reader.close()
    print(f"Successfully loaded {len(messages)} GPS messages")
    return messages

def segment_messages(messages):
    """Segment messages into three categories"""
    
    print(f"\n=== SEGMENTING {len(messages)} GPS MESSAGES ===")
    
    if not messages:
        return {}
    
    # Convert to DataFrame for easier analysis
    df = pd.DataFrame([{
        'timestamp': msg['timestamp'],
        'time_rel': msg['time_rel'] - messages[0]['time_rel'],
        'hdop': msg['hdop'],
        'utm_easting': msg['utm_easting'],
        'utm_northing': msg['utm_northing']
    } for msg in messages])
    
    # Calculate movement metrics
    df['easting_diff'] = df['utm_easting'].diff().abs()
    df['northing_diff'] = df['utm_northing'].diff().abs()
    df['position_change'] = np.sqrt(df['easting_diff']**2 + df['northing_diff']**2)
    
    # Smooth the data for better segmentation
    window = min(50, len(df)//10)
    df['hdop_smooth'] = df['hdop'].rolling(window=window, center=True).mean()
    df['movement_smooth'] = df['position_change'].rolling(window=window, center=True).mean()
    
    total_duration = df['time_rel'].max()
    print(f"Total duration: {total_duration:.1f} seconds")
    print(f"HDOP range: {df['hdop'].min():.2f} to {df['hdop'].max():.2f}")
    print(f"Max position change: {df['position_change'].max():.3f}m")
    
    # Define thresholds
    HDOP_POOR_THRESHOLD = 5        # HDOP > 5 = occluded
    HDOP_GOOD_THRESHOLD = 1      # HDOP < 1 = open  
    MOVEMENT_THRESHOLD = 10         # > 10m = moving
    
    print(f"\nUsing thresholds:")
    print(f"  OCCLUDED: HDOP > {HDOP_POOR_THRESHOLD}")
    print(f"  OPEN: HDOP < {HDOP_GOOD_THRESHOLD}")  
    print(f"  MOVING: Position change > {MOVEMENT_THRESHOLD}m")
    
    # Create segment masks
    occluded_mask = df['hdop_smooth'] > HDOP_POOR_THRESHOLD
    open_mask = df['hdop_smooth'] < HDOP_GOOD_THRESHOLD
    moving_mask = df['movement_smooth'] > MOVEMENT_THRESHOLD
    
    segments = {}
    
    # Apply masks to original messages
    if occluded_mask.any():
        segments['occluded'] = [messages[i] for i in range(len(messages)) if occluded_mask.iloc[i]]
    
    if open_mask.any():
        segments['open'] = [messages[i] for i in range(len(messages)) if open_mask.iloc[i]]
    
    if moving_mask.any():
        segments['moving'] = [messages[i] for i in range(len(messages)) if moving_mask.iloc[i]]
    
    # Fallback: Time-based segmentation if characteristic-based fails
    if len(segments) < 2 or any(len(seg) < 100 for seg in segments.values()):
        print(f"\nCharacteristic-based segmentation produced small segments.")
        print(f"Using time-based segmentation as backup...")
        
        segments = {}
        third_1 = total_duration / 3
        third_2 = 2 * total_duration / 3
        
        segments['occluded'] = [msg for msg in messages if msg['time_rel'] - messages[0]['time_rel'] < third_1]
        segments['moving'] = [msg for msg in messages if third_1 <= msg['time_rel'] - messages[0]['time_rel'] < third_2]
        segments['open'] = [msg for msg in messages if msg['time_rel'] - messages[0]['time_rel'] >= third_2]
        
        print(f"Time-based segments:")
        print(f"  OCCLUDED: 0 to {third_1:.0f}s")
        print(f"  MOVING: {third_1:.0f} to {third_2:.0f}s")
        print(f"  OPEN: {third_2:.0f} to {total_duration:.0f}s")
    
    # Print segment statistics
    print(f"\n=== SEGMENTATION RESULTS ===")
    for name, segment_msgs in segments.items():
        if segment_msgs:
            print(f"{name.upper()}: {len(segment_msgs)} messages")
    
    return segments

def write_segment_bag(segment_messages, output_path, segment_name):
    """Write a segment of messages to a new MCAP bag file"""
    
    if not segment_messages:
        print(f"No messages for {segment_name}, skipping...")
        return False
    
    print(f"Writing {segment_name} bag with {len(segment_messages)} messages...")
    
    # Create output directory
#    os.makedirs(output_path, exist_ok=True)
    
    # Setup bag writer
    storage_options = rosbag2_py.StorageOptions(uri=output_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
    
    # Create topic info
    topic_info = rosbag2_py.TopicMetadata(   # All parameters provided
    id=0,
    name='/gps', 
    type='gps_driver/msg/Customgps',
    serialization_format='cdr'
)
    
    writer.create_topic(topic_info)
    
    # Write all messages
    for msg_data in segment_messages:
        writer.write('/gps', msg_data['raw_data'], msg_data['timestamp'])
    
    writer.close()
    print(f"  Saved {segment_name} → {output_path}")
    return True

def create_summary_report(segments, output_base_dir):
    """Create a summary report of the segmentation"""
    
    report_path = os.path.join(output_base_dir, "segmentation_summary.txt")
    
    with open(report_path, 'w') as f:
        f.write("GPS DATA SEGMENTATION SUMMARY\n")
        f.write("="*40 + "\n\n")
        
        f.write(f"Total segments created: {len(segments)}\n\n")
        
        for name, segment_msgs in segments.items():
            if segment_msgs:
                f.write(f"{name.upper()} SEGMENT:\n")
                f.write(f"  Messages: {len(segment_msgs)}\n")
                
                # Calculate time span
                timestamps = [msg['time_rel'] for msg in segment_msgs]
                time_span = max(timestamps) - min(timestamps)
                f.write(f"  Time span: {time_span:.1f}s\n")
                
                # Calculate HDOP range
                hdop_values = [msg['hdop'] for msg in segment_msgs]
                f.write(f"  HDOP range: {min(hdop_values):.2f} - {max(hdop_values):.2f}\n")
                f.write(f"  Directory: {name}_data/\n")
                f.write("\n")
        
        f.write("Created directories:\n")
        for name in segments.keys():
            f.write(f"  {name}_data/\n")
    
    print(f"Summary report saved: {report_path}")
    return report_path

def main():
    """Main function to segment GPS bag into three separate bags"""
    
    # Configuration
    input_bag = '/home/xingyue/EECE5554_old/gnss/data/open_area_data'
    output_base_dir = '/home/xingyue/EECE5554_old/gnss/preprocessed_data'
    
    print("GPS Bag Separator")
    print("="*50)
    print(f"Input bag: {input_bag}")
    print(f"Output base directory: {output_base_dir}")
    
    try:
        # Check if input exists
        if not os.path.exists(input_bag):
            print(f"ERROR: Input bag not found: {input_bag}")
            return
        
        # Read and analyze GPS data
        messages = read_and_analyze_bag(input_bag)
        
        if not messages:
            print("ERROR: No GPS messages found in bag file!")
            return
        
        # Segment the messages
        segments = segment_messages(messages)
        
        if not segments:
            print("ERROR: No segments could be created!")
            return
        
        # Write each segment to separate bag directories
        created_bags = []
        for segment_name, segment_msgs in segments.items():
            if segment_msgs:
                output_path = os.path.join(output_base_dir, f"{segment_name}_data")
                success = write_segment_bag(segment_msgs, output_path, segment_name)
                if success:
                    created_bags.append(f"{segment_name}_data")
        
        # Create summary report
        summary_path = create_summary_report(segments, output_base_dir)
        
        print("\n" + "="*50)
        print("SEGMENTATION COMPLETE!")
        print("="*50)
        print(f"Created {len(created_bags)} bag directories:")
        for bag_name in created_bags:
            bag_path = os.path.join(output_base_dir, bag_name)
            print(f"  {bag_path}/")
        
        print(f"\nYou can now use these bags with your original analysis script:")
        print(f"  ros2 bag info {output_base_dir}/occluded_data")
        print(f"  ros2 bag info {output_base_dir}/moving_data") 
        print(f"  ros2 bag info {output_base_dir}/open_data")
        
        print(f"\nNext step:")
        print(f"Update your analysis script paths to use these new bag directories!")
        
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
