import numpy as np
import matplotlib.pyplot as plt
import pickle
from scipy import integrate
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ================== LOAD DATA FROM ROSBAG ==================
print("Loading IMU data from rosbag...")

bag_file = '/home/xingyue/gnss2/data/imu_square_data/'  # CHANGE TO YOUR BAG FILE
storage_options = StorageOptions(uri=bag_file, storage_id='mcap')
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

reader = SequentialReader()
reader.open(storage_options, converter_options)

# Get message type
topic_types = reader.get_all_topics_and_types()
msg_type = get_message('custom_interfaces/msg/Vectornav')
for topic_info in topic_types:
    if topic_info.name == '/imu':
        msg_type = topic_info.type
        print(f"Found topic: {topic_info.name} with type: {msg_type}")
        break

if not msg_type:
    print("Error: /imu topic not found!")
    exit(1)

msg_class = get_message(msg_type)

# Read all messages and extract data
timestamps = []
mag_x = []
mag_y = []
mag_z = []
accel_x = []
accel_y = []
accel_z = []
gyro_x = []
gyro_y = []
gyro_z = []

print("Reading messages...")
message_count = 0
error_count = 0

while reader.has_next():
    (topic, data, timestamp) = reader.read_next()
    
    if topic == '/imu':
        try:
            msg = deserialize_message(data, msg_class)
            
            # Check if raw_imu_string is valid
            if not msg.raw_imu_string or not msg.raw_imu_string.startswith('$VNYMR'):
                error_count += 1
                continue
            
            # Parse yaw, pitch, roll from raw_imu_string
            clean_string = msg.raw_imu_string.strip()
            if '*' in clean_string:
                clean_string = clean_string.split('*')[0]
            
            data_parts = clean_string.split(',')
            if len(data_parts) < 13:
                error_count += 1
                continue
            
            # Extract timestamp (convert to seconds)
            ts = timestamp * 1e-9  # Convert nanoseconds to seconds
            timestamps.append(ts)
            
            # Extract sensor data from message
            mag_x.append(msg.mag_field.magnetic_field.x)
            mag_y.append(msg.mag_field.magnetic_field.y)
            mag_z.append(msg.mag_field.magnetic_field.z)
            
            accel_x.append(msg.imu.linear_acceleration.x)
            accel_y.append(msg.imu.linear_acceleration.y)
            accel_z.append(msg.imu.linear_acceleration.z)
            
            gyro_x.append(msg.imu.angular_velocity.x)
            gyro_y.append(msg.imu.angular_velocity.y)
            gyro_z.append(msg.imu.angular_velocity.z)
            
            message_count += 1
            
            if message_count % 10000 == 0:
                print(f"  Processed {message_count} messages...")
            
        except Exception as e:
            error_count += 1
            continue

print(f"\n✓ Successfully loaded {message_count} messages")
print(f"✗ Skipped {error_count} invalid messages")

# Convert to numpy arrays
timestamps = np.array(timestamps)
time = timestamps - timestamps[0]  # Start at t=0

mag_x = np.array(mag_x)
mag_y = np.array(mag_y)
mag_z = np.array(mag_z)

accel_x = np.array(accel_x)
accel_y = np.array(accel_y)
accel_z = np.array(accel_z)

gyro_x = np.array(gyro_x)
gyro_y = np.array(gyro_y)
gyro_z = np.array(gyro_z)

print(f"\nData summary:")
print(f"  Duration: {time[-1]:.2f} seconds")
print(f"  Sample rate: {len(time)/time[-1]:.2f} Hz")
print(f"  Data points: {len(time)}")


# ========== MAGNETOMETER CALIBRATION ==========
def calibrate_magnetometer(mag_x, mag_y):
    # Center the data
    x_offset = (mag_x.max() + mag_x.min()) / 2
    y_offset = (mag_y.max() + mag_y.min()) / 2
    
    x_centered = mag_x - x_offset
    y_centered = mag_y - y_offset
    
    # Scale to circle
    x_scale = (mag_x.max() - mag_x.min()) / 2
    y_scale = (mag_y.max() - mag_y.min()) / 2
    avg_scale = (x_scale + y_scale) / 2
    
    x_cal = x_centered * (avg_scale / x_scale)
    y_cal = y_centered * (avg_scale / y_scale)
        # Return as DICTIONARY (not tuple!)
    cal_params = {
        'x_offset': x_offset,
        'y_offset': y_offset,
        'x_scale': x_scale,
        'y_scale': y_scale,
        'avg_scale': avg_scale
    }

    
    return x_cal, y_cal, cal_params

# ========== INTEGRATION FUNCTION ==========
def cumulative_integrate(data, time):
    return integrate.cumtrapz(data, time, initial=0)

# Calibrate circle data
mag_x_cal, mag_y_cal, cal_params = calibrate_magnetometer(mag_x, mag_y)

# ========== SAVE CALIBRATION PARAMETERS ==========
with open('mag_calibration.pkl', 'wb') as f:
    pickle.dump(cal_params, f)
print("✓ Calibration parameters saved to mag_calibration.pkl")
print(f"  X offset: {cal_params['x_offset']:.4f}")
print(f"  Y offset: {cal_params['y_offset']:.4f}")

# ========== CIRCLE BAG ANALYSIS ==========
# Fig 1: Mag calibration

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
ax1.plot(mag_x, mag_y, 'b.', alpha=0.5, label='Raw')
ax1.set_xlabel('Mag East'); ax1.set_ylabel('Mag North')
ax1.set_title('Before Calibration'); ax1.axis('equal'); ax1.legend()

ax2.plot(mag_x_cal, mag_y_cal, 'r.', alpha=0.5, label='Calibrated')
ax2.set_xlabel('Mag East'); ax2.set_ylabel('Mag North')
ax2.set_title('After Calibration'); ax2.axis('equal'); ax2.legend()
plt.tight_layout(); plt.show()

# Fig 2: X-axis gyro
theta_x = cumulative_integrate(gyro_x, time)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
ax1.plot(time, gyro_x); ax1.set_ylabel('Gyro X (rad/s)'); ax1.set_title('X Rotation')
ax2.plot(time, theta_x); ax2.set_ylabel('Angle X (rad)'); ax2.set_xlabel('Time (s)')
plt.tight_layout(); plt.show()

# Fig 3: Y-axis gyro
theta_y = cumulative_integrate(gyro_y, time)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
ax1.plot(time, gyro_y); ax1.set_ylabel('Gyro Y (rad/s)'); ax1.set_title('Y Rotation')
ax2.plot(time, theta_y); ax2.set_ylabel('Angle Y (rad)'); ax2.set_xlabel('Time (s)')
plt.tight_layout(); plt.show()

# Fig 4: Z-axis + magnetometer heading
theta_z = cumulative_integrate(gyro_z, time)
mag_heading = np.arctan2(mag_y_cal, mag_x_cal)

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
ax1.plot(time, gyro_z); ax1.set_ylabel('Gyro Z (rad/s)'); ax1.set_title('Z Rotation')
ax2.plot(time, theta_z); ax2.set_ylabel('Angle Z (rad)')
ax3.plot(time, mag_heading); ax3.set_ylabel('Mag Heading (rad)'); ax3.set_xlabel('Time (s)')
plt.tight_layout(); plt.show()

# Fig 5: X acceleration
vel_x = cumulative_integrate(accel_x, time)
pos_x = cumulative_integrate(vel_x, time)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
ax1.plot(time, accel_x); ax1.set_ylabel('Accel X (m/s²)'); ax1.set_title('X Motion')
ax2.plot(time, vel_x); ax2.set_ylabel('Velocity X (m/s)')
plt.tight_layout(); plt.show()

# Fig 6: Y acceleration
vel_y = cumulative_integrate(accel_y, time)
pos_y = cumulative_integrate(vel_y, time)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
ax1.plot(time, accel_y); ax1.set_ylabel('Accel Y (m/s²)'); ax1.set_title('Y Motion')
ax2.plot(time, vel_y); ax2.set_ylabel('Velocity Y (m/s)')
plt.tight_layout(); plt.show()

# Fig 7: Z acceleration
vel_z = cumulative_integrate(accel_z, time)
pos_z = cumulative_integrate(vel_z, time)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
ax1.plot(time, accel_z); ax1.set_ylabel('Accel Z (m/s²)'); ax1.set_title('Z Motion')
ax2.plot(time, vel_z); ax2.set_ylabel('Velocity Z (m/s)')
plt.tight_layout(); plt.show()

# ========== NEW PLOT: N vs E Position with Two Heading Sources ==========

# Get displacement in X (already calculated from Fig 5)
displacement_x = pos_x  # This is your twice-integrated acceleration

# Method 1: Use magnetometer heading
east_mag = displacement_x * np.cos(mag_heading)
north_mag = displacement_x * np.sin(mag_heading)

# Method 2: Use gyro heading (integrated gyro_z)
east_gyro = displacement_x * np.cos(theta_z)
north_gyro = displacement_x * np.sin(theta_z)

# Plot both trajectories
fig, ax = plt.subplots(1, 1, figsize=(10, 10))

# Magnetometer-based trajectory
ax.plot(east_mag, north_mag, 'b.-', alpha=0.7, linewidth=1.5, 
        markersize=2, label='Magnetometer Heading')

# Gyro-based trajectory
ax.plot(east_gyro, north_gyro, 'r.-', alpha=0.7, linewidth=1.5, 
        markersize=2, label='Gyro Heading')

# Mark start and end points
ax.plot(east_mag[0], north_mag[0], 'go', markersize=10, label='Start')
ax.plot(east_mag[-1], north_mag[-1], 'mo', markersize=10, label='End (Mag)')
ax.plot(east_gyro[-1], north_gyro[-1], 'ko', markersize=10, label='End (Gyro)')

ax.set_xlabel('East Position (m)', fontsize=12)
ax.set_ylabel('North Position (m)', fontsize=12)
ax.set_title('Trajectory: N vs E Position (Magnetometer vs Gyro Heading)', fontsize=14)
ax.axis('equal')  # Keep aspect ratio square
ax.grid(True, alpha=0.3)
ax.legend(fontsize=10)
plt.tight_layout()
plt.show()
