#!/usr/bin/env python3
import rosbag2_py
from rclpy.serialization import deserialize_message
from imu_interfaces.msg import IMUmsg
import numpy as np
import allantools
import matplotlib.pyplot as plt

def read_imu_bag(bag_path):
    """
    Read IMU data from ROS2 bag file
    Returns gyro data as numpy arrays for x, y, z axes
    """
    # Initialize storage
    gyro_x = []
    gyro_y = []
    gyro_z = []
    
    # Open bag
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Read messages
    while reader.has_next():
        try:
            (topic, data, timestamp) = reader.read_next()
            
            # Deserialize the message
            msg = deserialize_message(data, IMUmsg)
            
            # Extract gyro data (angular velocity)
            gyro_x.append(msg.imu.angular_velocity.x)
            gyro_y.append(msg.imu.angular_velocity.y)
            gyro_z.append(msg.imu.angular_velocity.z)
            
        except Exception as e:
            # Error handling for corrupted messages
            print(f"Warning: Skipping corrupted message - {e}")
            continue
    
    # Convert to numpy arrays
    gyro_x = np.array(gyro_x)
    gyro_y = np.array(gyro_y)
    gyro_z = np.array(gyro_z)
    
    print(f"Successfully read {len(gyro_x)} IMU messages")
    
    # DIAGNOSTIC PRINTS:
    print("\n=== Gyro Data Statistics ===")
    print(f"X axis - Mean: {np.mean(gyro_x):.8f}, Std: {np.std(gyro_x):.8f}")
    print(f"         Min: {np.min(gyro_x):.8f}, Max: {np.max(gyro_x):.8f}")
    print(f"Y axis - Mean: {np.mean(gyro_y):.8f}, Std: {np.std(gyro_y):.8f}")
    print(f"         Min: {np.min(gyro_y):.8f}, Max: {np.max(gyro_y):.8f}")
    print(f"Z axis - Mean: {np.mean(gyro_z):.8f}, Std: {np.std(gyro_z):.8f}")
    print(f"         Min: {np.min(gyro_z):.8f}, Max: {np.max(gyro_z):.8f}")
    
    # Check for constant values
    print(f"\nNumber of unique X values: {len(np.unique(gyro_x))}")
    print(f"Number of unique Y values: {len(np.unique(gyro_y))}")
    print(f"Number of unique Z values: {len(np.unique(gyro_z))}")
    
    return gyro_x, gyro_y, gyro_z


def calculate_allan_variance(data, rate, axis_name):
    """
    Calculate Allan variance using oadev function
    
    Parameters:
    - data: gyro data array (rad/s)
    - rate: sampling rate (Hz)
    - axis_name: string for labeling ('X', 'Y', or 'Z')
    """
    print(f"\nCalculating Allan variance for {axis_name} axis...")
    
    # CRITICAL: Remove the mean bias first!
    data_debiased = data - np.mean(data)
    
    # Convert rad/s to deg/s for easier interpretation
    data_deg = np.rad2deg(data_debiased)
    
    # Check if data has variation
    if np.std(data_deg) < 1e-10:
        print(f"WARNING: {axis_name} axis has almost no variation!")
        print(f"Standard deviation: {np.std(data_deg):.2e} deg/s")
        # Return dummy values to avoid errors
        return np.array([1.0]), np.array([1e-10]), np.array([0.0])
    
    print(f"Data stats after debiasing - Std: {np.std(data_deg):.6f} deg/s")
    
    # Calculate overlapping Allan deviation
    # taus='all' uses all possible tau values
    # data_type='freq' for rate data (gyroscope)
    tau, adev, adev_err, adev_n = allantools.oadev(
        data_deg, 
        rate=rate, 
        data_type='freq',
        taus='all'
    )
    
    return tau, adev, adev_err


def find_noise_parameters(tau, adev, axis_name):
    """
    Extract noise parameters from Allan deviation data
    
    Returns:
    - N: Angle Random Walk (deg/sqrt(hr))
    - B: Bias Instability (deg/hr)
    - K: Rate Random Walk (deg/sqrt(hr))
    """
    print(f"\n=== Noise Parameters for {axis_name} axis ===")
    
    # Check if we have dummy data
    if len(tau) == 1:
        print("Skipping analysis - insufficient data variation")
        return 0, 0, 0, 0, 0, 0
    
    # 1. Angle Random Walk (N) at tau = 1 second
    # Find the closest tau to 1 second
    idx_1s = np.argmin(np.abs(tau - 1.0))
    N_deg_sqrt_s = adev[idx_1s]
    N_deg_sqrt_hr = N_deg_sqrt_s * np.sqrt(3600)  # Convert to deg/sqrt(hr)
    
    print(f"Angle Random Walk (N) at tau=1s: {N_deg_sqrt_s:.6f} deg/√s")
    print(f"Angle Random Walk (N): {N_deg_sqrt_hr:.6f} deg/√hr")
    
    # 2. Bias Instability (B) - minimum of Allan deviation
    idx_min = np.argmin(adev)
    B = adev[idx_min]
    tau_B = tau[idx_min]
    
    print(f"Bias Instability (B): {B:.6f} deg/hr at tau={tau_B:.2f}s")
    
    # 3. Rate Random Walk (K) - slope after minimum
    # Find tau = 3 seconds (or closest point after minimum)
    idx_3s = np.argmin(np.abs(tau - 3.0))
    if idx_3s <= idx_min:
        idx_3s = min(idx_min + 5, len(tau) - 1)  # Use point after minimum
    
    K_deg_sqrt_hr = adev[idx_3s] * np.sqrt(tau[idx_3s] / 3600)
    
    print(f"Rate Random Walk (K): {K_deg_sqrt_hr:.6f} deg/√hr at tau={tau[idx_3s]:.2f}s")
    
    return N_deg_sqrt_hr, B, K_deg_sqrt_hr, idx_1s, idx_min, idx_3s


def plot_allan_deviation(tau, adev, axis_name, idx_1s, idx_min, idx_3s):
    """
    Plot Allan deviation with marked noise parameters
    """
    # Skip plotting if dummy data
    if len(tau) == 1:
        print(f"Skipping plot for {axis_name} - insufficient data")
        return
        
    plt.figure(figsize=(10, 6))
    plt.loglog(tau, adev, 'b-', linewidth=2, label='Allan Deviation')
    
    # Mark the three characteristic points
    plt.loglog(tau[idx_1s], adev[idx_1s], 'ro', markersize=10, label=f'N (τ=1s)')
    plt.loglog(tau[idx_min], adev[idx_min], 'go', markersize=10, label=f'B (min)')
    plt.loglog(tau[idx_3s], adev[idx_3s], 'mo', markersize=10, label=f'K (τ=3s)')
    
    plt.xlabel('Averaging Time τ (s)', fontsize=12)
    plt.ylabel('Allan Deviation (deg/s or deg/hr)', fontsize=12)
    plt.title(f'Allan Deviation Plot - Gyro {axis_name} Axis', fontsize=14)
    plt.grid(True, which='both', alpha=0.3)
    plt.legend(fontsize=10)
    plt.tight_layout()
    
    plt.savefig(f'allan_deviation_gyro_{axis_name.lower()}.png', dpi=300)
    print(f"Plot saved as: allan_deviation_gyro_{axis_name.lower()}.png")


def main():
    # Configuration
    bag_path = '/home/xingyue/gnss2/data/00.db3'
    imu_rate = 40  # Hz - your VectorNav output rate
    
    print("=" * 60)
    print("Allan Variance Analysis for IMU Data")
    print("=" * 60)
    
    # Read bag file ONCE
    try:
        gyro_x, gyro_y, gyro_z = read_imu_bag(bag_path)
    except Exception as e:
        print(f"Error reading bag file: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # Process each axis
    axes_data = [
        (gyro_x, 'X'),
        (gyro_y, 'Y'),
        (gyro_z, 'Z')
    ]
    
    results = {}
    
    for data, axis_name in axes_data:
        # Calculate Allan variance
        tau, adev, adev_err = calculate_allan_variance(data, imu_rate, axis_name)
        
        # Find noise parameters
        N, B, K, idx_1s, idx_min, idx_3s = find_noise_parameters(tau, adev, axis_name)
        
        # Store results
        results[axis_name] = {
            'N': N,
            'B': B,
            'K': K,
            'tau': tau,
            'adev': adev
        }
       
        # Plot
        plot_allan_deviation(tau, adev, axis_name, idx_1s, idx_min, idx_3s)
    
    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY OF NOISE PARAMETERS")
    print("=" * 60)
    for axis in ['X', 'Y', 'Z']:
        print(f"\nGyro {axis} axis:")
        print(f"  N (Angle Random Walk): {results[axis]['N']:.6f} deg/√hr")
        print(f"  B (Bias Instability):  {results[axis]['B']:.6f} deg/hr")
        print(f"  K (Rate Random Walk):  {results[axis]['K']:.6f} deg/√hr")
    
    plt.show()


if __name__ == '__main__':
    main()