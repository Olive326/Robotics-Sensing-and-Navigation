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
    timestamps = []
    
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
            timestamps.append(timestamp / 1e9)  # Convert to seconds
            
        except Exception as e:
            # Error handling for corrupted messages
            print(f"Warning: Skipping corrupted message - {e}")
            continue
    
    # Convert to numpy arrays
    gyro_x = np.array(gyro_x)
    gyro_y = np.array(gyro_y)
    gyro_z = np.array(gyro_z)
    timestamps = np.array(timestamps)
    
    # Data quality checks
    duration = (timestamps[-1] - timestamps[0]) / 3600  # hours
    print(f"\n{'='*60}")
    print(f"Successfully read {len(gyro_x)} IMU messages")
    print(f"Data duration: {duration:.2f} hours")
    
    if duration < 1.0:
        print(f"WARNING: Data duration ({duration:.2f}h) is less than recommended 1-2 hours!")
        print("Allan Variance analysis may not show clear flat region for Bias Instability")
    
    print(f"\n{'='*60}")
    print("=== Gyro Data Statistics (rad/s) ===")
    print(f"{'='*60}")
    print(f"X axis - Mean: {np.mean(gyro_x):.8f}, Std: {np.std(gyro_x):.8f}")
    print(f"         Range: [{np.min(gyro_x):.8f}, {np.max(gyro_x):.8f}]")
    print(f"Y axis - Mean: {np.mean(gyro_y):.8f}, Std: {np.std(gyro_y):.8f}")
    print(f"         Range: [{np.min(gyro_y):.8f}, {np.max(gyro_y):.8f}]")
    print(f"Z axis - Mean: {np.mean(gyro_z):.8f}, Std: {np.std(gyro_z):.8f}")
    print(f"         Range: [{np.min(gyro_z):.8f}, {np.max(gyro_z):.8f}]")
    
    # Check for motion (stationary data should have small variations)
    max_variation_x = np.max(np.abs(gyro_x - np.mean(gyro_x)))
    max_variation_y = np.max(np.abs(gyro_y - np.mean(gyro_y)))
    max_variation_z = np.max(np.abs(gyro_z - np.mean(gyro_z)))
    
    print(f"\n=== Motion Check ===")
    print(f"Max deviation from mean (should be < 0.01 rad/s for stationary):")
    print(f"  X: {max_variation_x:.6f} rad/s")
    print(f"  Y: {max_variation_y:.6f} rad/s")
    print(f"  Z: {max_variation_z:.6f} rad/s")
    
    if max_variation_x > 0.05 or max_variation_y > 0.05 or max_variation_z > 0.05:
        print("WARNING: Large variations detected - IMU may not have been stationary!")
    
    return gyro_x, gyro_y, gyro_z, timestamps


def plot_raw_data(timestamps, gyro_x, gyro_y, gyro_z):
    """
    Plot raw gyro data over time to check for motion/trends
    """
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    time_hours = (timestamps - timestamps[0]) / 3600
    
    axes[0].plot(time_hours, gyro_x, 'b-', linewidth=0.5)
    axes[0].set_ylabel('Gyro X (rad/s)', fontsize=11)
    axes[0].set_title('Raw Gyroscope Data - Check for Stationarity', fontsize=14)
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=np.mean(gyro_x), color='r', linestyle='--', label=f'Mean: {np.mean(gyro_x):.6f}')
    axes[0].legend()
    
    axes[1].plot(time_hours, gyro_y, 'g-', linewidth=0.5)
    axes[1].set_ylabel('Gyro Y (rad/s)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=np.mean(gyro_y), color='r', linestyle='--', label=f'Mean: {np.mean(gyro_y):.6f}')
    axes[1].legend()
    
    axes[2].plot(time_hours, gyro_z, 'orange', linewidth=0.5)
    axes[2].set_ylabel('Gyro Z (rad/s)', fontsize=11)
    axes[2].set_xlabel('Time (hours)', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    axes[2].axhline(y=np.mean(gyro_z), color='r', linestyle='--', label=f'Mean: {np.mean(gyro_z):.6f}')
    axes[2].legend()
    
    plt.tight_layout()
    plt.savefig('raw_gyro_time_series.png', dpi=300)
    print("Raw data plot saved: raw_gyro_time_series.png")


def calculate_allan_variance(data, rate, axis_name):
    """
    Calculate Allan variance using oadev function
    
    Parameters:
    - data: gyro data array (rad/s)
    - rate: sampling rate (Hz)
    - axis_name: string for labeling ('X', 'Y', or 'Z')
    """
    print(f"\n{'='*60}")
    print(f"Calculating Allan variance for {axis_name} axis...")
    print(f"{'='*60}")
    
    # CRITICAL: Remove the mean bias first!
    data_debiased = data - np.mean(data)
    
    # Convert rad/s to deg/s for easier interpretation
    data_deg = np.rad2deg(data_debiased)
    
    # Check if data has variation
    if np.std(data_deg) < 1e-10:
        print(f"WARNING: {axis_name} axis has almost no variation!")
        print(f"Standard deviation: {np.std(data_deg):.2e} deg/s")
        return np.array([1.0]), np.array([1e-10]), np.array([0.0])
    
    print(f"Data points: {len(data_deg)}")
    print(f"Std deviation after debiasing: {np.std(data_deg):.6f} deg/s")
    
    # Calculate overlapping Allan deviation
    try:
        tau, adev, adev_err, adev_n = allantools.oadev(
            data_deg, 
            rate=rate, 
            data_type='freq',
            taus='octave'  # Use octave spacing for cleaner plots
        )
    except Exception as e:
        print(f"Error calculating Allan variance: {e}")
        return np.array([1.0]), np.array([1e-10]), np.array([0.0])
    
    print(f"Generated {len(tau)} tau points")
    print(f"Tau range: {tau[0]:.3f}s to {tau[-1]:.1f}s")
    
    return tau, adev, adev_err


def find_noise_parameters(tau, adev, axis_name):
    """
    Extract noise parameters using IEEE standard methodology with corrections
    
    Returns:
    - N: Angle Random Walk (deg/sqrt(hr))
    - B: Bias Instability (deg/hr) - with 0.664 correction
    - K: Rate Random Walk (deg/sqrt(hr))
    """
    print(f"\n{'='*60}")
    print(f"Extracting Noise Parameters - {axis_name} axis")
    print(f"{'='*60}")
    
    # Check if we have enough data
    if len(tau) < 10:
        print("Insufficient data points for analysis")
        return 0, 0, 0, 0, 0, 0
    
    # === 1. Angle Random Walk (N) at tau = 1 second ===
    idx_1s = np.argmin(np.abs(tau - 1.0))
    N_raw = adev[idx_1s]
    
    # Convert: Currently in deg/s (from rate data)
    # Need: deg/√hr
    N_deg_sqrt_hr = N_raw * np.sqrt(3600)
    
    print(f"\n1. Angle Random Walk (N):")
    print(f"   Read at τ={tau[idx_1s]:.2f}s")
    print(f"   Raw value: {N_raw:.8f} deg/s")
    print(f"   N = {N_deg_sqrt_hr:.6f} deg/√hr")
    
    # === 2. Bias Instability (B) at minimum with correction ===
    # Search for minimum in expected range [10s, 1000s]
    mask_bi = (tau >= 10) & (tau <= 1000)
    
    if np.any(mask_bi):
        tau_bi_region = tau[mask_bi]
        adev_bi_region = adev[mask_bi]
        idx_bi_local = np.argmin(adev_bi_region)
        idx_bi = np.where(mask_bi)[0][idx_bi_local]
        tau_B = tau[idx_bi]
    else:
        # Fallback to global minimum if no points in expected range
        idx_bi = np.argmin(adev)
        tau_B = tau[idx_bi]
        print(f"   WARNING: No minimum found in expected range [10s, 1000s]")
        print(f"   Using global minimum at τ={tau_B:.2f}s")
    
    B_raw = adev[idx_bi]
    B = B_raw / 0.664  # Apply IEEE correction factor for flicker noise
    
    print(f"\n2. Bias Instability (B):")
    print(f"   Minimum at τ={tau_B:.2f}s")
    print(f"   Raw minimum: {B_raw:.8f} deg/hr")
    print(f"   B (corrected) = {B:.6f} deg/hr")
    print(f"   Correction factor: 1/0.664 = 1.506")
    
    # === 3. Rate Random Walk (K) at tau = 3 seconds ===
    # Must be AFTER the minimum (on +1/2 slope region)
    
    # First, try to find τ=3s
    idx_3s = np.argmin(np.abs(tau - 3.0))
    
    # If τ=3s is before minimum, find a point after minimum
    if tau[idx_3s] < tau_B:
        # Find first point after minimum where tau >= 3s
        mask_after_min = tau >= max(3.0, tau_B * 1.2)
        if np.any(mask_after_min):
            idx_3s = np.where(mask_after_min)[0][0]
        else:
            # Use a point well after minimum
            idx_3s = min(idx_bi + 10, len(tau) - 1)
        print(f"   Note: τ=3s is before minimum, using τ={tau[idx_3s]:.2f}s instead")
    
    K_raw = adev[idx_3s]
    tau_K = tau[idx_3s]
    
    # Convert to deg/√hr with proper scaling
    # Formula: K = σ(τ_K) × √(τ_K / 3) for reading at tau_K
    # Then convert from deg/s to deg/√hr
    K_scaling = np.sqrt(tau_K / 3.0)  # Normalize to τ=3s reference
    K_deg_sqrt_hr = K_raw * K_scaling * np.sqrt(3600)
    
    print(f"\n3. Rate Random Walk (K):")
    print(f"   Read at τ={tau_K:.2f}s")
    print(f"   Raw value: {K_raw:.8f}")
    print(f"   Scaling factor: √(τ/{3.0}) = {K_scaling:.4f}")
    print(f"   K = {K_deg_sqrt_hr:.6f} deg/√hr")
    
    # Check if we're actually on a +1/2 slope
    if idx_3s < len(tau) - 3:
        # Estimate slope between this point and next few points
        log_tau_diff = np.log10(tau[idx_3s+2]) - np.log10(tau[idx_3s])
        log_adev_diff = np.log10(adev[idx_3s+2]) - np.log10(adev[idx_3s])
        slope = log_adev_diff / log_tau_diff
        print(f"   Estimated slope at this region: {slope:.3f} (should be ≈+0.5 for RRW)")
        
        if slope < 0:
            print(f"   WARNING: Negative slope detected! May not be in RRW region yet.")
    
    return N_deg_sqrt_hr, B, K_deg_sqrt_hr, idx_1s, idx_bi, idx_3s


def plot_allan_deviation(tau, adev, axis_name, idx_1s, idx_bi, idx_3s, N, B, K):
    """
    Plot Allan deviation with marked noise parameters and reference slopes
    """
    # Skip plotting if insufficient data
    if len(tau) < 3:
        print(f"Skipping plot for {axis_name} - insufficient data")
        return
        
    plt.figure(figsize=(12, 8))
    
    # Main Allan Deviation curve
    plt.loglog(tau, adev, 'b-', linewidth=2.5, label='Allan Deviation (measured)', zorder=3)
    
    # Mark the three characteristic points
    plt.loglog(tau[idx_1s], adev[idx_1s], 'ro', markersize=14, 
               label=f'N @ τ={tau[idx_1s]:.2f}s: {N:.4f} deg/√hr', zorder=4)
    plt.loglog(tau[idx_bi], adev[idx_bi], 'go', markersize=14, 
               label=f'B @ τ={tau[idx_bi]:.2f}s: {B:.4f} deg/hr', zorder=4)
    plt.loglog(tau[idx_3s], adev[idx_3s], 'mo', markersize=14, 
               label=f'K @ τ={tau[idx_3s]:.2f}s: {K:.4f} deg/√hr', zorder=4)
    
    # Draw reference slope lines
    # White noise reference (slope -1/2)
    tau_ref_white = np.array([tau[0], tau[idx_1s] * 3])
    adev_ref_white = adev[idx_1s] * (tau_ref_white / tau[idx_1s])**(-0.5)
    plt.loglog(tau_ref_white, adev_ref_white, 'r--', alpha=0.6, linewidth=1.5, 
               label='Slope -1/2 (White Noise)', zorder=2)
    
    # Brown noise reference (slope +1/2) - only if we have rising region
    if idx_3s < len(tau) - 5 and tau[idx_3s] > tau[idx_bi]:
        tau_ref_brown = np.array([tau[idx_3s], tau[-1]])
        adev_ref_brown = adev[idx_3s] * (tau_ref_brown / tau[idx_3s])**(0.5)
        plt.loglog(tau_ref_brown, adev_ref_brown, 'm--', alpha=0.6, linewidth=1.5, 
                   label='Slope +1/2 (Brown Noise)', zorder=2)
    
    # Formatting
    plt.xlabel('Averaging Time τ (seconds)', fontsize=13, fontweight='bold')
    plt.ylabel('Allan Deviation (deg/s or deg/hr)', fontsize=13, fontweight='bold')
    plt.title(f'Allan Deviation Plot - Gyroscope {axis_name} Axis\nNoise Characterization', 
              fontsize=15, fontweight='bold')
    plt.grid(True, which='both', alpha=0.4, linestyle=':')
    plt.grid(True, which='major', alpha=0.6, linestyle='-')
    plt.legend(fontsize=10, loc='best', framealpha=0.9)
    plt.tight_layout()
    
    # Save with higher DPI
    plt.savefig(f'allan_deviation_gyro_{axis_name.lower()}.png', dpi=300, bbox_inches='tight')
    print(f"   Plot saved: allan_deviation_gyro_{axis_name.lower()}.png")


def main():
    # Configuration
    bag_path = '/home/xingyue/data/00.db3'
    imu_rate = 40  # Hz - your VectorNav output rate
    
    print("\n" + "=" * 70)
    print(" " * 15 + "ALLAN VARIANCE ANALYSIS FOR IMU DATA")
    print("=" * 70)
    
    # Read bag file
    try:
        gyro_x, gyro_y, gyro_z, timestamps = read_imu_bag(bag_path)
    except Exception as e:
        print(f"Error reading bag file: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # Plot raw data to verify stationarity
    plot_raw_data(timestamps, gyro_x, gyro_y, gyro_z)
    
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
        N, B, K, idx_1s, idx_bi, idx_3s = find_noise_parameters(tau, adev, axis_name)
        
        # Store results
        results[axis_name] = {
            'N': N,
            'B': B,
            'K': K,
            'tau': tau,
            'adev': adev,
            'tau_N': tau[idx_1s],
            'tau_B': tau[idx_bi],
            'tau_K': tau[idx_3s]
        }
       
        # Plot with annotations
        plot_allan_deviation(tau, adev, axis_name, idx_1s, idx_bi, idx_3s, N, B, K)
    
    # Final Summary Table
    print("\n" + "=" * 70)
    print(" " * 20 + "NOISE PARAMETERS SUMMARY")
    print("=" * 70)
    print(f"\n{'Axis':<6} {'N (deg/√hr)':<18} {'B (deg/hr)':<18} {'K (deg/√hr)':<18}")
    print("-" * 70)
    
    for axis in ['X', 'Y', 'Z']:
        print(f"{axis:<6} {results[axis]['N']:<18.6f} {results[axis]['B']:<18.6f} {results[axis]['K']:<18.6f}")
    
    print("\n" + "=" * 70)
    print("Reading Points:")
    print("=" * 70)
    for axis in ['X', 'Y', 'Z']:
        print(f"{axis} axis: N @ τ={results[axis]['tau_N']:.2f}s, " + 
              f"B @ τ={results[axis]['tau_B']:.2f}s, " +
              f"K @ τ={results[axis]['tau_K']:.2f}s")
    
    print("\n" + "=" * 70)
    print("Interpretation Guide:")
    print("=" * 70)
    print("N (Angle Random Walk):  White noise - short-term random fluctuations")
    print("B (Bias Instability):   Flicker noise - slow bias drift (corrected by /0.664)")
    print("K (Rate Random Walk):   Brown noise - long-term unbounded drift")
    print("=" * 70)
    
    # Show all plots
    plt.show()


if __name__ == '__main__':
    main()