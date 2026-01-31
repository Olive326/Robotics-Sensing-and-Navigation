#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from custom_interfaces.msg import Vectornav
from sensor_msgs.msg import NavSatFix
from scipy import signal
from scipy.signal import butter, filtfilt
import sys
import json
import os

class NavigationAnalyzer:
    def __init__(self, bag_path, calibration_file='mag_calibration.json'):
        self.bag_path = bag_path
        
        # Data storage
        self.timestamps = []
        self.mag_x, self.mag_y, self.mag_z = [], [], []
        self.gyro_z = []
        self.accel_x, self.accel_y = [], []
        self.imu_yaw = []
        
        self.gps_time, self.gps_lat, self.gps_lon = [], [], []
        
        # Load calibration from file
        self.load_calibration(calibration_file)
    
    def load_calibration(self, calibration_file):
        """Load saved magnetometer calibration"""
        if os.path.exists(calibration_file):
            print(f"\n{'='*60}")
            print("LOADING MAGNETOMETER CALIBRATION")
            print(f"{'='*60}")
            
            with open(calibration_file, 'r') as f:
                calib = json.load(f)
            
            self.mag_offset = np.array(calib['hard_iron_offset'])
            self.mag_scale = np.diag(calib['soft_iron_scale'])
            
            print(f"✓ Hard Iron Offset: {self.mag_offset}")
            print(f"✓ Soft Iron Scale: {np.diag(self.mag_scale)}")
            print(f"{'='*60}\n")
        else:
            print(f"\n⚠ ERROR: {calibration_file} not found!")
            print("  Please run analyze_calibration.py first\n")
            sys.exit(1)
    
    def read_bag(self):
        """Read driving bag data"""
        print(f"Reading bag: {self.bag_path}")
        
        storage_options = StorageOptions(uri=self.bag_path, storage_id='mcap')
        converter_options = ConverterOptions('', '')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            t = timestamp * 1e-9
            
            if topic == '/imu':
                try:
                    msg = deserialize_message(data, Vectornav)
                    
                    self.timestamps.append(t)
                    self.mag_x.append(msg.mag_field.magnetic_field.x)
                    self.mag_y.append(msg.mag_field.magnetic_field.y)
                    self.mag_z.append(msg.mag_field.magnetic_field.z)
                    self.gyro_z.append(msg.imu.angular_velocity.z)
                    self.accel_x.append(msg.imu.linear_acceleration.x)
                    self.accel_y.append(msg.imu.linear_acceleration.y)
                    self.imu_yaw.append(msg.imu_yaw)
                except:
                    pass
                    
            elif topic in ['/gps', '/fix', '/gps/fix']:
                try:
                    msg = deserialize_message(data, NavSatFix)
                    if msg.status.status >= 0:
                        self.gps_time.append(t)
                        self.gps_lat.append(msg.latitude)
                        self.gps_lon.append(msg.longitude)
                except:
                    pass
        
        # Convert to numpy
        self.timestamps = np.array(self.timestamps)
        self.mag_x = np.array(self.mag_x)
        self.mag_y = np.array(self.mag_y)
        self.mag_z = np.array(self.mag_z)
        self.gyro_z = np.array(self.gyro_z)
        self.accel_x = np.array(self.accel_x)
        self.accel_y = np.array(self.accel_y)
        self.imu_yaw = np.array(self.imu_yaw)
        
        if len(self.gps_time) > 0:
            self.gps_time = np.array(self.gps_time)
            self.gps_lat = np.array(self.gps_lat)
            self.gps_lon = np.array(self.gps_lon)
        
        # Relative time
        if len(self.timestamps) > 0:
            t0 = self.timestamps[0]
            self.timestamps -= t0
            if len(self.gps_time) > 0:
                self.gps_time -= t0
        
        print(f"✓ Loaded {len(self.timestamps)} IMU samples")
        print(f"✓ Loaded {len(self.gps_time)} GPS samples")
        print(f"✓ Duration: {self.timestamps[-1]:.2f} seconds\n")
    
    def apply_mag_calibration(self, mx, my, mz):
        """Apply loaded calibration"""
        mag_vec = np.array([mx, my, mz])
        corrected = self.mag_scale @ (mag_vec - self.mag_offset)
        return corrected
    
    def compute_mag_yaw(self, mx, my):
        """Compute yaw from magnetometer"""
        return np.arctan2(my, mx)
    
    def integrate_gyro_yaw(self):
        """Integrate gyro_z to get yaw"""
        dt = np.diff(self.timestamps)
        dt = np.append(dt, dt[-1])
        yaw = np.cumsum(self.gyro_z * dt)
        return yaw
    
    def low_pass_filter(self, data, cutoff_hz=1.0, fs=100.0):
        """Low pass filter - removes high frequency noise"""
        nyquist = fs / 2
        normal_cutoff = np.clip(cutoff_hz / nyquist, 0.001, 0.999)
        b, a = signal.butter(4, normal_cutoff, btype='low')  # ← Using SciPy!
        return signal.filtfilt(b, a, data)

    def high_pass_filter(self, data, cutoff_hz=0.1, fs=100.0):
        """High pass filter - removes low frequency drift"""
        nyquist = fs / 2
        normal_cutoff = np.clip(cutoff_hz / nyquist, 0.001, 0.999)
        b, a = signal.butter(4, normal_cutoff, btype='high')  # ← Using SciPy!
        return signal.filtfilt(b, a, data)

    def complementary_filter(self, mag_yaw, gyro_yaw, alpha=0.98):
        """Fuse magnetometer and gyroscope yaw"""
        fused = np.zeros_like(mag_yaw)
        fused[0] = mag_yaw[0]
    
        for i in range(1, len(mag_yaw)):
           dt = self.timestamps[i] - self.timestamps[i-1]
           gyro_change = self.gyro_z[i] * dt
           fused[i] = alpha * (fused[i-1] + gyro_change) + (1 - alpha) * mag_yaw[i]
#         ↑ 98% trust gyro                      ↑ 2% trust mag
    
        return fused
    
    def compute_velocity_from_accel(self):
        """Integrate acceleration to velocity"""
        if len(self.accel_x) > 100:
            accel_forward = self.accel_x - np.mean(self.accel_x[:100])
        else:
            accel_forward = self.accel_x - np.mean(self.accel_x)
        
        dt = np.diff(self.timestamps)
        dt = np.append(dt, dt[-1])
        velocity = np.cumsum(accel_forward * dt)
        return velocity, accel_forward
    
    def latlon_to_meters(self, lat, lon):
        """Convert lat/lon to meters"""
        lat0, lon0 = lat[0], lon[0]
        lat_m = (lat - lat0) * 111320
        lon_m = (lon - lon0) * 111320 * np.cos(np.radians(lat0))
        return lat_m, lon_m
    
    def compute_velocity_from_gps(self):
        """Compute velocity from GPS"""
        if len(self.gps_lat) < 2:
            return np.array([]), np.array([]), np.array([])
        
        lat_m, lon_m = self.latlon_to_meters(self.gps_lat, self.gps_lon)
        distances = np.sqrt(np.diff(lat_m)**2 + np.diff(lon_m)**2)
        dt = np.diff(self.gps_time)
        dt[dt == 0] = 1e-6
        velocity = distances / dt
        velocity = np.append(0, velocity)
        return velocity, lat_m, lon_m
    
    def compute_trajectory_from_imu(self, velocity, yaw):
        """Dead reckoning trajectory"""
        dt = np.diff(self.timestamps)
        dt = np.append(dt, dt[-1])
        
        x, y = np.zeros(len(velocity)), np.zeros(len(velocity))
        for i in range(1, len(velocity)):
            x[i] = x[i-1] + velocity[i] * np.cos(yaw[i]) * dt[i]
            y[i] = y[i-1] + velocity[i] * np.sin(yaw[i]) * dt[i]
        
        return x, y
    def analyze_position_accuracy(self, x_imu, y_imu, lat_m, lon_m):
        """Analyze when IMU and GPS trajectories diverge (Q6 & Q7)"""
        from scipy.interpolate import interp1d
        
        print("\n" + "="*60)
        print("POSITION ACCURACY ANALYSIS (Q6 & Q7)")
        print("="*60)
        
        # Interpolate IMU trajectory to GPS timestamps
        f_x = interp1d(self.timestamps, x_imu, bounds_error=False, fill_value='extrapolate')
        f_y = interp1d(self.timestamps, y_imu, bounds_error=False, fill_value='extrapolate')
        
        x_imu_interp = f_x(self.gps_time)
        y_imu_interp = f_y(self.gps_time)
        
        # Calculate position error at each GPS time
        error = np.sqrt((x_imu_interp - lon_m)**2 + (y_imu_interp - lat_m)**2)
        
        # Find when error exceeds 2m threshold
        good_idx = np.where(error < 2.0)[0]
        
        if len(good_idx) > 0:
            good_time = self.gps_time[good_idx[-1]]
            print(f"\n✓ Position accurate (< 2m error) for: {good_time:.1f} seconds")
        else:
            print(f"\n✗ Position error exceeded 2m immediately")
            good_time = 0
        
        # Calculate final position error
        final_error = error[-1] if len(error) > 0 else 0
        print(f"✓ Final position error: {final_error:.1f} meters")
        
        # Plot position error over time
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(self.gps_time, error, linewidth=2, color='#e74c3c')
        ax.axhline(2.0, color='g', linestyle='--', linewidth=2, label='2m threshold')
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Position Error (m)', fontsize=11)
        ax.set_title('Q7: IMU vs GPS Position Error Over Time', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig('fig_position_error.png', dpi=300, bbox_inches='tight')
        print(f"✓ Saved: fig_position_error.png")
        plt.close()
        
        print("="*60 + "\n")
        
        return good_time, final_error

    def analyze_motion_model(self):
        """Compare predicted vs observed lateral acceleration (Q5)"""

        # Get velocity and yaw rate
        velocity_accel, _ = self.compute_velocity_from_accel()
        yaw_rate = self.gyro_z  # ω (angular velocity)

        # Predicted lateral acceleration from motion model
        # a_lateral_predicted = ω × v
        a_lateral_predicted = yaw_rate * velocity_accel

        # Observed lateral acceleration
        a_lateral_observed = self.accel_y

        # Plot comparison
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(self.timestamps, a_lateral_observed, label='Observed (accel_y)', alpha=0.7, linewidth=1.5)
        ax.plot(self.timestamps, a_lateral_predicted, label='Predicted (ω × v)', alpha=0.7, linewidth=1.5)
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Lateral Acceleration (m/s²)', fontsize=11)
        ax.set_title('Q5: Motion Model - Predicted vs Observed Lateral Acceleration', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig('fig_motion_model_comparison.png', dpi=300, bbox_inches='tight')
        print(f"✓ Saved: fig_motion_model_comparison.png")
        plt.close()

        # Calculate correlation
        correlation = np.corrcoef(a_lateral_observed, a_lateral_predicted)[0, 1]
        print(f"✓ Correlation coefficient: {correlation:.3f}")
        
        # Calculate RMSE
        rmse = np.sqrt(np.mean((a_lateral_observed - a_lateral_predicted)**2))
        print(f"✓ RMSE: {rmse:.3f} m/s²")
    
    def plot_navigation(self):
        """Generate Fig 2-6 and answer Q5, Q6, Q7"""
        # Apply calibration
        mag_corrected = np.array([self.apply_mag_calibration(mx, my, mz) 
                                for mx, my, mz in zip(self.mag_x, self.mag_y, self.mag_z)])
        mag_yaw = self.compute_mag_yaw(mag_corrected[:, 0], mag_corrected[:, 1])
        gyro_yaw = self.integrate_gyro_yaw()
        
        # ====== Q5: Motion Model Analysis ======
        print("\n" + "="*60)
        print("Q5: MOTION MODEL ANALYSIS")
        print("="*60)
        self.analyze_motion_model()  
        print("="*60 + "\n")
        
        # Figure 2: Gyro yaw
        print("[1/5] Generating Figure 2: Gyro yaw...")
        fig2, ax2 = plt.subplots(figsize=(12, 6))
        ax2.plot(self.timestamps, np.degrees(gyro_yaw), linewidth=1.5, color='#2ca02c')
        ax2.set_xlabel('Time (s)', fontsize=11)
        ax2.set_ylabel('Yaw (degrees)', fontsize=11)
        ax2.set_title('Figure 2: Gyroscope Yaw Estimation', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig('fig2_gyro_yaw.png', dpi=300, bbox_inches='tight')
        print("  ✓ Saved: fig2_gyro_yaw.png")
        plt.close()
        
        # Figure 3: Complementary filter
        print("[2/5] Generating Figure 3: Complementary filter...")
        fs = 1.0 / np.mean(np.diff(self.timestamps))
        mag_yaw_lpf = self.low_pass_filter(mag_yaw, cutoff_hz=1.0, fs=fs)
        gyro_yaw_hpf = self.high_pass_filter(gyro_yaw, cutoff_hz=0.1, fs=fs)
        complementary = self.complementary_filter(mag_yaw, gyro_yaw, alpha=0.98)
        
        fig3, axes = plt.subplots(4, 1, figsize=(12, 10))
        fig3.suptitle('Figure 3: Complementary Filter Components', fontsize=14, fontweight='bold')
        
        axes[0].plot(self.timestamps, np.degrees(mag_yaw_lpf), linewidth=1.5, color='#1f77b4')
        axes[0].set_ylabel('Mag LPF (deg)', fontsize=11)
        axes[0].set_title('Low Pass Filtered Magnetometer', fontsize=11)
        axes[0].grid(True, alpha=0.3)
        
        axes[1].plot(self.timestamps, np.degrees(gyro_yaw_hpf), linewidth=1.5, color='#ff7f0e')
        axes[1].set_ylabel('Gyro HPF (deg)', fontsize=11)
        axes[1].set_title('High Pass Filtered Gyroscope', fontsize=11)
        axes[1].grid(True, alpha=0.3)
        
        axes[2].plot(self.timestamps, np.degrees(complementary), linewidth=1.5, color='#2ca02c')
        axes[2].set_ylabel('Complementary (deg)', fontsize=11)
        axes[2].set_title('Complementary Filter Output', fontsize=11)
        axes[2].grid(True, alpha=0.3)
        
        axes[3].plot(self.timestamps, np.degrees(mag_yaw), label='Magnetometer', alpha=0.5, linewidth=1.5)
        axes[3].plot(self.timestamps, np.degrees(complementary), label='Fused', linewidth=2)
        axes[3].set_ylabel('Heading (deg)', fontsize=11)
        axes[3].set_xlabel('Time (s)', fontsize=11)
        axes[3].set_title('IMU Heading Estimate', fontsize=11)
        axes[3].legend(fontsize=10)
        axes[3].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('fig3_complementary_filter.png', dpi=300, bbox_inches='tight')
        print("  ✓ Saved: fig3_complementary_filter.png")
        plt.close()
        
        # Figure 4: Accel velocity
        print("[3/5] Generating Figure 4: Accelerometer velocity...")
        velocity_accel, accel_corrected = self.compute_velocity_from_accel()
        
        fig4, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig4.suptitle('Figure 4: Forward Velocity from Accelerometer', fontsize=14, fontweight='bold')
        
        axes[0].plot(self.timestamps, self.accel_x, label='Raw', alpha=0.7, linewidth=1.5)
        axes[0].plot(self.timestamps, accel_corrected, label='Corrected', alpha=0.7, linewidth=1.5)
        axes[0].set_ylabel('Acceleration (m/s²)', fontsize=11)
        axes[0].legend(fontsize=10)
        axes[0].grid(True, alpha=0.3)
        
        axes[1].plot(self.timestamps, velocity_accel, linewidth=1.5, color='#d62728')
        axes[1].set_ylabel('Velocity (m/s)', fontsize=11)
        axes[1].set_xlabel('Time (s)', fontsize=11)
        axes[1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('fig4_accel_velocity.png', dpi=300, bbox_inches='tight')
        print("  ✓ Saved: fig4_accel_velocity.png")
        plt.close()
        
        # Figure 5: GPS velocity
        if len(self.gps_time) > 1:
            print("[4/5] Generating Figure 5: GPS velocity...")
            velocity_gps, lat_m, lon_m = self.compute_velocity_from_gps()
            
            fig5, ax = plt.subplots(figsize=(12, 6))
            ax.plot(self.gps_time, velocity_gps, linewidth=1.5, color='#9467bd')
            ax.set_xlabel('Time (s)', fontsize=11)
            ax.set_ylabel('Velocity (m/s)', fontsize=11)
            ax.set_title('Figure 5: Forward Velocity from GPS', fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.savefig('fig5_gps_velocity.png', dpi=300, bbox_inches='tight')
            print("  ✓ Saved: fig5_gps_velocity.png")
            plt.close()
        else:
            print("[4/5] Skipping Figure 5: No GPS data")
            lat_m, lon_m = None, None
        
        # Figure 6: Trajectories
        print("[5/5] Generating Figure 6: Trajectories...")
        x_imu, y_imu = self.compute_trajectory_from_imu(velocity_accel, complementary)
        
        if len(self.gps_time) > 1 and lat_m is not None:
            fig6, axes = plt.subplots(1, 2, figsize=(16, 7))
            fig6.suptitle('Figure 6: Estimated Trajectories', fontsize=14, fontweight='bold')
            
            # GPS
            axes[0].plot(lon_m, lat_m, 'b-', linewidth=2)
            axes[0].plot(lon_m[0], lat_m[0], 'go', markersize=12, label='Start')
            axes[0].plot(lon_m[-1], lat_m[-1], 'ro', markersize=12, label='End')
            axes[0].set_xlabel('East (m)', fontsize=11)
            axes[0].set_ylabel('North (m)', fontsize=11)
            axes[0].set_title('GPS Trajectory', fontsize=12, fontweight='bold')
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)
            axes[0].axis('equal')
            
            # IMU
            axes[1].plot(x_imu, y_imu, 'r-', linewidth=2)
            axes[1].plot(x_imu[0], y_imu[0], 'go', markersize=12, label='Start')
            axes[1].plot(x_imu[-1], y_imu[-1], 'ro', markersize=12, label='End')
            axes[1].set_xlabel('X (m)', fontsize=11)
            axes[1].set_ylabel('Y (m)', fontsize=11)
            axes[1].set_title('IMU Trajectory', fontsize=12, fontweight='bold')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)
            axes[1].axis('equal')
            
            plt.tight_layout()
            plt.savefig('fig6_trajectories.png', dpi=300, bbox_inches='tight')
            print("  ✓ Saved: fig6_trajectories.png")
            plt.close()
            
            # ====== Q6 & Q7: Position Accuracy Analysis ======
            good_time, final_error = self.analyze_position_accuracy(x_imu, y_imu, lat_m, lon_m)
            
        else:
            fig6, ax = plt.subplots(figsize=(10, 10))
            ax.plot(x_imu, y_imu, 'b-', linewidth=2)
            ax.plot(x_imu[0], y_imu[0], 'go', markersize=15, label='Start')
            ax.plot(x_imu[-1], y_imu[-1], 'ro', markersize=15, label='End')
            ax.set_xlabel('X (m)', fontsize=11)
            ax.set_ylabel('Y (m)', fontsize=11)
            ax.set_title('Figure 6: IMU Trajectory', fontsize=14, fontweight='bold')
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
            
            plt.tight_layout()
            plt.savefig('fig6_trajectories.png', dpi=300, bbox_inches='tight')
            print("  ✓ Saved: fig6_trajectories.png")
            plt.close()
        
        print("\n✅ All navigation plots generated!")
def main():
    if len(sys.argv) < 2:
        print("\nUsage: python3 analyze_navigation.py <driving_bag_path> [calibration_file]")
        print("\nExample:")
        print("  python3 analyze_navigation.py ~/gnss/bags/data_driving")
        print("  python3 analyze_navigation.py data_driving mag_calibration.json\n")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    if bag_path.endswith('.mcap'):
        bag_path = bag_path[:-5]
    
    calib_file = sys.argv[2] if len(sys.argv) > 2 else 'mag_calibration.json'
    
    analyzer = NavigationAnalyzer(bag_path, calib_file)
    analyzer.read_bag()
    analyzer.plot_navigation()

if __name__ == '__main__':
    main()
