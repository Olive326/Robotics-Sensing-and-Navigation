#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from custom_interfaces.msg import Vectornav
import sys
import json

class CalibrationAnalyzer:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.timestamps = []
        self.mag_x, self.mag_y, self.mag_z = [], [], []
        self.mag_offset = np.array([0.0, 0.0, 0.0])
        self.mag_scale = np.eye(3)
        
    def read_bag(self):
        """Read only magnetometer data from rosbag"""
        print(f"Reading bag: {self.bag_path}")
        
        storage_options = StorageOptions(uri=self.bag_path, storage_id='mcap')
        converter_options = ConverterOptions('', '')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            
            if topic == '/imu':
                try:
                    msg = deserialize_message(data, Vectornav)
                    t = timestamp * 1e-9
                    
                    self.timestamps.append(t)
                    self.mag_x.append(msg.mag_field.magnetic_field.x)
                    self.mag_y.append(msg.mag_field.magnetic_field.y)
                    self.mag_z.append(msg.mag_field.magnetic_field.z)
                except:
                    pass
        
        # Convert to numpy arrays
        self.timestamps = np.array(self.timestamps)
        self.mag_x = np.array(self.mag_x)
        self.mag_y = np.array(self.mag_y)
        self.mag_z = np.array(self.mag_z)
        self.timestamps -= self.timestamps[0]
        
        print(f"✓ Loaded {len(self.timestamps)} magnetometer samples")
        print(f"✓ Duration: {self.timestamps[-1]:.2f} seconds\n")
    
    def calibrate_magnetometer(self):
        """Compute hard and soft iron calibration"""
        print("="*60)
        print("MAGNETOMETER CALIBRATION")
        print("="*60)
        
        mag_data = np.column_stack([self.mag_x, self.mag_y, self.mag_z])
        
        # Hard iron: constant bias (center of ellipsoid)
        self.mag_offset = np.mean(mag_data, axis=0)
        
        # Soft iron: scale factors (ellipsoid to sphere)
        centered = mag_data - self.mag_offset
        scale_x = np.max(np.abs(centered[:, 0]))
        scale_y = np.max(np.abs(centered[:, 1]))
        scale_z = np.max(np.abs(centered[:, 2]))
        avg_scale = (scale_x + scale_y + scale_z) / 3
        
        self.mag_scale = np.diag([avg_scale/scale_x, avg_scale/scale_y, avg_scale/scale_z])
        
        print(f"\nHard Iron Offset (subtract from raw):")
        print(f"  X: {self.mag_offset[0]:+8.4f} μT")
        print(f"  Y: {self.mag_offset[1]:+8.4f} μT")
        print(f"  Z: {self.mag_offset[2]:+8.4f} μT")
        
        print(f"\nSoft Iron Scale (multiply after offset):")
        print(f"  X: {np.diag(self.mag_scale)[0]:8.4f}")
        print(f"  Y: {np.diag(self.mag_scale)[1]:8.4f}")
        print(f"  Z: {np.diag(self.mag_scale)[2]:8.4f}")
        print("="*60 + "\n")
    
    def save_calibration(self, filename='mag_calibration.json'):
        """Save calibration to JSON file"""
        calib_data = {
            'hard_iron_offset': self.mag_offset.tolist(),
            'soft_iron_scale': np.diag(self.mag_scale).tolist(),
            'description': 'Apply as: corrected = soft_iron_scale * (raw - hard_iron_offset)'
        }
        
        with open(filename, 'w') as f:
            json.dump(calib_data, f, indent=2)
        
        print(f"✓ Saved calibration to: {filename}\n")
    
    def apply_calibration(self, mx, my, mz):
        """Apply hard and soft iron correction"""
        mag_vec = np.array([mx, my, mz])
        corrected = self.mag_scale @ (mag_vec - self.mag_offset)
        return corrected
    
    def compute_yaw(self, mx, my):
        """Compute yaw angle from mag X and Y"""
        return np.arctan2(my, mx)
    
    def plot_results(self):
        """Generate Figure 0 and Figure 1"""
        
        # Apply calibration to all data
        mag_corrected = np.array([self.apply_calibration(mx, my, mz) 
                                  for mx, my, mz in zip(self.mag_x, self.mag_y, self.mag_z)])
        
        # ===== FIGURE 0: Magnetometer X, Y, Z =====
        print("Generating Figure 0: Magnetometer calibration...")
        fig0, axes = plt.subplots(3, 1, figsize=(12, 8))
        fig0.suptitle('Figure 0: Magnetometer Data Before and After Calibration', 
                     fontsize=14, fontweight='bold')
        
        # X axis
        axes[0].plot(self.timestamps, self.mag_x, 
                    label='Before', alpha=0.7, linewidth=1.5, color='#1f77b4')
        axes[0].plot(self.timestamps, mag_corrected[:, 0], 
                    label='After', alpha=0.9, linewidth=1.5, color='#ff7f0e')
        axes[0].set_ylabel('Mag X (μT)', fontsize=11)
        axes[0].legend(fontsize=10)
        axes[0].grid(True, alpha=0.3)
        
        # Y axis
        axes[1].plot(self.timestamps, self.mag_y, 
                    label='Before', alpha=0.7, linewidth=1.5, color='#1f77b4')
        axes[1].plot(self.timestamps, mag_corrected[:, 1], 
                    label='After', alpha=0.9, linewidth=1.5, color='#ff7f0e')
        axes[1].set_ylabel('Mag Y (μT)', fontsize=11)
        axes[1].legend(fontsize=10)
        axes[1].grid(True, alpha=0.3)
        
        # Z axis
        axes[2].plot(self.timestamps, self.mag_z, 
                    label='Before', alpha=0.7, linewidth=1.5, color='#1f77b4')
        axes[2].plot(self.timestamps, mag_corrected[:, 2], 
                    label='After', alpha=0.9, linewidth=1.5, color='#ff7f0e')
        axes[2].set_ylabel('Mag Z (μT)', fontsize=11)
        axes[2].set_xlabel('Time (s)', fontsize=11)
        axes[2].legend(fontsize=10)
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('fig0_magnetometer_calibration.png', dpi=300, bbox_inches='tight')
        print("  ✓ Saved: fig0_magnetometer_calibration.png")
        plt.close()
        
        # ===== FIGURE 1: Magnetometer Yaw =====
        print("Generating Figure 1: Magnetometer yaw...")
        yaw_before = self.compute_yaw(self.mag_x, self.mag_y)
        yaw_after = self.compute_yaw(mag_corrected[:, 0], mag_corrected[:, 1])
        
        fig1, ax = plt.subplots(figsize=(12, 6))
        ax.plot(self.timestamps, np.degrees(yaw_before), 
               label='Before Calibration', alpha=0.7, linewidth=1.5, color='#1f77b4')
        ax.plot(self.timestamps, np.degrees(yaw_after), 
               label='After Calibration', alpha=0.9, linewidth=1.5, color='#ff7f0e')
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Yaw (degrees)', fontsize=11)
        ax.set_title('Figure 1: Magnetometer Yaw Estimation', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig('fig1_mag_yaw.png', dpi=300, bbox_inches='tight')
        print("  ✓ Saved: fig1_mag_yaw.png")
        plt.close()
        
        print("\n✅ Calibration complete!")
        print("   Next: Run analyze_imu_gps.py with data_driving bag\n")

def main():
    if len(sys.argv) < 2:
        print("\nUsage: python3 analyze_calibration.py <circles_bag_path>")
        print("\nExample:")
        print("  python3 analyze_calibration.py ~/gnss/bags/data_going_in_circles")
        print()
        sys.exit(1)
    
    bag_path = sys.argv[1]
    if bag_path.endswith('.mcap'):
        bag_path = bag_path[:-5]
    
    analyzer = CalibrationAnalyzer(bag_path)
    analyzer.read_bag()
    analyzer.calibrate_magnetometer()
    analyzer.save_calibration('mag_calibration.json')
    analyzer.plot_results()

if __name__ == '__main__':
    main()
