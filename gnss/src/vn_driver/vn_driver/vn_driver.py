#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from custom_interfaces.msg import Vectornav  

class IMUDriver(Node):
    def __init__(self):
        super().__init__('vn_driver')
        
        # Step 1: Declare parameters (similar to your GPS driver)
        self.declare_parameter('port', '/dev/ttyUSB0')  # emulator port
        self.declare_parameter('baudrate', 115200)
        
        # Get parameters
        serial_port = self.get_parameter('port').value
        serial_baud = self.get_parameter('baudrate').value
        
        # Create publisher for IMU data
        self.imu_pub = self.create_publisher(Vectornav, '/imu', 10)
        
        # Step 2: Open serial port
        self.serial = serial.Serial(serial_port, serial_baud, timeout=1)
        
        self.get_logger().info(f'IMU driver started on {serial_port}')
        
        # Create timer to read data periodically
        self.timer = self.create_timer(0.01, self.read_imu_data)  # 100Hz
    
    def configure_output_rate(self):  # ADD THIS ENTIRE METHOD
        """
        Configure VectorNav to output data at 40Hz
        Writes to register 7 (async data output frequency)
        """
        # Construct command: $VNWRG,07,40*checksum
        command_base = "VNWRG,07,40"
        
        # Calculate checksum (XOR of all bytes)
        checksum = 0
        for char in command_base:
            checksum ^= ord(char)
        
        # Format complete command with $ prefix, * separator, and \r\n terminator
        command = f"${command_base}*{checksum:02X}\r\n"
        
        self.get_logger().info(f'Configuring VectorNav to 40Hz: {command.strip()}')
        
        # Write to serial port
        self.serial.write(command.encode('utf-8'))
        
        # Wait for response
        time.sleep(0.1)
        if self.serial.in_waiting > 0:
            response = self.serial.readline().decode('utf-8').strip()
            self.get_logger().info(f'VectorNav response: {response}')
    
    
    def read_imu_data(self):
        # Step 3: Read serial data
        if self.serial.in_waiting > 0:
            line = self.serial.readline().decode('utf-8').strip()
            
            # Step 4: Check if it's a VNYMR string
            if line.startswith('$VNYMR'):
                # Step 5: Parse the data (do NOT use pynmea)
                self.parse_vnymr(line)
    
    def parse_vnymr(self, line):
        # Step 6: Manual parsing
        # $VNYMR format: $VNYMR,yaw,pitch,roll,magX,magY,magZ,accelX,accelY,accelZ,gyroX,gyroY,gyroZ*checksum
        
        try:
            # Remove '$VNYMR,' prefix and checksum
            data = line.split(',')
            
            # Extract values (refer to the user manual for exact indices)
            yaw = float(data[1])
            pitch = float(data[2])
            roll = float(data[3])
            mag_x = float(data[4])
            mag_y = float(data[5])
            mag_z = float(data[6])
            accel_x = float(data[7])
            accel_y = float(data[8])
            accel_z = float(data[9])
            gyro_x = float(data[10])
            gyro_y = float(data[11])
            gyro_z = float(data[12].split('*')[0])  # Remove checksum
            
            # Step 7: Create and publish message
            msg = Vectornav()
            msg.mag_field.header.stamp = self.get_clock().now().to_msg()
            msg.mag_field.header.frame_id = "IMU1_Frame"

            msg.imu.linear_acceleration.x = accel_x
            msg.imu.linear_acceleration.y = accel_y
            msg.imu.linear_acceleration.z = accel_z
            msg.imu.angular_velocity.x = gyro_x
            msg.imu.angular_velocity.y = gyro_y
            msg.imu.angular_velocity.z = gyro_z
            msg.mag_field.magnetic_field.x = mag_x
            msg.mag_field.magnetic_field.y = mag_y
            msg.mag_field.magnetic_field.z = mag_z
            msg.mag_field.magnetic_field_covariance = [0.0] * 9
            msg.raw_imu_string = line
            
            self.imu_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error parsing VNYMR: {e}')

def main(args=None):
    rclpy.init(args=args)
    imu_driver = IMUDriver()
    rclpy.spin(imu_driver)
    imu_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
