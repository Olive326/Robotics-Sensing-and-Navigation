#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import utm
import serial
import time
from custom_interfaces.msg import Customrtk
from std_msgs.msg import Header

def utc_to_seconds(utc_time):
    """Convert UTC time from GNGGA format to seconds since midnight"""
    hours = int(utc_time / 10000)
    minutes = int((utc_time % 10000) / 100)
    seconds = utc_time % 100
    total_seconds = hours * 3600 + minutes * 60 + seconds
    return total_seconds

def UTCtoUTCEpoch(UTC):
    """Convert UTC time to Unix epoch timestamp"""
    UTCinSecs = utc_to_seconds(UTC)
    TimeSinceEpoch = time.time()
    TimeSinceEpochBOD = TimeSinceEpoch - (TimeSinceEpoch % 86400) - time.timezone
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime)
    CurrentTimeNsec = int((CurrentTime - CurrentTimeSec) * 1_000_000_000)
    return [CurrentTimeSec, CurrentTimeNsec]

def defMinstoDegDec(Lat0rLong):
    """Convert NMEA format (DDMM.MMMM) to decimal degrees"""
    deg = Lat0rLong // 100
    mins = Lat0rLong - deg * 100
    degDec = mins / 60
    return (deg + degDec)

def LatLongSignConvetion(Lat0rLong, Lat0rLongDir):
    """Apply correct sign based on hemisphere"""
    if Lat0rLongDir in ["W", "S"]:
        Lat0rLong = -float(Lat0rLong)
    return Lat0rLong

def parse_gngga(gngga_string):
    """Parse GNGGA string and return all needed values"""
    if not gngga_string.startswith('$GNGGA'):
        return None
    
    gnggaSplit = gngga_string.split(',')
    
    if len(gnggaSplit) < 15:
        return None
    
    try:
        UTC = float(gnggaSplit[1])
        Latitude = float(gnggaSplit[2])
        LatitudeDir = (gnggaSplit[3])
        Longitude = float(gnggaSplit[4])
        LongitudeDir = (gnggaSplit[5])
        fix_quality = int(gnggaSplit[6])
        altitude = float(gnggaSplit[9]) if gnggaSplit[9] else 0.0
        HDOP = float(gnggaSplit[8]) if gnggaSplit[8] else 0.0
        
        new_Latitude = defMinstoDegDec(Latitude)
        new_Longitude = defMinstoDegDec(Longitude)
        
        LatitudeSigned = LatLongSignConvetion(new_Latitude, LatitudeDir)
        LongitudeSigned = LatLongSignConvetion(new_Longitude, LongitudeDir)
        
        UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
        UTMEasting = UTMVals[0]
        UTMNorthing = UTMVals[1]
        UTMZone = UTMVals[2]
        UTMLetter = UTMVals[3]
        
        CurrentTime = UTCtoUTCEpoch(UTC)
        
        return {
            'utc_sec': CurrentTime[0],
            'utc_nsec': CurrentTime[1],
            'latitude': LatitudeSigned,
            'longitude': LongitudeSigned,
            'altitude': altitude,
            'utm_easting': UTMEasting,
            'utm_northing': UTMNorthing,
            'zone': UTMZone,
            'letter': UTMLetter,
            'hdop': HDOP,
            'fix_quality': fix_quality,
            'gngga_read': gngga_string.strip()
        }
    except (ValueError, IndexError) as e:
        return None

class RTKDriverNode(Node):
    def __init__(self):
        super().__init__('rtk_driver')
        
        # Declare serial port parameter
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        
        self.get_logger().info(f'Connecting to serial port: {serial_port}')
        
        # Create publisher
        self.rtk_pub = self.create_publisher(Customrtk, '/rtk', 10)
        
        # Open serial port
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=1.0
            )
            self.get_logger().info('Serial connection established!')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Create timer to read serial data at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    
    def timer_callback(self):
        """Read and publish RTK data from serial port"""
        try:
            if self.serial_conn.in_waiting > 0:
                # Read one line from serial
                gngga_line = self.serial_conn.readline().decode('utf-8').strip()
                
                parsed_data = parse_gngga(gngga_line)
                
            
                if parsed_data:
                    rtk_msg = Customrtk()
                    
                    # Fill in header
                    rtk_msg.header = Header()
                    rtk_msg.header.stamp.sec = parsed_data['utc_sec']
                    rtk_msg.header.stamp.nanosec = parsed_data['utc_nsec']
                    rtk_msg.header.frame_id = 'RTK1_Frame'
                        
                    # Fill in GPS data
                    rtk_msg.latitude = parsed_data['latitude']
                    rtk_msg.longitude = parsed_data['longitude']
                    rtk_msg.altitude = parsed_data['altitude']
                    rtk_msg.utm_easting = parsed_data['utm_easting']
                    rtk_msg.utm_northing = parsed_data['utm_northing']
                    rtk_msg.zone = parsed_data['zone']
                    rtk_msg.letter = parsed_data['letter']
                    rtk_msg.hdop = parsed_data['hdop']
                    rtk_msg.fix_quality = parsed_data['fix_quality']
                    rtk_msg.gpgga_read = parsed_data['gngga_read']
                        
                    # Publish
                    self.rtk_pub.publish(rtk_msg)
                    self.get_logger().info(
        f'Published RTK: Lat={rtk_msg.latitude:.6f}, Lon={rtk_msg.longitude:.6f}, '
        f'Alt={rtk_msg.altitude:.2f}m, UTM_E={rtk_msg.utm_easting:.2f}, '
        f'UTM_N={rtk_msg.utm_northing:.2f}, Zone={rtk_msg.zone}{rtk_msg.letter}, '
        f'HDOP={rtk_msg.hdop:.2f}'
    )
                    

        except Exception as e:
            self.get_logger().warn(f'Error processing RTK data: {e}')

    def destroy_node(self):
        """Cleanup when shutting down"""
        if hasattr(self, 'serial_conn'):
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RTKDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
