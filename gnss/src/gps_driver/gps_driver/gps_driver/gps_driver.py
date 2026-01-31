#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import utm
import time
from custom_interfaces.msg import Customgps
from std_msgs.msg import Header

def utc_to_seconds(utc_time):
    """Convert UTC time from GPGGA format to seconds since midnight"""
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

def parse_gpgga(gpgga_string):
    """Parse GPGGA string and return all needed values"""
    if not gpgga_string.startswith('$GPGGA'):
        return None
    
    gpggaSplit = gpgga_string.split(',')
    
    if len(gpggaSplit) < 15:
        return None
    
    try:
        UTC = float(gpggaSplit[1])
        Latitude = float(gpggaSplit[2])
        LatitudeDir = gpggaSplit[3]
        Longitude = float(gpggaSplit[4])
        LongitudeDir = gpggaSplit[5]
        altitude = float(gpggaSplit[9]) if gpggaSplit[9] else 0.0
        HDOP = float(gpggaSplit[8]) if gpggaSplit[8] else 0.0
        
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
            'gpgga_read': gpgga_string.strip()
        }
    except (ValueError, IndexError) as e:
        return None

class GPSDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/pts/2')
        self.declare_parameter('baudrate', 4800)
        
        # Get parameters
        serial_port = self.get_parameter('port').value
        serial_baud = self.get_parameter('baudrate').value
        
        # Create publisher
        self.gps_pub = self.create_publisher(Customgps, '/gps', 10)
        
        self.get_logger().info(f'GPS driver started on {serial_port} at {serial_baud} baud')
        
        # Open serial port
        try:
            self.serial_port = serial.Serial(serial_port, serial_baud, timeout=1.0)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Create timer to read GPS data
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        """Read and publish GPS data"""
        if self.serial_port.in_waiting > 0:
            try:
                gpgga_line = self.serial_port.readline().decode('utf-8').strip()
                parsed_data = parse_gpgga(gpgga_line)
                
                if parsed_data:
                    gps_msg = Customgps()
                    
                    # Fill in header
                    gps_msg.header = Header()
                    gps_msg.header.stamp.sec = parsed_data['utc_sec']
                    gps_msg.header.stamp.nanosec = parsed_data['utc_nsec']
                    gps_msg.header.frame_id = 'GPS1_Frame'
                    
                    # Fill in GPS data
                    gps_msg.latitude = parsed_data['latitude']
                    gps_msg.longitude = parsed_data['longitude']
                    gps_msg.altitude = parsed_data['altitude']
                    gps_msg.utm_easting = parsed_data['utm_easting']
                    gps_msg.utm_northing = parsed_data['utm_northing']
                    gps_msg.zone = parsed_data['zone']
                    gps_msg.letter = parsed_data['letter']
                    gps_msg.hdop = parsed_data['hdop']
                    gps_msg.gpgga_read = parsed_data['gpgga_read']
                    
                    # Publish
                    self.gps_pub.publish(gps_msg)
                    self.get_logger().info(f'Published GPS: Lat={gps_msg.latitude:.6f}, Lon={gps_msg.longitude:.6f}')
                    
            except UnicodeDecodeError:
                self.get_logger().warn('Failed to decode serial data')
            except Exception as e:
                self.get_logger().warn(f'Error processing GPS data: {e}')
    
    def destroy_node(self):
        """Cleanup when shutting down"""
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
