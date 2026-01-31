# Robotics-Sensing-and-Navigation
## Overview 
This lab taught me how to develop ROS2 sensors drivers for GPS and IMU,collect sensor data using ROSbag files, and leverage serial emulator for hardware-independent testing. I analyzed how environmental factors(obstructions, motion) affect sensor accuracy by collecting datasets in various scenarios and evaluating metrics like HDOP, positioning error and IMU noise characteristics.

# The use of sensor emulator
- The sensor emulator will behave like GPS or IMU. It will write data(either NMEA strings or VectorNav strings) to specific serial port, thus emulating the sensor's output.
- To get the serial emulator, clone this repository with: $ git clone https://github.com/ECE-RSN/sensor_emulator/
- The emulator has a dependencies on the pyserial module, so you need to first get that by: $ pip3 install pyserial
- Then run the emulator: $ python3 serial_emulator.py --file GPS_Chicago.txt --device_type gps --loop "no"    
(why loop? GPS data is continious streaming-satellite always broadcast)
- Run the IMU: $ python serial_emulator.py --file imu_data.txt --device_type imu -V appropriate-VectorNav-string
(why no loop? as IMU testing often focus on specific motion sequences; why validation?: VectorNav IMUs have specific output formats that must be verified)
- The emulator will print pseudo device address /dev/pts/N to the terminal. You can use this pseudo address to test your driver or see output on minicom with: minicom -D /dev/pts/N

# Lab1: GPS Data Collection and Analysis

In this lab, I learned how to write python program to parse data read over USB serial of GPS and transfer the latitude and longtitude to UTM and then publish a custom message using a ROS publisher node /gps. Finally save data using a rosbag file and analyze the northing/easting centroid of each data set.

## Data Collection
In this lab, I collected three kinds of GPS data: 
1. 5 mins stationary data in an open area
2. 5 min stationary data in an occluded area(trees or building nearby)
3. walking data as I walking approximately 200m

## Results
here is the plot we get: 

<p align = "center">
  <img width="400" height="350" alt="gps1" src="https://github.com/user-attachments/assets/4e24ea40-b243-4636-bf38-2ce297ffc947" />
  <img width="400" height="350" alt="gps2" src="https://github.com/user-attachments/assets/ac2ae139-bb99-46cc-8bad-4e6e05a27b26" />
  <img width="600" height="350" alt="gps3" src="https://github.com/user-attachments/assets/eeb7afe2-21c6-4622-b8ab-ff2e9e2db571" />
  <img width="400" height="350" alt="gps4" src="https://github.com/user-attachments/assets/44fde23a-e80f-42b9-91eb-e2be3c7988a6" />
  <img width="450" height="350" alt="gps5" src="https://github.com/user-attachments/assets/65ba5567-25a4-4de4-8c54-f1fdf9b60378" />
</p>p

It's clear that the occluded data have more error/deviation compared with opening data.

## Analysis

### Do errors match HDOP(horizontal dilution of precision) predictions?

**Open Area**
- HDOP: 2.0
- Expected error: HDOP x Base error = 2 x 1-2m = **2-4m**
- Measures precision: **2.15m**
- **Conclusion:** Matches prediction

**Occluded Area**
- HDOP: 4.0
- Expected error: HDOP x Base error = 4 x 1-2m = **8m**
- Measures precision: **8.01m**
- **Conclusion:** Matches prediction

# Lab2
In this Lab, we further delve into GNSS and introduces real time kinematics(RTK) GNSS. I use my GNSS data from Lab1 compare standalone GNSS data and RTK GNSS data. I also further analyze walking and stationary data collected using RTK GPS.

***Standard GPS vs RTK GPS***
Standard GPS
How it works:
- Receive gets signals from satallites
- calculate position independently
- Accuracy: 2-10 meters

Error sources:
- Atmospheric delays
- Satellite clock error
- Multipath reflection

Standard GPS
How it works:
- Uses TWO receivers:
  1. Base station - at known, fixed location
  2. Rover - your mobile receiver
- Base stations calculates its errors, sends corrections to rover
- Rover applies correction - cm-level accuracy!

## Date collection
Since we don't have access to RTK GPS hardware, we use pre-collected datasets instead.To emulate the RTK sensor output, we run these datasets through a serial emulator to generate Ros2bag files for analysis. 

**Stationary Occluded**
- File: occludedRTK.txt
- Reference GPS Coordinates: (42.3372702, -71.0869305)
- Data Collection Duration: 5 minutes 11 seconds

**Stationary Open**
- File: openRTK.txt
- Reference GPS Coordinates: (42.3372702, -71.0869305)
- Data Collection Duration: 5 minutes 21 seconds

**Walking Open**
- File: walkingRTK.txt
- Reference GPS Coordinates:  Start - (42.3381079, -71.0866221) | End - (42.3385825, -71.0860263)
- Data Collection Duration: 5 minutes 11 seconds

## Results & Analysis
Here is the link of full report: (https://drive.google.com/drive/folders/1PlWLl1iZByoK3ymxwEBpUAyies0ycJsj?dmr=1&ec=wgc-drive-globalnav-goto)

# Lab3
I write a device driver that communicate with VectorNav IMU over USB serial and identify and device parameters and source the error using Allan variance tools.

***Why use  Allan Variance? ***
As IMU data contains various kinds of noises,which we could identify by coefficient N B K. But if the datasheet doesn't provide that,one approach is to analyze the power spectrum of your IMU an fit a model to find the coefficients. The most commonly used tool is Allan Variance.
***Key IMU error Paramters***

- Angle Random Walk
White noise in gyroscope measurements
High-frequency, random fluctuations
changes every measurements

**Allan Variance signature:**
**Slope: -1/2** on log-log plot
Read at **τ = 1 seconds** 

- Bias Instability (BI)
Slow drift in sensor bias over time
Bias randomly walks around

**Allan Variance signature:**
**Slope: flat region
Read at lowest point on the plot

- Rate Random Walk (RRW)
Bias change is itself random
Long-term drift

**Allan Variance signature:**
**Slope: +1/2** on right side of plot
Read at **τ = 3 seconds** (convention)

reference: (https://www.tangramvision.com/blog/the-allan-deviation-and-imu-error-modeling-part-4-of-5)


## Data collection
- stationary data set for 5 minutes of data collected when the IMU is far away from any sources of vibrations or electrical noise.
- one collected 5 h rosbag
- one short rosbag collected by my group of an interesting motion.

## Results & Analysis
Here is the Allan varaince plot i get, I can determined N B and K based on the curve:
/home/xingyue/Downloads/pic-of-lab/Allan-variables.png
 N (Angle Random Walk)( Read at τ=1s)
 Gyro X axis: 3×10⁻³ deg/√hr
 Gyro Y axis: 8×10⁻³
 Gyro Z axis: 15
 
 B (Bias Instability)( Read at minimum)
 Gyro X axis: 3×10⁻³ deg/√hr
 Gyro Y axis: 9×10⁻³
 Gyro Z axis: 0.45
 
 K (Rate Random Walk)( Read at τ=3s)
 Gyro X axis: 2.5×10⁻³ deg/√hr
 Gyro Y axis: 7×10⁻³
 Gyro Z axis: 11
 
 Analyze of motion:
/home/xingyue/Downloads/pic-of-lab/1.png
/home/xingyue/Downloads/pic-of-lab/2.png
/home/xingyue/Downloads/pic-of-lab/3.png
- Fig4: Gyro Y shows large rotation about Y-axis while Gyro X and Gyro Z have smaller variations, which corresponds to a horizontal arm swing.
- Fig5: Gyro X shows dominant peak, Gyro Y is also large but Gyro Z is the most stable one, which align with the 3D rotation motion.
- Fig6: Gyro Y shows two peaks in the plot,which can perfectly represents my up and down motion in the picture. Also Gyro X maintains stable in the whole process.

# Lab4


## Data collection

## Results & Analysis


# Lab5


## Data collection

## Results & Analysis






