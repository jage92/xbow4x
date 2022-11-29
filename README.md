# xbow4x
Interface library for a Crossbow AHRS400CC-100

This interface is heavily based on David Hodo interface library for [Crossbow IMU440](https://github.com/GAVLab/xbow440) and also in Mobility software developed by Robert T Pack, Steve Clark, Mike Davis and Tyson D. Sawyer for Real World Interface Inc.

This interface is heavily based on David Dias interface library for [XBOW6X](https://github.com/diasdm/xbow6x)

The first steps with the model Crossbow AHRS400CC were implemented by Jesus Morales from University of Malaga

Requires the [serial library](https://github.com/RFRIEDM-Trimble/serial-ros2). 

Tested on ROS Galactic and with Crossbow AHRS400CC.

## Getting Started

Communication with Crossbow AHRS400CC-100 is done through RS-232, so you will need a compatible serial adaptor.

Clone xbow4x package to your catkin workspace and compile it.

Connect the adaptor to the Crossbow and check the correspondent serial port, in my case is /dev/ttyUSB1. You can check serial ports by running:
```
dmesg | grep tty
```
If it is a different one change the serialPort argument in  xbow4x/launch/xbow4x.launch.

Give the following permissions to your serial port by running:
```
sudo chmod 666 /dev/ttyUSB1
```
As a permanent alternative it is possible to set udev rules for your serial adaptor.

## ROS Services

- send_command: allows to send commands to the IMU with which to activate the
 different operating modes, obtain data and sensor information.
    Usage: send_Command [command]. Commands:
	- R: Reset. Stops all calculations and communications, reset the IMU keeps the measurement mode and leaves the IMU in polled mode.
	- S: Returns the IMU serial number.
	- v: Returns the version of the IMU in use.
	- G: Returns a data packet in polling mode. For continuous mode, switching to polling mode.
	- P: Polling mode. If continuous mode is active, stops it and activates polling mode. The G command is used to poll packets.
	- C: Continuous mode. Returns data packets at a frequency of 60 Hz.
	- a: Angle mode. Returns angular velocity, linear acceleration, magnetic field and the angles (roll, pitch and yaw). In this mode a Kalman Filter is used to filter the signal.
	- c: Scaled Mode. Returns angular velocity, linear acceleration and magnetic field. This mode corrects the measurements.
	- r: Voltage mode. Returns angular velocity, linear acceleration and magnetic field. The data are raw, uncorrected and uncalibrated.
- calibrate: allows to send calibration commands.
    Usage: calibrate [command]. Commands:
	- s: starts calibration mode. Stops the other modes (continuous or polling). While the calibration is active the other modes do not work, it is necessary to stop the calibration.
	- u: stops calibration mode. The message mode active after it is polling mode.
	- h: clear hard iron calibration data
	- t: clear soft iron calibration data
- set_baudrate: allows to use the automatic IMU baud rate setting functionality. 
    Usage: set_baudrate [baudrate]
Note: does not work as well as it should because the IMU make a response but the baudrate not change.
- status: returns IMU status, measurement and communication modes enabled.
    Usage: status
- broadcast_ft: allows to enable and disable the broadcasting of the IMU reference
 system with respect to the WORD reference system.
    Usage: broadcast_ft [true/false]

More details in the AHRS400CC-100 documentation.

