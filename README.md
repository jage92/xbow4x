# xbow4x
Interface library for a Crossbow AHRS400CC-100

This interface is heavily based on David Hodo interface library for [Crossbow IMU440](https://github.com/GAVLab/xbow440) and also in Mobility software developed by Robert T Pack, Steve Clark, Mike Davis and Tyson D. Sawyer for Real World Interface Inc.

This interface is heavily based on David Dias interface library for [XBOW6X](https://github.com/diasdm/xbow6x)

The first steps with the model Crossbow AHRS400CC were implemented by Jesus Morales from University of Malaga

Requires the [serial library](https://github.com/wjwwood/serial). Last tested for version 1.2.1.

Tested on ROS Noetic and with Crossbow AHRS400CC.

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

## ROS Topics

The read sensor measures are published on a different topic:
- /microstrain_3dmgx2_imu/imu/data: publishes the scaled sensor measures of the acceleration, angular velocity, and orientation.
- /microstrain_3dmgx2_imu/mag: publishes the scaled sensor measures of the magnetic field
- /microstrain_3dmgx2_imu/imu/data_no_grab: publishes the scaled sensor measures of the acceleration without gravity component, angular velocity, and orientation.

   Althought the IMU can be in different measurement type, the topics are the same and publish the same data, only chage how the sensor obtain that data. When the orientation
   is not available, the identity quaternion is published.

## ROS Services
Different node services in order to be able to use all IMU functionalities

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

## Params

The node can be configured with different parameters that allow users to use the sensor as they need.
- port: serial port to read IMU data.
- baudrate: the baudrate with the IMU and the node make the communication
- frame_id: name of the local reference frame of the IMU.
- base_frame: name of the base reference frame with respect to the frame_id TF is published
- broadcast_tf: if true the node publishes the IMU TF (ENU or NED) with respect to the base frame.
- use_enu_frame: if true the IMU data is published with respect ENU reference frame. The IMU printed frame not corresponds in this case. If false the NED reference frame is used and corresponds with the IMU printed frame.
- measurement_mode: Allows to select the mode which the IMU will use to configure the internal sensors
    - a: Angle mode. Returns angular velocity, linear acceleration, magnetic field and the angles (roll, pitch and yaw). In this mode a Kalman Filter is used to filter the signal.
    - c: Scaled Mode. Returns angular velocity, linear acceleration and magnetic field. This mode corrects the measurements.
    - r: Voltage mode. Returns angular velocity, linear acceleration and magnetic field. The data are raw, uncorrected and uncalibrated.
- message_mode: Allows the IMU to publish the data continuously or pooling. To pool data see the services.
    - P: Polling mode. If continuous mode is active, stops it and activates polling mode. The G command is used to poll packets.
    - C: Continuous mode. Returns data packets at a frequency of 60 Hz.

The file imu_params.yaml contains different interesting matrices. The covariance matrices are calculated by obtaining data with the IMU immobile.
- tf_translation: translation vector of the IMU reference frame with respect to the ENU frame called world
- orientation_cov: covariance matrix of the covariance of the Euler angles obtained from de IMU
- ang_vel_cov: covariance matrix of the scaled angular velocity
- lin_acc_cov: covariance matrix of the scaled linear acceleration
- lin_acc_cov_no_grab: covariance matrix of the scaled linear acceleration without the gravity component
- mag_cov: covariance matrix of the scaled magnetic field vector
More details in the AHRS400CC-100 documentation.

## Test

To check the working rate of the IMU it is possible to use the ../diagnostics topic. 

To make a node test use: 

`rosrun self_test run_selftest /self_test`

or 

`rosservice call /self_test`

with the node runnig. 
