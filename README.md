# xbow6x
Interface library for a Crossbow DMU-6X-003

This interface is heavily based on David Hodo interface library for [Crossbow IMU440](https://github.com/GAVLab/xbow440) and also in Mobility software developed by Robert T Pack, Steve Clark, Mike Davis and Tyson D. Sawyer for Real World Interface Inc.

Requires the [serial library](https://github.com/wjwwood/serial). Last tested for version 1.2.1.

Tested on ROS Kinetic.

## Getting Started

Communication with Crossbow DMU-6X-003 is done through RS-232, so you will need a compatible serial adaptor.

Clone xbow6x package to your catkin workspace and compile it.

Connect the adaptor to the Crossbow and check the correspondent serial port, in my case is /dev/ttyUSB1. You can check serial ports by running:
```
dmesg | grep tty
```
If it is a different one change the serialPort argument in  xbow6x/launch/xbow6x.launch.

Give the following permissions to your serial port by running:
```
sudo chmod 666 /dev/ttyUSB1
```
As a permanent alternative it is possible to set udev rules for your serial adaptor.

