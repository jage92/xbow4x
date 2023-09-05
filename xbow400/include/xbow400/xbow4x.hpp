/*!
 * \file include/xbow4x.h
 * \author David Hodo <david.hodo@gmail.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 David Hodo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides an interface for the Crossbow AHRS400CC-100 inertial measurement unit.
 *
 * This library depends on CMake-2.4.6 or later: http://www.cmake.org/
 * This library depends on Serial: https://github.com/wjwwood/serial
 *
 */


#ifndef XBOW4X_H
#define XBOW4X_H

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <queue>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <poll.h>

using namespace std;

// Serial Header
//#include "serial/serial.h"

// ROS Header
#include <rclcpp/rclcpp.hpp>

#define MAX_BYTES_SKIPPED  100

namespace xbow4x{

enum class MeasurementType{None,AngleMode='a',ScaledMode='c',VoltageMode='r'};
enum class MessageMode{None,Continous='C',Poll='P'};

//accelerations and turning rates
struct ImuData {
    int datatype; //1 for accels and turning rates, 2 for delta v's and delta thetas.
    rclcpp::Time receive_time;
    double roll; //1:rad 2:rad
    double pitch; //1:rad 2:rad
    double yaw; //1:rad 2:rad
    double ax; //1:g's 2:m/s
    double ay; //1:g's 2:m/s
    double az; //1:g's 2:m/s
    double rollrate; //1:rad/s 2:rad
    double pitchrate; //1:rad/s 2:rad
    double yawrate; //1:rad/s 2:rad
    double xmag; //gauss
    double ymag; //gauss
    double zmag; //gauss
    double boardtemp; //degC
    unsigned short counter; //16-bit counter (timer)
    unsigned short bitstatus;
};

/*!
 * This function type describes the prototype for the data callback.
 *
 * The function takes a xbow4x::imuData reference and returns nothing.  It is
 * called from the library when new data arrives and is parsed.
 *
 * @see XBOW4X::setDataHandler
 */

class XBOW4X{
public:

  XBOW4X(rclcpp::Logger logger);

  ~XBOW4X(){};

    /*!
     * Connects to the XBOW4X IMU given a serial port.
     *
     * @param port Defines which serial port to connect to in serial mode.
     * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
     *
     * @throws ConnectionFailedException connection attempt failed.
     * @throws UnknownErrorCodeException unknown error code returned.
     */
    //void connect(std::string port, int baudrate=38400, long timeout=50);
    void openPort(const char *port_name, int baudrate);

   /*!
    * Disconnects from the serial port
    */
    //void disconnect();
    void closePort();

    /*!
     * Send a command to the IMU
     * Commands in the Crossbow documentation
     *
     * \param command string of a one character that contains the command to be send
     * \param returnMessage message returned to show to the user
     */
    string sendCommand(uint8_t command);

    /*!
     * Send a command to the IMU to calibrate the magnetometer
     * Commands in the Crossbow documentation
     *
     * \param command string of a one character that contains the command to be send
     * \param returnMessage message returned to show to the user
     */
    string calibrateCommand(uint8_t command);

//DO NOT WORK, IMU DOES NOT REPLY
//    /*!
//     * Send a command to the IMU to calibrate the magnetometer
//     * Commands in the Crossbow documentation
//     *
//     * \param command string of a one character that contains the command to be send
//     * \param returnMessage message returned to show to the user
//     */
//    string setBaudrate(u_int32_t baudrate);
    MeasurementType getMeasurementType() const;

    MessageMode getMessageMode() const;

    /*!
     * Reads the serial port and check the checksum. If the packet is not correct discarts it.
     */
    ImuData readSerialPort();


    bool getReadingStatus();

    /*!
    * Starts a thread to continuously read from the serial port.
    *
    * Starts a thread that runs 'ReadSerialPort' which constantly reads
    * from the serial port.  When valid data is received, parse and then
    *  the data callback functions are called.
    *
    * @see XBOW4X::DataCallback, XBOW4X::XBOW4X::ReadSerialPort, XBOW4X::XBOW4X::StopReading
    */
    void startContinuousReading();

   /*!
    * Starts the thread that reads from the serial port
    *
    * @see XBOW4X::XBOW4X::ReadSerialPort, XBOW4X::XBOW4X::StartReading
    */
    void stopContinousReading();

private:
   /*!
    * Parses a packet of data from the IMU in angle measurement type.  Scale factors are
    * also applied to the data to convert into engineering units.
    */
    void parseAngleMode(unsigned char *packet);

    /*!
     * Parses a packet of data from the IMU in scale measurement type.  Scale factors are
     * also applied to the data to convert into engineering units.
     */
     void parseScaledMode(unsigned char *packet);

     /*!
      * Parses a packet of data from the IMU in voltage measurement type.  Scale factors are
      * also applied to the data to convert into engineering units.
      */
     void parseVoltageMode(unsigned char *packet);

     /*!
      * Reopens the serial port and cleans it
      *
      */
     void reopenSerialPort();

     /**
      * @brief send sends data to the IMU throw serial port
      * @param cmd data to send
      * @param cmd_len length of the data to send
      * @return length of the sent data
      */
     int send(uint8_t *cmd, int cmd_len);

     //!
     /**
      * @brief read_data Read data from the IMU throw serial port
      * @param buff data read
      * @param count length of the data read
      * @param timeout time maximun to wait for the data (default 1000 ms)
      * @return lenght of the data read
      */
     int read_with_timeout(uint8_t *buff, size_t count, int timeout);
     int read_data(uint8_t *rep, uint8_t first_byte, int rep_len, int timeout=1000);


    //! Serial port object for communicating with sensor
//    serial::Serial *serial_port_;
    //! most recently parsed IMU data
    ImuData imu_data_;

   /*!
    * The number of bytes read during each call to read on the serial
    * port.  This value is set intially to 31 (assumes an S2 packet),
    * but is modified by Resync to match the size of the packets actually
    * being received.
    */
    size_t read_size_;
    bool reading_status_;  //!< True if the read thread is running, false otherwise.
    MeasurementType measurementType; //!< Measurements type
    MessageMode messageMode;
    char port[100]; //! Serial port file in use
    int baudrate; //! Serial port daudrate in use
    long timeout; //! Serial port timeout un use
    bool calibrationModeEnabled; //! Indicates if calibration mode is started
    rclcpp::Logger logger_;
    //rclcpp::Clock clock;

    //! The file descriptor
    int fd;

};

} // end namespace

#endif

