#include "xbow400/xbow4x.hpp"

#define WIN32_LEAN_AND_MEAN
#define _USE_MATH_DEFINES

#define AHRS_ANGLE_MODE_PACKET_SIZE 30
#define AHRS_SCALED_MODE_PACKET_SIZE 24
#define AHRS_VOLTAGE_MODE_PACKET_SIZE 24

using namespace xbow4x;

XBOW4X::XBOW4X(rclcpp::Logger logger) : logger_(logger)
{
  serial_port_ = NULL;
  //data_handler_ = DefaultProcessData;
  read_size_ = AHRS_ANGLE_MODE_PACKET_SIZE;
  reading_status_ = false;
  measurementType = MeasurementType::None;
  messageMode=MessageMode::None;
  calibrationModeEnabled = false;
}

bool XBOW4X::connect(std::string port, int baudrate, long timeout) {
  serial_port_ = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(timeout));
  u_int8_t buffer[100];

  this->port=port;
  this->baudrate=baudrate;
  this->timeout=timeout;

  if (!serial_port_->isOpen()){
    RCLCPP_INFO(logger_,"Serial port: %s failed to open.", port.c_str());
    delete serial_port_;
    serial_port_ = NULL;
    return false;
  } else {
    RCLCPP_INFO(logger_,"Serial port: %s opened successfully.", port.c_str());
    RCLCPP_INFO(logger_,"Searching for IMU...");

    //Cleanning the buffer
    serial_port_->read(buffer,99);
  }
  return true;
}

void XBOW4X::disconnect() {
  RCLCPP_INFO(logger_,"Disconnecting DMU.");
  stopContinousReading();
  serial_port_->close();
  delete serial_port_;
  serial_port_ = NULL;
  measurementType = MeasurementType::None;
}

int XBOW4X::calibrateCommand(u_int8_t command,string &returnMessage) {
  u_int8_t buffer[100];
  int size;
  string s;

  switch (command) {
  case 's':
    if(!calibrationModeEnabled) {
      sendCommand('R',s);
      RCLCPP_INFO(logger_,"Command %c",command);
      serial_port_->write(&command,1);
      size = serial_port_->read(buffer,1);
      RCLCPP_INFO(logger_,"Response %d",buffer[0]);
      if(buffer[0]=='S') {
        calibrationModeEnabled=true;
        returnMessage = "Calibration Mode started";
        return 0;
      }
      returnMessage = "Impossible to communicate with the IMU";
      return 1;
    }
    else {
      returnMessage = "The calibrating is already enabled";
      return 1;
    }
  case 'u':
    if(calibrationModeEnabled) {
      serial_port_->write(&command,1);
      RCLCPP_INFO(logger_,"Command %c",command);
      size = serial_port_->read(buffer,1);
      RCLCPP_INFO(logger_,"Response %d",buffer[0]);
      if(buffer[0]=='U') {
        calibrationModeEnabled=false;
        returnMessage = "Calibration Mode stopped";
        return 0;
      }
      returnMessage = "Impossible to communicate with the IMU";
      return 1;
    }
    else {
      returnMessage = "The calibrating mode is not enabled";
      return 1;
    }
  case 'h':
    if(!calibrationModeEnabled) {
      sendCommand('R',s);
      serial_port_->write(&command,1);
      RCLCPP_INFO(logger_,"Command %c",command);
      size = serial_port_->read(buffer,1);
      RCLCPP_INFO(logger_,"Response %d",buffer[0]);
      if(buffer[0]=='H') {
        returnMessage = "Hard iron calibration cleared";
        return 0;
      }
      returnMessage = "Impossible to communicate with the IMU";
      return 1;
    }
    else {
      returnMessage = "The calibrating is enabled";
      return 1;
    }
  case 't':
    if(!calibrationModeEnabled) {
      sendCommand('R',s);
      serial_port_->write(&command,1);
      RCLCPP_INFO(logger_,"Command %c",command);
      size = serial_port_->read(buffer,1);
      RCLCPP_INFO(logger_,"Response %d",buffer[0]);
      if(buffer[0]=='T') {
        returnMessage = "Soft iron calibration cleared";
        return 0;
      }
      returnMessage = "Impossible to communicate with the IMU";
      return 1;
    }
    else {
      returnMessage = "The calibrating is enabled";
      return 1;
    }
  default:
    returnMessage = "Incorrect command";
    return 1;
    break;
  }
}

void XBOW4X::reopenSerialPort() {
  u_int8_t buffer[100];
  serial_port_->close();
  delete serial_port_;
  serial_port_ = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(timeout));
  serial_port_->read(buffer,99);
}

MessageMode XBOW4X::getMessageMode() const
{
  return messageMode;
}

MeasurementType XBOW4X::getMeasurementType() const
{
  return measurementType;
}

int XBOW4X::setBaudrate(u_int32_t baudrate, string &returnMessage) {

  string s;
  u_int8_t command;
  u_int8_t buffer[100];
  int previousBaudrate;
  sendCommand('R',s);

  command = 'b';
  serial_port_->write(&command,1);
  RCLCPP_INFO(logger_,"Command %c",command);
  serial_port_->read(buffer,1);
  RCLCPP_INFO(logger_,"Response %c",buffer[0]);

  if(buffer[0]=='B') {
    previousBaudrate = this->baudrate;
    this->baudrate=baudrate;
    reopenSerialPort();

    command = 'a';
    serial_port_->write(&command,1);
    RCLCPP_INFO(logger_,"Command %c",command);
    serial_port_->read(buffer,1);
    RCLCPP_INFO(logger_,"Response %c",buffer[0]);

    if(buffer[0]=='A') {
      sendCommand('R',s);
      sendCommand((u_int8_t)measurementType,s);
      sendCommand((u_int8_t)messageMode,s);
      returnMessage = "Baudrate changed";
      return 0;
    }
    else {
      this->baudrate=previousBaudrate;
      reopenSerialPort();
      returnMessage = "It is not possible to set a new baudrate";
      return 1;
    }
  }
  else {
    returnMessage = "Impossible to communicate with the IMU";
    return 1;
  }


}

int XBOW4X::sendCommand(u_int8_t command, string &returnMessage) {
      u_int8_t buffer[100];
      char temp[25];
      int size;
      int num;
      bool flag=false;
      string s;

      if(!calibrationModeEnabled) {
        switch (command) {
        case 'R': //PING
          if(reading_status_) {
            stopContinousReading();
          }
          RCLCPP_INFO(logger_,"Command %c",command);
          serial_port_->write(&command,1);
          size = serial_port_->read(buffer,1);
          RCLCPP_INFO(logger_,"Response %d",buffer[0]);
          //measurementType = MeasurementType::None;
          num=100;
          while(buffer[0]!='H') {
            //SEARCH PING
            RCLCPP_INFO(logger_,"Command %c",command);
            serial_port_->write("G"); //STOP Continuous mode
//            messageMode = MessageMode::Poll;
            reopenSerialPort();
            RCLCPP_INFO(logger_,"Command %c",command);
            //PING
            serial_port_->write(&command,1);
            //sleep(1);
            size = serial_port_->read(buffer,1);
            RCLCPP_INFO(logger_,"Response %d",buffer[0]);
            if(!(--num)) {
              returnMessage = "It is not possible to communicate with the IMU";
              return 1;
            }
          }
          returnMessage = "PING";
          return 0;
        case 'r': //Voltage Mode
          if(reading_status_) {
            sendCommand('R',s);
            flag=true;
          }
          RCLCPP_INFO(logger_,"Command %c",command);
          serial_port_->write(&command,1);
          size = serial_port_->read(buffer,1);
          RCLCPP_INFO(logger_,"Response %c",buffer[0]);
          if(buffer[0]=='R') {
            measurementType = MeasurementType::VoltageMode;
            read_size_ = AHRS_VOLTAGE_MODE_PACKET_SIZE;
            returnMessage = "Voltage Mode enabled";
            if(flag)
              sendCommand('C',s);
            return 0;
          }
          else {
            returnMessage = "It is not possible to communicate with the IMU";
            return 1;
          }
        case 'c': //Scaled Mode
          if(reading_status_) {
            sendCommand('R',s);
            flag=true;
          }
          RCLCPP_INFO(logger_,"Command %c",command);
          serial_port_->write(&command,1);
          size = serial_port_->read(buffer,1);
          RCLCPP_INFO(logger_,"Response %c",buffer[0]);
          if(buffer[0]=='C') {
            measurementType = MeasurementType::ScaledMode;
            read_size_ = AHRS_SCALED_MODE_PACKET_SIZE;
            returnMessage = "Scaled Mode enabled";
            if(flag)
              sendCommand('C',s);
            return 0;
          }
          else {
            returnMessage = "It is not possible to communicate with the IMU";
            return 1;
          }
        case 'a': //Angle Mode
          if(reading_status_) {
            sendCommand('R',s);
            flag=true;
          }
          RCLCPP_INFO(logger_,"Command %c",command);
          serial_port_->write(&command,1);
          size = serial_port_->read(buffer,1);
          RCLCPP_INFO(logger_,"Response %c",buffer[0]);
          if(buffer[0]=='A') {
            measurementType = MeasurementType::AngleMode;
            read_size_ = AHRS_ANGLE_MODE_PACKET_SIZE;
            returnMessage = "Angle Mode enabled";
            RCLCPP_INFO(logger_,"Angle Mode enabled");
            if(flag)
              sendCommand('C',s);
            return 0;
          }
          else {
            returnMessage = "It is not possible to communicate with the IMU";
            RCLCPP_ERROR(logger_,"It is not possible to communicate with the IMU");
            return 1;
          }
        case 'P': //Poll Mode
            if(reading_status_) {
              sendCommand('R',s);
              flag=true;
            }
            RCLCPP_INFO(logger_,"Command %c",command);
            serial_port_->write(&command,1);
            messageMode = MessageMode::Poll;
            returnMessage = "Poll Mode enabled";
            return 0;
        case 'C': //Continuous Mode
            RCLCPP_INFO(logger_,"Command %c",command);
            serial_port_->write(&command,1);
            if(measurementType == MeasurementType::None) {
              returnMessage = "No measurement mode selected";
              return 1;
            }
            if(startContinuousReading()) {
              returnMessage = "Starting continuous mode";
              RCLCPP_INFO(logger_,"%s",returnMessage.c_str());
              return 0;
            }
            else {
              returnMessage = "Imposible to start continuous mode";
              RCLCPP_INFO(logger_,"%s",returnMessage.c_str());
              return 1;
            }
        case 'G': //Request Data
            if(reading_status_) {
              sendCommand('R',s);
            }
            RCLCPP_INFO(logger_,"Command %c",command);
            serial_port_->write(&command,1);
            if(measurementType == MeasurementType::None) {
              returnMessage = "No measurement mode selected";
              return 1;
            }
            messageMode = MessageMode::Poll;
            readSerialPortPollMode();
            returnMessage = "Packet readed";
            return 0;
        case 'v': //Query DMU Version
          if(!reading_status_) {
            RCLCPP_INFO(logger_,"Command %c",command);
            serial_port_->write(&command,1);
            size = serial_port_->read(buffer,26);
            strncpy(temp,(char *)&buffer[1],size-1);
            RCLCPP_INFO(logger_,"Response %s",temp);
            if(strlen(temp)==24) {
              returnMessage = string(temp);
              return 0;
            }
            else {
              returnMessage = "It is not possible to communicate with the IMU";
              return 1;
            }
           }
           else {
              returnMessage = "The IMU is in continous mode, stop it to check the version";
              return 0;
           }

        case 'S': //Query Serial Number
          if(!reading_status_) {
            serial_port_->write(&command,1);
            size = serial_port_->read(buffer,6);
            num = int((unsigned char)(buffer[1]) << 24 |
                        (unsigned char)(buffer[2]) << 16 |
                        (unsigned char)(buffer[3]) << 8 |
                        (unsigned char)(buffer[4]));
            RCLCPP_INFO(logger_,"Response %d",num);
            if(size == 6) {
              returnMessage = to_string(num);
              return 0;
            }
            else {
              returnMessage = "It is not possible to communicate with the IMU";
              return 1;
            }
          }
          else {
             returnMessage = "The IMU is in continous mode, stop it to check the serial number";
             return 0;
          }
        default:
            returnMessage = "Incorrect command";
            return 1;
        }
    }
    returnMessage = "Calibrating mode is active";
    return 1;
}



bool XBOW4X::startContinuousReading() {
  // create thread to read from sensor
  if(measurementType == MeasurementType::None) {
    RCLCPP_ERROR(logger_,"No MEASUREMENT MODE selected");
    return false;
  }
  messageMode = MessageMode::Continous;
  reading_status_=true;
  //read_thread_ptr_ = boost::shared_ptr<boost::thread> (new boost::thread(boost::bind(&XBOW4X::readSerialPortContinuousMode, this)));

  return true;
}

void XBOW4X::stopContinousReading() {
  reading_status_=false;
  serial_port_->write("G"); //STOP Continuous mode
//  messageMode = MessageMode::Poll;
}

ImuData XBOW4X::readSerialPortPollMode() {
  unsigned char buffer[read_size_];
  int len, sum, i;
  string s;

//  if(!serialPort_enabled) //Because concurrency
//    return;
  len = serial_port_->read(buffer, read_size_);
  imu_data_.receive_time = clock.now();
  sum = 0;
  for(int i = 1; i < (len - 1); i++) {
      sum += (int)buffer[i];
  }
  sum = sum % 256;

  // check if we have a complete read and if checksum is correct
  if ((len != read_size_) || (sum != (int)buffer[(len - 1)])) {
    // display data
    RCLCPP_INFO(logger_,"Read data: %d",len);
    for(i = 0; i < len; i++)
        RCLCPP_INFO(logger_,"%d", (int)buffer[i]);
    RCLCPP_INFO(logger_,"");
    RCLCPP_INFO(logger_,"Computed checksum: %d Data checksum: %d",sum, (int)buffer[(len - 1)]);

    sendCommand('R',s);
    RCLCPP_INFO(logger_,"%s",s.c_str());
    sendCommand((u_int8_t)messageMode,s);
    RCLCPP_INFO(logger_,"%s",s.c_str());
  }
  else {
    // parse packet
    if(measurementType == MeasurementType::AngleMode)
      parseAngleMode(buffer);
    else if(measurementType == MeasurementType::ScaledMode)
      parseScaledMode(buffer);
    else if(measurementType == MeasurementType::VoltageMode)
      parseVoltageMode(buffer);
  }
  return imu_data_;

}

void XBOW4X::readSerialPortContinuousMode() {

  while (reading_status_) {
    readSerialPortPollMode();
  }

}

void XBOW4X::parseAngleMode(unsigned char *packet) { //ANGLE MODE
    // We're OK and actually have a good packet /w good checksum,
    // decode it, convert the units and update state:
    if ((packet[1] & 0x80) == 0)
        imu_data_.roll = ((packet[1] << 8) + packet[2]);
    else
        imu_data_.roll = -32768 + ((packet[1] & 0x7F) << 8) + packet[2];
    imu_data_.roll *= M_PI/32768.0; // rad

    if ((packet[3] & 0x80) == 0)
        imu_data_.pitch = ((packet[3] << 8) + packet[4]);
    else
        imu_data_.pitch = -32768 +((packet[3] & 0x7F) << 8) + packet[4];
    imu_data_.pitch *= M_PI/32768.0; // rad

    if ((packet[5] & 0x80) == 0)
       imu_data_.yaw = ((packet[5] << 8) + packet[6]);
    else
       imu_data_.yaw = -32768 + ((packet[5] & 0x7F) << 8) + packet[6];
    imu_data_.yaw *= M_PI/32768.0; // rad

    if ((packet[7] & 0x80) == 0)
        imu_data_.rollrate = ((packet[7] << 8) + packet[8]);
    else
        imu_data_.rollrate = -32768 + ((packet[7] & 0x7F) << 8) + packet[8];
    imu_data_.rollrate *= 1.5*100.0/32768.0*M_PI/180; // rad/sec

    if ((packet[9] & 0x80) == 0)
        imu_data_.pitchrate = ((packet[9] << 8) + packet[10]);
    else
        imu_data_.pitchrate = -32768 +((packet[9] & 0x7F) << 8) + packet[10];
    imu_data_.pitchrate *= 1.5*100.0/32768.0*M_PI/180; // rad/sec

    if ((packet[11] & 0x80) == 0)
       imu_data_.yawrate = ((packet[11] << 8) + packet[12]);
    else
       imu_data_.yawrate = -32768 + ((packet[11] & 0x7F) << 8) + packet[12];
    imu_data_.yawrate *= 1.5*100.0/32768.0*M_PI/180; // rad/sec

    if ((packet[13] & 0x80) == 0)
         imu_data_.ax = ((packet[13] << 8) + packet[14]);
    else
         imu_data_.ax = -32768 +((packet[13] & 0x7F) << 8)+packet[14];
    imu_data_.ax *= 1.5*2.0/32768.0*9.81; // m^2/s

    if ((packet[15] & 0x80) == 0)
         imu_data_.ay = ((packet[15] << 8) + packet[16]);
    else
         imu_data_.ay = -32768 + ((packet[15] & 0x7F) << 8)+packet[16];
    imu_data_.ay *= 1.5*2.0/32768.0*9.81; // m^2/s

    if ((packet[17] & 0x80) == 0)
         imu_data_.az = ((packet[17] << 8) + packet[18]);
    else
         imu_data_.az = -32768 + ((packet[17] & 0x7F) << 8)+packet[18];
    imu_data_.az *= 1.5*2.0/32768.0*9.81; // m^2/s

    if ((packet[19] & 0x80) == 0)
         imu_data_.xmag = ((packet[19] << 8) + packet[20]);
    else
         imu_data_.xmag = -32768 +((packet[19] & 0x7F) << 8)+packet[20];
    imu_data_.xmag *= 1.25*1.5/32768.0; // gauss

    if ((packet[21] & 0x80) == 0)
         imu_data_.ymag = ((packet[21] << 8) + packet[22]);
    else
         imu_data_.ymag = -32768 +((packet[21] & 0x7F) << 8)+packet[22];
    imu_data_.ymag *= 1.25*1.5/32768.0; // gauss

    if ((packet[23] & 0x80) == 0)
         imu_data_.zmag = ((packet[23] << 8) + packet[24]);
    else
         imu_data_.zmag = -32768 +((packet[23] & 0x7F) << 8)+packet[24];
    imu_data_.zmag *= 1.25*1.5/32768.0; // gauss

    imu_data_.boardtemp = (packet[25] <<8) + packet[26];
    imu_data_.boardtemp = (imu_data_.boardtemp*5.0/4096.0-1.375)*44.4;

    imu_data_.counter = (packet[27] <<8) + packet[28];
}


void XBOW4X::parseScaledMode(unsigned char *packet) { //SCALE MODE
    // We're OK and actually have a good packet /w good checksum,
    // decode it, convert the units and update state:
    if ((packet[1] & 0x80) == 0)
        imu_data_.rollrate = ((packet[1] << 8) + packet[2]);
    else
        imu_data_.rollrate = -32768 + ((packet[1] & 0x7F) << 8) + packet[2];
    imu_data_.rollrate *= 1.5*100.0/32768.0*M_PI/180; // rad/sec

    if ((packet[3] & 0x80) == 0)
        imu_data_.pitchrate = ((packet[3] << 8) + packet[4]);
    else
        imu_data_.pitchrate = -32768 +((packet[3] & 0x7F) << 8) + packet[4];
    imu_data_.pitchrate *= 1.5*100.0/32768.0*M_PI/180; // rad/sec

    if ((packet[5] & 0x80) == 0)
       imu_data_.yawrate = ((packet[5] << 8) + packet[6]);
    else
       imu_data_.yawrate = -32768 + ((packet[5] & 0x7F) << 8) + packet[6];
    imu_data_.yawrate *= 1.5*100.0/32768.0*M_PI/180; // rad/sec

    if ((packet[7] & 0x80) == 0)
         imu_data_.ax = ((packet[7] << 8) + packet[8]);
    else
         imu_data_.ax = -32768 +((packet[7] & 0x7F) << 8)+packet[8];
    imu_data_.ax *= 1.5*2.0/32768.0*9.81; // m^2/s

    if ((packet[9] & 0x80) == 0)
         imu_data_.ay = ((packet[9] << 8) + packet[10]);
    else
         imu_data_.ay = -32768 + ((packet[9] & 0x7F) << 8)+packet[10];
    imu_data_.ay *= 1.5*2.0/32768.0*9.81; // m^2/s

    if ((packet[11] & 0x80) == 0)
         imu_data_.az = ((packet[11] << 8) + packet[12]);
    else
         imu_data_.az = -32768 + ((packet[11] & 0x7F) << 8)+packet[12];
    imu_data_.az *= 1.5*2.0/32768.0*9.81; // m^2/s

    if ((packet[13] & 0x80) == 0)
         imu_data_.xmag = ((packet[13] << 8) + packet[14]);
    else
         imu_data_.xmag = -32768 +((packet[13] & 0x7F) << 8)+packet[14];
    imu_data_.xmag *= 1.25*1.5/32768.0; // gauss

    if ((packet[15] & 0x80) == 0)
         imu_data_.ymag = ((packet[15] << 8) + packet[16]);
    else
         imu_data_.ymag = -32768 +((packet[15] & 0x7F) << 8)+packet[16];
    imu_data_.ymag *= 1.25*1.5/32768.0; // gauss

    if ((packet[17] & 0x80) == 0)
         imu_data_.zmag = ((packet[17] << 8) + packet[18]);
    else
         imu_data_.zmag = -32768 +((packet[17] & 0x7F) << 8)+packet[18];
    imu_data_.zmag *= 1.25*1.5/32768.0; // gauss

    imu_data_.boardtemp = (packet[19] <<8) + packet[20];
    imu_data_.boardtemp = (imu_data_.boardtemp*5.0/4096.0-1.375)*44.4;

    imu_data_.counter = (packet[21] <<8) + packet[22];
}


void XBOW4X::parseVoltageMode(unsigned char *packet) { //VOLTAGE MODE

    //TO better results it is necesary the calibration sheet of the device
    imu_data_.rollrate = ((packet[1] << 8) + packet[2]);
    imu_data_.rollrate *= 5/4096.0;
    imu_data_.rollrate *= 100*1.5/4.096*M_PI/180; //rad/sec

    imu_data_.pitchrate = ((packet[3] << 8) + packet[4]);
    imu_data_.pitchrate *= 5/4096.0;
    imu_data_.pitchrate *= 100*1.5/4.096*M_PI/180; //rad/sec

    imu_data_.yawrate = ((packet[5] << 8) + packet[6]);
    imu_data_.yawrate *= 5/4096.0;
    imu_data_.yawrate *= 100*1.5/4.096*M_PI/180; //rad/sec


    imu_data_.ax = ((packet[7] << 8) + packet[8]);
    imu_data_.ax *= 5/4096.0;
    imu_data_.ax = (imu_data_.ax-2.512)*1.01*9.81; // m^2/s

    imu_data_.ay = ((packet[9] << 8) + packet[10]);
    imu_data_.ay *= 5/4096.0;
    imu_data_.ay = (imu_data_.ay-2.512)*1.01*9.81; // m^2/s

    imu_data_.az = ((packet[11] << 8) + packet[12]);
    imu_data_.az *= 5/4096.0;
    imu_data_.az = (imu_data_.az-2.512)*1.01*9.81; // m^2/s

    imu_data_.xmag = ((packet[13] << 8) + packet[14]);
    imu_data_.xmag *= 5/4096.0;
    imu_data_.xmag *= 1.25*1.5/4.096; // gauss

    imu_data_.ymag = ((packet[15] << 8) + packet[16]);
    imu_data_.ymag *= 5/4096.0;
    imu_data_.ymag *= 1.25*1.5/4.096; // gauss

    imu_data_.zmag = ((packet[17] << 8) + packet[18]);
    imu_data_.zmag *= 5/4096.0;
    imu_data_.zmag *= 1.25*1.5/4.096; // gauss

    imu_data_.boardtemp = (packet[19] <<8) + packet[20];
    imu_data_.boardtemp = (imu_data_.boardtemp*5.0/4096.0-1.375)*44.4;

    imu_data_.counter = (packet[21] <<8) + packet[22];
}
