#include "xbow4x/xbow4x.h"
using namespace xbow4x;

#define WIN32_LEAN_AND_MEAN 
#define _USE_MATH_DEFINES

#define AHRS_ANGLE_MODE_PACKET_SIZE 30
#define AHRS_SCALED_MODE_PACKET_SIZE 24
#define AHRS_VOLTAGE_MODE_PACKET_SIZE 24


XBOW4X::XBOW4X()
{	
  serial_port_ = NULL;
  read_size_ = AHRS_ANGLE_MODE_PACKET_SIZE;
  reading_status_ = false;
  measurementType = MeasurementType::None;
  messageMode=MessageMode::None;
  calibrationModeEnabled = false;
}

void XBOW4X::connect(std::string port, int baudrate, long timeout) {
  serial_port_ = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(timeout));
  u_int8_t buffer[100];

  this->port=port;
  this->baudrate=baudrate;
  this->timeout=timeout;

  if (!serial_port_->isOpen()){
    ROS_DEBUG("Serial port: %s failed to open.", port.c_str());
    delete serial_port_;
    serial_port_ = NULL;
    throw "Serial port: %s failed to open" + port;
  }
  else {
    ROS_INFO("Serial port: %s opened successfully.", port.c_str());
    ROS_INFO("Searching for IMU...");

    //Cleanning the buffer
    serial_port_->read(buffer,99);
  }
}

void XBOW4X::disconnect() {
  ROS_INFO("Disconnecting DMU.");
  stopContinousReading();
  serial_port_->close();
  delete serial_port_;
  serial_port_ = NULL;
  measurementType = MeasurementType::None;
}

string XBOW4X::calibrateCommand(u_int8_t command) {
  u_int8_t buffer[100];
  int size;
  string s;

  switch (command) {
  case 's':
    if(!calibrationModeEnabled) {
      sendCommand('R');
      ROS_DEBUG("Command %c",command);
      serial_port_->write(&command,1);
      size = serial_port_->read(buffer,1);
      ROS_DEBUG("Response %d",buffer[0]);
      if(buffer[0]=='S') {
        calibrationModeEnabled=true;
        return "Calibration Mode started";
      }
      throw "Impossible to communicate with the IMU";
    }
    else
      throw "The calibrating is already enabled";
  case 'u':
    if(calibrationModeEnabled) {
      serial_port_->write(&command,1);
      ROS_DEBUG("Command %c",command);
      size = serial_port_->read(buffer,1);
      ROS_DEBUG("Response %d",buffer[0]);
      if(buffer[0]=='U') {
        calibrationModeEnabled=false;
        return "Calibration Mode stopped";
      }
      throw "Impossible to communicate with the IMU";
    }
    else
      throw "The calibrating mode is not enabled";

  case 'h':
    if(!calibrationModeEnabled) {
      sendCommand('R');
      serial_port_->write(&command,1);
      ROS_DEBUG("Command %c",command);
      size = serial_port_->read(buffer,1);
      ROS_DEBUG("Response %d",buffer[0]);
      if(buffer[0]=='H')
        return "Hard iron calibration cleared";
      throw "Impossible to communicate with the IMU";
    }
    else
      throw "The calibrating is enabled";

  case 't':
    if(!calibrationModeEnabled) {
      sendCommand('R');
      serial_port_->write(&command,1);
      ROS_DEBUG("Command %c",command);
      size = serial_port_->read(buffer,1);
      ROS_DEBUG("Response %d",buffer[0]);
      if(buffer[0]=='T')
        return "Soft iron calibration cleared";
      throw "Impossible to communicate with the IMU";
    }
    else
      throw "The calibrating is enabled";

  default:
    throw "Incorrect command";
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

string XBOW4X::setBaudrate(u_int32_t baudrate) {

  string s;
  u_int8_t command;
  u_int8_t buffer[100];
  int previousBaudrate;
  sendCommand('R');

  command = 'b';
  serial_port_->write(&command,1);
  ROS_DEBUG("Command %c",command);
  serial_port_->read(buffer,1);
  ROS_DEBUG("Response %c",buffer[0]);

  if(buffer[0]=='B') {
    previousBaudrate = this->baudrate;
    this->baudrate=baudrate;
    reopenSerialPort();

    command = 'a';
    serial_port_->write(&command,1);
    ROS_DEBUG("Command %c",command);
    serial_port_->read(buffer,1);
    ROS_DEBUG("Response %c",buffer[0]);

    if(buffer[0]=='A') {
      sendCommand('R');
      sendCommand((u_int8_t)measurementType);
      sendCommand((u_int8_t)messageMode);
      return "Baudrate changed";
    }
    else {
      this->baudrate=previousBaudrate;
      reopenSerialPort();
      throw "It is not possible to set a new baudrate";
    }
  }
  else
    throw "Impossible to communicate with the IMU";
}

string XBOW4X::sendCommand(u_int8_t command) {
  u_int8_t buffer[100];
  char temp[25];
  u_int8_t c;
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
      ROS_DEBUG("Command %c",command);
      serial_port_->write(&command,1);
      size = serial_port_->read(buffer,1);
      ROS_DEBUG("Response %d",buffer[0]);
      //measurementType = MeasurementType::None;
      num=100;
      while(buffer[0]!='H') {
        //SEARCH PING
        ROS_DEBUG("Command %c",command);
        serial_port_->write("G"); //STOP Continuous mode
        //            messageMode = MessageMode::Poll;
        reopenSerialPort();
        ROS_DEBUG("Command %c",command);
        //PING
        serial_port_->write(&command,1);
        //sleep(1);
        size = serial_port_->read(buffer,1);
        ROS_DEBUG("Response %d",buffer[0]);
        if(!(--num))
          throw "It is not possible to communicate with the IMU";
      }
      return "PING";
    case 'r': //Voltage Mode
      if(reading_status_) {
        sendCommand('R');
        flag=true;
      }
      ROS_DEBUG("Command %c",command);
      serial_port_->write(&command,1);
      size = serial_port_->read(buffer,1);
      ROS_DEBUG("Response %c",buffer[0]);
      if(buffer[0]=='R') {
        measurementType = MeasurementType::VoltageMode;
        read_size_ = AHRS_VOLTAGE_MODE_PACKET_SIZE;
        if(flag)
          sendCommand('C');
        return "Voltage Mode enabled";
      }
      else
        throw "It is not possible to communicate with the IMU";

    case 'c': //Scaled Mode
      if(reading_status_) {
        sendCommand('R');
        flag=true;
      }
      ROS_DEBUG("Command %c",command);
      serial_port_->write(&command,1);
      size = serial_port_->read(buffer,1);
      ROS_DEBUG("Response %c",buffer[0]);
      if(buffer[0]=='C') {
        measurementType = MeasurementType::ScaledMode;
        read_size_ = AHRS_SCALED_MODE_PACKET_SIZE;

        if(flag)
          sendCommand('C');
        return "Scaled Mode enabled";
      }
      else
        throw "It is not possible to communicate with the IMU";

    case 'a': //Angle Mode
      if(reading_status_) {
        sendCommand('R');
        flag=true;
      }
      ROS_DEBUG("Command %c",command);
      serial_port_->write(&command,1);
      size = serial_port_->read(buffer,1);
      ROS_DEBUG("Response %c",buffer[0]);
      if(buffer[0]=='A') {
        measurementType = MeasurementType::AngleMode;
        read_size_ = AHRS_ANGLE_MODE_PACKET_SIZE;
        ROS_DEBUG("Angle Mode enabled");
        if(flag)
          sendCommand('C');
        return "Angle Mode enabled";
      }
      else {
        ROS_ERROR("It is not possible to communicate with the IMU");
        throw "It is not possible to communicate with the IMU";
      }
    case 'P': //Poll Mode
      if(reading_status_) {
        sendCommand('R');
        flag=true;
      }
      ROS_DEBUG("Command %c",command);
      serial_port_->write(&command,1);
      messageMode = MessageMode::Poll;
      return "Poll Mode enabled";
    case 'C': //Continuous Mode
      ROS_DEBUG("Command %c",command);
      serial_port_->write(&command,1);
      if(measurementType == MeasurementType::None) {
        throw "No measurement mode selected";
      }
      if(startContinuousReading()) {
        ROS_DEBUG("Starting continuous mode");
        return "Starting continuous mode";
      }
      else {
        ROS_DEBUG("Imposible to start continuous mode");
        return "Imposible to start continuous mode";
      }
    case 'G': //Request Data
      if(reading_status_) {
        sendCommand('R');
      }
      ROS_DEBUG("Command %c",command);
      serial_port_->write(&command,1);
      if(measurementType == MeasurementType::None)
        return"No measurement mode selected";

      messageMode = MessageMode::Poll;
      readSerialPort();
      return "Packet read";

    case 'v': //Query DMU Version
      if(!reading_status_) {
        ROS_DEBUG("Command %c",command);
        serial_port_->write(&command,1);
        ROS_DEBUG("V1");
        size = serial_port_->read(buffer,26);

        strncpy(temp,(char *)&buffer[1],size-1);
        ROS_INFO("DMU Version: %s",temp);

        if(strlen(temp)==24) {
          ROS_DEBUG("V4");
          return string(temp);
        }
        else
          throw "It is not possible to communicate with the IMU";
      }
      else
        throw "The IMU is in continous mode, stop it to check the version";

    case 'S': //Query Serial Number
      if(!reading_status_) {
        serial_port_->write(&command,1);
        size = serial_port_->read(buffer,6);
        num = int((unsigned char)(buffer[1]) << 24 |
                                                (unsigned char)(buffer[2]) << 16 |
                                                                              (unsigned char)(buffer[3]) << 8 |
                                                                                                            (unsigned char)(buffer[4]));
        ROS_DEBUG("Response %d",num);
        if(size == 6)
          return to_string(num);
        else
          throw "It is not possible to communicate with the IMU";
      }
      else
        return "The IMU is in continous mode, stop it to check the serial number";
    default:
      throw "Incorrect command";
    }
  }
  return "Calibrating mode is active";
}



bool XBOW4X::startContinuousReading() {
  // create thread to read from sensor
  if(measurementType == MeasurementType::None) {
    ROS_ERROR("No MEASUREMENT MODE selected");
    return false;
  }
  messageMode = MessageMode::Continous;
  reading_status_=true;

  return true;
}

void XBOW4X::stopContinousReading() {
  reading_status_=false;
  serial_port_->write("G"); //STOP Continuous mode
  //  messageMode = MessageMode::Poll;
}

ImuData xbow4x::XBOW4X::readSerialPort() {
  unsigned char buffer[read_size_];
  int len, sum, i;
  string s;

  //  if(!serialPort_enabled) //Because concurrency
  //    return;
  len = serial_port_->read(buffer, read_size_);
  imu_data_.receive_time = ros::Time::now();
  sum = 0;
  for(int i = 1; i < (len - 1); i++) {
    sum += (int)buffer[i];
  }
  sum = sum % 256;

  // check if we have a complete read and if checksum is correct
  if ((len != read_size_) || (sum != (int)buffer[(len - 1)])) {
    // display data
    ROS_DEBUG("Read data: %d",len);
    //    for(i = 0; i < len; i++)
    //        ROS_DEBUG("%d", (int)buffer[i]);
    //    ROS_DEBUG("");
    ROS_DEBUG("Computed checksum: %d Data checksum: %d",sum, (int)buffer[(len - 1)]);

    s = sendCommand('R');
    ROS_DEBUG("%s",s.c_str());
    s = sendCommand((u_int8_t)messageMode);
    ROS_DEBUG("%s",s.c_str());
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
  imu_data_.ax *= -1.5*2.0/32768.0*9.81; // m^2/s

  if ((packet[15] & 0x80) == 0)
    imu_data_.ay = ((packet[15] << 8) + packet[16]);
  else
    imu_data_.ay = -32768 + ((packet[15] & 0x7F) << 8)+packet[16];
  imu_data_.ay *= -1.5*2.0/32768.0*9.81; // m^2/s

  if ((packet[17] & 0x80) == 0)
    imu_data_.az = ((packet[17] << 8) + packet[18]);
  else
    imu_data_.az = -32768 + ((packet[17] & 0x7F) << 8)+packet[18];
  imu_data_.az *= -1.5*2.0/32768.0*9.81; // m^2/s

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
  imu_data_.ax *= -1.5*2.0/32768.0*9.81; // m^2/s //Negative ROS Convention

  if ((packet[9] & 0x80) == 0)
    imu_data_.ay = ((packet[9] << 8) + packet[10]);
  else
    imu_data_.ay = -32768 + ((packet[9] & 0x7F) << 8)+packet[10];
  imu_data_.ay *= -1.5*2.0/32768.0*9.81; // m^2/s //Negative ROS Convention

  if ((packet[11] & 0x80) == 0)
    imu_data_.az = ((packet[11] << 8) + packet[12]);
  else
    imu_data_.az = -32768 + ((packet[11] & 0x7F) << 8)+packet[12];
  imu_data_.az *= -1.5*2.0/32768.0*9.81; // m^2/s //Negative ROS Convention

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
  imu_data_.ax = -(imu_data_.ax-2.512)*1.01*9.81; // m^2/s //Negative ROS Convention

  imu_data_.ay = ((packet[9] << 8) + packet[10]);
  imu_data_.ay *= 5/4096.0;
  imu_data_.ay = -(imu_data_.ay-2.512)*1.01*9.81; // m^2/s //Negative ROS Convention

  imu_data_.az = ((packet[11] << 8) + packet[12]);
  imu_data_.az *= 5/4096.0;
  imu_data_.az = -(imu_data_.az-2.512)*1.01*9.81; // m^2/s //Negative ROS Convention

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
