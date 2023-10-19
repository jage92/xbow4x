#include "xbow4x/xbow4x.h"

#define WIN32_LEAN_AND_MEAN
#define _USE_MATH_DEFINES

#define AHRS_ANGLE_MODE_PACKET_SIZE 30
#define AHRS_SCALED_MODE_PACKET_SIZE 24
#define AHRS_VOLTAGE_MODE_PACKET_SIZE 24

using namespace xbow4x;

XBOW4X::XBOW4X(): fd(-1)
{
  read_size_ = AHRS_ANGLE_MODE_PACKET_SIZE;
  reading_status_ = false;
  measurementType = MeasurementType::None;
  messageMode=MessageMode::None;
  calibrationModeEnabled = false;
}

////////////////////////////////////////////////////////////////////////////////
// Open the IMU port
void XBOW4X::openPort(const char *port_name, int baudrate)
{
  closePort(); // In case it was previously open, try to close it first.

  // Open the port
  fd = open(port_name, O_RDWR | O_SYNC | O_NONBLOCK | O_NOCTTY, S_IRUSR | S_IWUSR );
  if (fd < 0)
  {
    const char *extra_msg = "";
    switch (errno)
    {
    case EACCES:
      extra_msg = "You probably don't have premission to open the port for reading and writing.";
      break;
    case ENOENT:
      extra_msg = "The requested port does not exist. Is the IMU connected? Was the port name misspelled?";
      break;
    }

    throw "Unable to open serial port ["+ string(port_name) + "]. "+strerror(errno)+". "+extra_msg;
  }

  // Lock the port
  struct flock fl;
  fl.l_type   = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len   = 0;
  fl.l_pid   = getpid();

  if (fcntl(fd, F_SETLK, &fl) != 0)
    throw "Device "+string(port_name)+" is already locked. Try 'lsof | grep "+string(port_name)+"' to find other processes that currently have the port open.";

  // Change port settings
  struct termios term;
  if (tcgetattr(fd, &term) < 0)
    throw "Unable to get serial port attributes. The port you specified ("+string(port_name)+") may not be a serial port.";

  cfmakeraw( &term );
  if(baudrate == 38400) {
    cfsetispeed(&term, B38400);
    cfsetospeed(&term, B38400);
  }
  else if(baudrate == 115200) {
    cfsetispeed(&term, B115200);
    cfsetospeed(&term, B115200);
  }
  else if(baudrate == 9600) {
    cfsetispeed(&term, B9600);
    cfsetospeed(&term, B9600);
  }
  else {
    throw "Incorrect baudrate";
  }

  if (tcsetattr(fd, TCSAFLUSH, &term) < 0 )
    throw "Unable to set serial port attributes. The port you specified ("+string(port_name)+") may not be a serial port.";

  // Stop continuous mode
  stopContinousReading();

  // Make sure queues are empty before we begin
  if (tcflush(fd, TCIOFLUSH) != 0)
    throw "Tcflush failed. Please report this error if you see it.";
}

////////////////////////////////////////////////////////////////////////////////
// Close the IMU port
void XBOW4X::closePort()
{
  if (fd != -1)
  {
    if (reading_status_)
        stopContinousReading();
    if (close(fd) != 0)
      throw "Unable to close serial port; ["+std::string(strerror(errno))+"]";
    fd = -1;
  }
  measurementType = MeasurementType::None;
}

////////////////////////////////////////////////////////////////////////////////
// Send a packet to the IMU.
// Returns the number of bytes written.
int XBOW4X::send(uint8_t *cmd, int cmd_len)
{
  int bytes;

  bytes = write(fd, cmd, cmd_len);


  if (bytes < 0)
    throw "error writing to IMU ["+string(strerror(errno))+"]";

  if (bytes != cmd_len)
    throw "whole message not written to IMU";

  // Make sure the queue is drained
  // Synchronous IO doesnt always work
  if (tcdrain(fd) != 0)
    throw "tcdrain failed";

  return bytes;
}

int XBOW4X::read_data(uint8_t *rep, uint8_t first_byte, int rep_len, int timeout)
{
  int nbytes, bytes, skippedbytes;
  skippedbytes = 0;

  memset(rep,0,rep_len);

  while (*rep != first_byte && skippedbytes < MAX_BYTES_SKIPPED)
  {
    read_with_timeout(rep, 1, timeout);

    skippedbytes++;
    bytes = 1;
  }

  // Read the rest of the message:
  while (bytes < rep_len)
  {
    nbytes = read_with_timeout(rep + bytes, rep_len - bytes, timeout);

    if (nbytes < 0)
      throw "read failed  ["+string(strerror(errno))+"]";

    bytes += nbytes;
  }
  return bytes;
}


int XBOW4X::read_with_timeout(uint8_t *buff, size_t count, int timeout)
{
  ssize_t nbytes;
  int retval;

  struct pollfd ufd[1];
  ufd[0].fd = fd;
  ufd[0].events = POLLIN;
  //  timeout=0;
  if (timeout == 0)
    timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

  if ( (retval = poll(ufd, 1, timeout)) < 0 )
    throw "poll failed  ["+string(strerror(errno))+"]";

  if (retval == 0)
    throw "timeout reached";

  nbytes = read(fd, buff, count);

  if (nbytes < 0)
    throw "read failed  ["+string(strerror(errno))+"]";

  return nbytes;
}


string XBOW4X::calibrateCommand(uint8_t command) {
  uint8_t buffer[100];
  string s;

  switch (command) {
  case 's':
    if(!calibrationModeEnabled) {
      sendCommand('R');
      send(&command,1);
      read_data(buffer,'S',1);

      if(buffer[0]=='S') {
        calibrationModeEnabled=true;
        return "Calibration Mode started";
      }
      throw "Impossible to communicate with the IMU";
    }
    else
      throw"The calibrating is already enabled";

  case 'u':
    if(calibrationModeEnabled) {
      send(&command,1);

      read_data(buffer,'U',1);

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
      send(&command,1);
      read_data(buffer,'H',1);

      if(buffer[0]=='H') {
        return "Hard iron calibration cleared";
      }
      throw "Impossible to communicate with the IMU";
    }
    else
      throw "The calibrating is enabled";

  case 't':
    if(!calibrationModeEnabled) {
      sendCommand('R');
      send(&command,1);

      read_data(buffer,'T',1);
      //tcflush(fd, TCIFLUSH);

      if(buffer[0]=='T')
        return "Soft iron calibration cleared";
      throw "Impossible to communicate with the IMU";
    }
    else
      throw"The calibrating is enabled";

  default:
    throw "Incorrect command";
    break;
  }
}

MessageMode XBOW4X::getMessageMode() const
{
  return messageMode;
}

MeasurementType XBOW4X::getMeasurementType() const
{
  return measurementType;
}

string XBOW4X::sendCommand(uint8_t command) {
  uint8_t buffer[100];
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
      send(&command,1);
      read_data(buffer,'H',1);

      num=100;
      while(buffer[0]!='H') {

        uint8_t buff ='P';
        send(&buff,1); //STOP Continuous mode

        //PING
        send(&command,1);
        read_data(buffer,'H',1);

        if(!(--num))
          throw"It is not possible to communicate with the IMU";
      }
      return"PING";

    case 'r': //Voltage Mode
      if(reading_status_) {
        sendCommand('R');
        flag=true;
      }
      send(&command,1);
      read_data(buffer,'R',1);

      if(buffer[0]=='R') {
        measurementType = MeasurementType::VoltageMode;
        read_size_ = AHRS_VOLTAGE_MODE_PACKET_SIZE;
        if(flag)
          sendCommand('C');
        return "Voltage Mode enabled";
      }
      else
        throw"It is not possible to communicate with the IMU";

    case 'c': //Scaled Mode
      if(reading_status_) {
        sendCommand('R');
        flag=true;
      }

      send(&command,1);
      read_data(buffer,'C',1);

      if(buffer[0]=='C') {
        measurementType = MeasurementType::ScaledMode;
        read_size_ = AHRS_SCALED_MODE_PACKET_SIZE;
        if(flag)
          sendCommand('C');
        return "Scaled Mode enabled";
      }
      else
        throw"It is not possible to communicate with the IMU";

    case 'a': //Angle Mode
      if(reading_status_) {
        sendCommand('R');
        flag=true;
      }

      send(&command,1);
      read_data(buffer,'A',1);

      if(buffer[0]=='A') {
        measurementType = MeasurementType::AngleMode;
        read_size_ = AHRS_ANGLE_MODE_PACKET_SIZE;

        if(flag)
          sendCommand('C');
        return "Angle Mode enabled";
      }
      else {
        throw "It is not possible to communicate with the IMU";
      }
    case 'P': //Poll Mode
      if(reading_status_) {
        sendCommand('R');
        flag=true;
      }
      send(&command,1);
      //tcflush(fd, TCIFLUSH);
      messageMode = MessageMode::Poll;
      return "Poll Mode enabled";

    case 'C': //Continuous Mode
      send(&command,1);
      if(measurementType == MeasurementType::None) {
        throw "No measurement mode selected";
      }
      startContinuousReading();
      if(reading_status_) {
        return "Starting continuous mode";
      }
      else {
        throw "Imposible to start continuous mode";
      }
    case 'G': //Request Data
      if(reading_status_) {
        sendCommand('R');
      }
      send(&command,1);
      if(measurementType == MeasurementType::None) {
        throw "No measurement mode selected";
      }
      messageMode = MessageMode::Poll;
      //readSerialPort();
      return "Packet read";

    case 'v': //Query DMU Version
      if(!reading_status_) {
        send(&command,1);
        size = read_data(buffer,0xFF,26);
        strncpy(temp,(char *)&buffer[1],size-1);

        if(strlen(temp)==24) {
          return string(temp);
        }
        else
          throw "It is not possible to communicate with the IMU";
      }
      else
        throw "The IMU is in continous mode, stop it to check the version";

    case 'S': //Query Serial Number
      if(!reading_status_) {
        send(&command,1);
        size = read_data(buffer,0xFF,6);
        //tcflush(fd, TCIFLUSH);
        num = int((unsigned char)(buffer[1]) << 24 |
                                                (unsigned char)(buffer[2]) << 16 |
                                                                              (unsigned char)(buffer[3]) << 8 |
                                                                                                            (unsigned char)(buffer[4]));


        if(size == 6)
          return to_string(num);
        else
          throw "It is not possible to communicate with the IMU";

      }
      else
        throw "The IMU is in continous mode, stop it to check the serial number";

    default:
      throw "Incorrect command";
    }
  }
  throw "Calibrating mode is active";
}



void XBOW4X::startContinuousReading() {
  // create thread to read from sensor
  if(measurementType == MeasurementType::None) {
    throw "No MEASUREMENT MODE selected";
  }
  messageMode = MessageMode::Continous;
  reading_status_=true;
}

void XBOW4X::stopContinousReading() {
  reading_status_=false;
  //uint8_t buff ='G';
  uint8_t buff ='P';
  send(&buff,1); //STOP Continuous mode
}

ImuData XBOW4X::readSerialPort() {
  unsigned char buffer[read_size_];
  size_t len;
  int sum;
  string s;

  len = read_data(buffer,0xFF,read_size_);
  //tcflush(fd, TCIFLUSH);
  // imu_data_.receive_time = clock.now();
  sum = 0;
  for(int i = 1; i < int((len - 1)); i++) {
    sum += (int)buffer[i];
  }
  sum = sum % 256;

  // check if we have a complete read and if checksum is correct
  if ((len != read_size_) || (sum != (int)buffer[(len - 1)])) {
    sendCommand('R');
    sendCommand((uint8_t)messageMode);
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

bool XBOW4X::getReadingStatus() {
  return reading_status_;
}
