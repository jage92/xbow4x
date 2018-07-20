#include "xbow6x/xbow6x.h"
using namespace xbow6x;

#define WIN32_LEAN_AND_MEAN 
#define _USE_MATH_DEFINES

#define AHRS_ANGLE_MODE_PACKET_SIZE 30
#define AHRS_SCALED_MODE_PACKET_SIZE 24

#include "boost/date_time/posix_time/posix_time.hpp"

void DefaultProcessData(const ImuData& data) {
    std::cout << "IMU6X Packet:" << std::endl;
    std::cout << " Timestamp: " << data.receive_time;
    std::cout << " IMU Temperature: " << data.boardtemp << std::endl;
    std::cout << " Gyro Yaw: " << data.yawrate << std::endl;
    std::cout << " Gyro Roll: " << data.rollrate << std::endl;
    std::cout << " Gyro Pitch: " << data.pitchrate << std::endl;
    std::cout << " Accel X: " << data.ax << std::endl;
    std::cout << " Accel Y: " << data.ay << std::endl;
    std::cout << " Accel Z: " << data.az << std::endl;
    std::cout << std::endl;
}

XBOW6X::XBOW6X()
{	
	serial_port_ = NULL;
	data_handler_ = DefaultProcessData;
	read_size_ = AHRS_ANGLE_MODE_PACKET_SIZE;
	reading_status_ = false;
}

bool XBOW6X::Connect(std::string port, int baudrate, long timeout) {
	serial_port_ = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(timeout));

	if (!serial_port_->isOpen()){
		std::cout << "Serial port: " << port << " failed to open." << std::endl;
		delete serial_port_;
		serial_port_ = NULL;
		return false;
	} else {
		std::cout << "Serial port: " << port << " opened successfully." << std::endl;
		std::cout << "Searching for IMU..." << std::endl;
	}

	// look for Xbow by sending ping and waiting for response
	if (!Sync()){
		std::cout << "Xbow6x not found on port: " << port << std::endl;
		delete serial_port_;
		serial_port_ = NULL;
		return false;
	}

	// start reading
	std::cout << "Xbow6x found on port: " << port << std::endl;
	StartReading();
	return true;
}

void XBOW6X::Disconnect() {
    std::cout << "Disconnecting DMU." << std::endl;
	StopReading();
	serial_port_->close();
	delete serial_port_;
	serial_port_ = NULL;
}

bool XBOW6X::Sync(int num_attempts) {
	std::string read_data;
	size_t found_ping_response = string::npos;

	// reset the Xbow and wait for a response
	while (num_attempts-- > 0){
		// reset command
		serial_port_->write("R");
		// wait for response
		serial_port_->read(read_data,1000);

		// see if we got a response or a timeout
		found_ping_response = read_data.find("H");
        
		if (found_ping_response != string::npos) {
            found_ping_response = string::npos;
            std::cout << "DMU good reply to reset command: " << read_data << std::endl;
            // ask the DMU for angle mode.
            serial_port_->write("a");
            // wait for response
            serial_port_->read(read_data,1000);            
            // see if we got a ping response or a timeout
            found_ping_response = read_data.find("A");
            if (found_ping_response != string::npos) {
                std::cout << "DMU good reply to angle mode command: " << read_data << std::endl;
                return true;
            }
            std::cout << "DMU bad reply to angle mode command: " << read_data << std::endl;
		}
	}

	// no reponse found
	return false;
}

void XBOW6X::StartReading() {
	// create thread to read from sensor
	reading_status_=true;
	read_thread_ptr_ = boost::shared_ptr<boost::thread > (new boost::thread(boost::bind(&XBOW6X::ReadSerialPort, this)));
}

void XBOW6X::StopReading() {
	reading_status_=false;
    // reset the Xbow
    serial_port_->write("R");
}

void XBOW6X::ReadSerialPort() {
	unsigned char buffer[31];
	int len, sum, i;
    // ask the DMU to send continuous data stream.
    serial_port_->write("C");
    // syncs data stream
	Resync();
    
	while (reading_status_) {
		len = serial_port_->read(buffer, read_size_);
        // time stamp
		imu_data_.receive_time = ros::Time::now();
//         std::cout << "Read data: ";
//         for(i = 0; i < len; i++)
//             std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i] << " ";
//         std::cout << std::endl;
        sum = 0;
        for(i = 1; i < (len - 1); i++) {
            sum += (int)buffer[i];
        }
        sum = sum % 256;

		// check if we have a complete read and if checksum is correct
		if ((len != read_size_) || (sum != (int)buffer[(len - 1)])) {
            // display data
            std::cout << "Read data: ";
            for(i = 0; i < len; i++)
                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i] << " ";
            std::cout << std::endl;
            
            std::cout <<"Computed checksum: " << sum << " Data checksum: " << (int)buffer[(len - 1)] << std::endl;
			Resync();
			continue;
		}

		// parse packet
		Parse(buffer);
	}

}

void XBOW6X::Resync() {
	std::string result = "";
	size_t found = string::npos;
	bool synced = false;
	unsigned char data[31];
    std::string end_string(1, 0xFF);
    int i, ii, sum;

	// make up to 5 attempts to resync
	for(ii = 0; ii < 5; ii++){
		std::cout << "Resyncing.  Attempt # " << (ii + 1) << " of 5." << std::endl;
        // finds first 0xFF
		result = serial_port_->readline(1000, end_string);
		found = result.find(end_string);
        // if 0xFF found
		if (found != string::npos){
			sum = 0;
			size_t len = serial_port_->read(data, read_size_-1);
            // compute checksum
            for (int i = 0; i < read_size_-2; i++) {
                sum += (int)data[i];
            }
            sum = sum % 256;
            
			if (sum == (int)data[read_size_-2] && len == read_size_-1) {
				synced = true;
				break;
			}
		}
	}

	if (synced) {
		std::cout << "Successfully synchronized." << std::endl;
	} else {
		std::cout << "Failed to synchronize. Stopping sensor." << std::endl;
		Disconnect();
	}
}

void XBOW6X::Parse(unsigned char *packet) {
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

    // call callback with data
    if (data_handler_ != NULL)
    	data_handler_(imu_data_);
    
}
