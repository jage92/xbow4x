#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <signal.h>

#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"


#include "xbow4x/broadcast_tf.h"
#include "xbow4x/send_command.h"
#include "xbow4x/status.h"
#include "xbow4x/calibrate.h"
#include "xbow4x/set_baudrate.h"
#include "xbow4x/EulerStamped.h"

#include "xbow4x/xbow4x.h"
using namespace xbow4x;
using std::string;

#define X 0 //! Index of the first element of array that corresponds to x component of a point
#define Y 1 //! Index of the first element of array that corresponds to y component of a point
#define Z 2 //! Index of the first element of array that corresponds to z component of a point

class Xbow4xNode
{
private:
  self_test::TestRunner self_test_; //! Object to test the IMU
  diagnostic_updater::Updater diagnostic_; //! Object that allows the IMU diagnostic
  int error_count_; //! Counter of IMU errors
  std::string error_status_; //! Message of the error status

  double desired_freq_; //! Working frecuency of the IMU to pass to self_test
  std::unique_ptr<diagnostic_updater::FrequencyStatus> freq_diag_; //! Self_test object to monitoring the IMU diagnosis

  ros::Publisher imu_pub; //! Publisher object of the IMU data (angular rate, acceleration, orientation)
  ros::Publisher mag_pub; //! Publisher object of the magnetometer (magnetic field vector)
  ros::Publisher imu_no_grab_pub; //! Publisher object of the IMU data (angular rate whiout gravity, acceleration, orientation)
  ros::Publisher euler_pub; //! Publisher of the euler angles

  string frame_id; //! Name of the IMU frame
  string base_frame; //! Name of the BASE frame
  sensor_msgs::Imu imu_msg, imu_msg_no_grab; //! Message to public the IMU topic
  sensor_msgs::MagneticField msgmag; //! Message to public the Magnetometec topic
  xbow4x::EulerStamped eulerMsg; //! Message type for the orientation data (Euler Stamped)


  double ang_vel_mean[3]={0.0,0.0,0.0}; //! Array of the algular rate bias
  double lin_acc_mean_no_grab[3]={0.0,0.0,0.0}; //! Array of the linear acceleration whiout gravity

  bool is_initialized=false; //! Flag that indicates if the xbow is initialized
  bool imu_started=false; //! Flag that indicates if the xbow is strated
  int counter=0;
  ros::Timer timer; //! Timer to calculate de angular rate and the acceleration gravity
  bool broadcats_tf; //! Flag that indicates if the IMU reference frame is published or not
  xbow4x::MeasurementType measurementMode; //! Enum object of the IMU measurement type (AngleMode, ScaledMode, VoltageMode) active
  xbow4x::MessageMode messageMode; //! Enum object of the IMU message mode (Continous or Poll) active

  ros::NodeHandle node_handle; //! NodeHandle

  //Service declaration
  ros::ServiceServer serviceTf; //! ROS service to enable or diseble the imu tf broadcast
  ros::ServiceServer serviceStatus; //! ROS service that return the IMU status
  ros::ServiceServer serviceCommand; //!ROS service to send commands to the IMU
  ros::ServiceServer serviceCalibrate; //!ROS service to calibrate the IMU
  ros::ServiceServer serviceSetBaudrate; //!ROS service to change the baudrate of the IMU
  ros::ServiceServer start_calibrate_serv; //! Service to start magnetometer calibration
  ros::ServiceServer stop_calibrate_serv; //! Service to stop magnetometer calibration
  ros::ServiceServer poll_serv; //! Service to change the node to poll mode
  ros::ServiceServer continuous_serv; //!Service to change the node to continuous mode

  double temperature = 0.0;

  bool calibrated=false;
  bool calibrating=false;
  XBOW4X xbow;
  tf2::Quaternion q_rot;

  bool use_enu_frame; //! Determines if the ENU frame is used or not
  bool euler; //! Publish euler data in euler topic
  int baudrate;
  std::vector<double> tf_translation; //! Vector of the traslation vector of the TF translation matrix
  string port_name; //! Name of the device port
  ros::WallTimer wallTimer; //! Walltimer to read the IMU contiunosly
  bool timerActive; //! Wall timer was active when the calibration starts

public:
  ~Xbow4xNode() {
    ROS_INFO("Serial Port Disconnected");

    xbow.closePort();
  }

  Xbow4xNode() : node_handle("xbow4x_node"),
    self_test_(),
    diagnostic_(),
    error_count_(0)
  {
    string s;

    imu_pub = node_handle.advertise<sensor_msgs::Imu>("imu/data", 1);
    mag_pub = node_handle.advertise<sensor_msgs::MagneticField>("mag", 1);


    if(!node_handle.getParam("port", port_name))
      port_name = string("/dev/crossbow");

    if(!node_handle.getParam("baudrate", baudrate))//38400
      baudrate = 38400;

    if(!node_handle.getParam("broadcast_tf", broadcats_tf))
      broadcats_tf = true;

    if(!node_handle.getParam("use_enu_frame", use_enu_frame))
      use_enu_frame = false;

    if(!node_handle.getParam("frame_id", frame_id))
      frame_id = string("imu");

    if(!node_handle.getParam("base_frame", base_frame))
      base_frame = string("base");

    //Angle Mode: a Scaled Mode: c Voltage Mode: r
    if(node_handle.getParam("measurement_mode",s)) {//AngleMode
      measurementMode =static_cast<xbow4x::MeasurementType>(s.c_str()[0]);
      ROS_INFO("%s",s.c_str());
    }
    else
      measurementMode = xbow4x::MeasurementType::AngleMode;

    if(measurementMode == xbow4x::MeasurementType::AngleMode)
      imu_no_grab_pub = node_handle.advertise<sensor_msgs::Imu>("imu/data_no_grab", 1);

    //Continuous Mode: C Poll Mode: P Poll/Stop Continuous Mode: G
    if(node_handle.getParam("message_mode", s)) { //Continuous Mode
      ROS_INFO("%s",s.c_str());
      messageMode =static_cast<xbow4x::MessageMode>(s.c_str()[0]);
    }
    else
      messageMode = xbow4x::MessageMode::Continous;

    //Continuous Mode: C Poll Mode: P Poll/Stop Continuous Mode: G
    if(!node_handle.getParam("euler", euler)) { //Continuous Mode
      euler = false;
    }

    if(euler)
      euler_pub = node_handle.advertise<xbow4x::EulerStamped>("euler", 1);


    if(!node_handle.getParam("tf_translation",tf_translation))
      tf_translation =  {2.0, 2.0, 0.0};

    std::vector<double> orientation_cov;
    if(!node_handle.getParam("orientation_cov", orientation_cov))
      orientation_cov = {1.0e6,  0.0,    0.0,
                         0.0, 1.0e6, 0.0,
                         0.0, 0.0, 1.0e6};

    std::copy_n(orientation_cov.begin(),orientation_cov.size(),imu_msg.orientation_covariance.begin());
    std::copy_n(orientation_cov.begin(),orientation_cov.size(),imu_msg_no_grab.orientation_covariance.begin());

    std::vector<double> ang_vel_cov;
    if(!node_handle.getParam("ang_vel_cov", ang_vel_cov))
      ang_vel_cov = {1.0e-2, 0.0,    0.0,
                     0.0,    1.0e-2, 0.0,
                     0.0,    0.0,    1.0e-2};

    std::copy_n(ang_vel_cov.begin(),ang_vel_cov.size(),imu_msg.angular_velocity_covariance.begin());
    std::copy_n(ang_vel_cov.begin(),ang_vel_cov.size(),imu_msg_no_grab.angular_velocity_covariance.begin());

    std::vector<double> lin_acc_cov;
    if(!node_handle.getParam("lin_acc_cov", lin_acc_cov))
      lin_acc_cov = {1.0e-2, 0.0,    0.0,
                     0.0,    1.0e-2, 0.0,
                     0.0,    0.0,    1.0e-2};

    std::copy_n(lin_acc_cov.begin(),lin_acc_cov.size(),imu_msg.linear_acceleration_covariance.begin());

    std::vector<double> lin_acc_cov_no_grab;
    if(!node_handle.getParam("lin_acc_cov_no_grab", lin_acc_cov_no_grab))
      lin_acc_cov_no_grab = {1.0e-2, 0.0,    0.0,
                             0.0,    1.0e-2, 0.0,
                             0.0,    0.0,    1.0e-2};

    std::copy_n(lin_acc_cov_no_grab.begin(),lin_acc_cov_no_grab.size(),imu_msg_no_grab.linear_acceleration_covariance.begin());

    std::vector<double> mag_cov;
    if(!node_handle.getParam("mag_cov", mag_cov))
      mag_cov = {1.0e-2, 0.0,    0.0,
                 0.0,    1.0e-2, 0.0,
                 0.0,    0.0,    1.0e-2};

    std::copy_n(mag_cov.begin(),mag_cov.size(),msgmag.magnetic_field_covariance.begin());


    //Service declaration
    serviceTf = node_handle.advertiseService("broadcast_tf", &Xbow4xNode::broadcatsTf,this);
    serviceStatus = node_handle.advertiseService("status",&Xbow4xNode::statusSrv,this);
    serviceCommand = node_handle.advertiseService("send_command", &Xbow4xNode::sendCommand,this);
    serviceCalibrate = node_handle.advertiseService("calibrateCommand", &Xbow4xNode::calibrateSrv,this);
    poll_serv = node_handle.advertiseService("poll",&Xbow4xNode::poll, this);
    continuous_serv = node_handle.advertiseService("continuous",&Xbow4xNode::continuous, this);
    start_calibrate_serv = node_handle.advertiseService("start_calibrate",&Xbow4xNode::start_calibrate, this);
    stop_calibrate_serv = node_handle.advertiseService("stop_calibrate", &Xbow4xNode::stop_calibrate, this);
    timer = node_handle.createTimer(ros::Duration(10), &Xbow4xNode::timerCallback,this,false,false);

    q_rot.setRPY(M_PI,0,M_PI/2); //ENU
    q_rot.normalize();

    wallTimer = node_handle.createWallTimer(ros::WallDuration(0),&Xbow4xNode::wallTimerCallback,this,false,false);

    diagnostic_.add( "Bias Status", this, &Xbow4xNode::biasStatus);
    diagnostic_.add( "IMU Status", this, &Xbow4xNode::deviceStatus);
    diagnostic_.add( "Calibration Status", this, &Xbow4xNode::calibrationStatus);
    setWorkFrecuency();

    self_test_.add("Interruption Test", this, &Xbow4xNode::interruptionTest);
    self_test_.add("Connect Test", this, &Xbow4xNode::connectTest);
    self_test_.add("Read ID Test", this, &Xbow4xNode::readIDTest);
    self_test_.add("Gyro Bias Test", this, &Xbow4xNode::biasTest);
    self_test_.add("Streamed Data Test", this, &Xbow4xNode::streamedDataTest);
    self_test_.add("Gravity Test", this, &Xbow4xNode::gravityTest);
    self_test_.add("Disconnect Test", this, &Xbow4xNode::disconnectTest);
    self_test_.add("Resume Test", this, &Xbow4xNode::resumeTest);

    try {
      xbow.openPort(port_name.c_str(), baudrate);

      xbow.sendCommand('R');

      diagnostic_.setHardwareID(getID());

      xbow.sendCommand((u_int8_t)measurementMode);

      //First in continous mode to calculate de bias
      xbow.sendCommand('C');
      wallTimer.start();
    }
    catch(const char* error) {
      ROS_ERROR("%s",error);
      error_count_++;
      diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU ERROR: "+string(error));
      node_handle.shutdown();
      exit(1);
    }

  }

  /**
   * @brief setWorkFrecuency Set the work frecuency depending of the Measurement Type
   * The IMU works at different fecuencies depending of the activated mode. The documentation said that the IMU work at 60Hz
   *
   */
  void setWorkFrecuency() {
    switch (measurementMode) {
    case xbow4x::MeasurementType::AngleMode:
      desired_freq_=60.0;
      break;
    case xbow4x::MeasurementType::ScaledMode:
      desired_freq_=120.0;
      break;
    case xbow4x::MeasurementType::VoltageMode:
      desired_freq_=165.0;
      break;
    default:
      desired_freq_=165.0;
      break;
    }
    diagnostic_.removeByName("Frequency Status");
    freq_diag_ = std::make_unique<diagnostic_updater::FrequencyStatus>(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05));
    freq_diag_->clear();
    diagnostic_.add( *(freq_diag_.get()));
  }

  /**
   * @brief getID Obtains the sensor information (dmu_number and serial number)
   * @return device info string
   */
  string getID() {

    string dmu_ver;
    string serial_number;
    string id;

    try {
      dmu_ver = xbow.sendCommand('v');
      serial_number = xbow.sendCommand('S');
      id = dmu_ver + "_" + serial_number;

      ROS_INFO("Connected to IMU %s",id.c_str());

      return id;
    }
    catch(const char* error) {
      ROS_ERROR("%s",error);
      error_count_++;
      diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN, "IMU ERROR:: "+string(error));
    }
    return "";
  }

  /**
   * @brief Callback that reads the data stream from de IMU sensor
   * @return
   */
  void wallTimerCallback(const ros::WallTimerEvent& event) {
    publishImuData();

    self_test_.checkTest();
    diagnostic_.update();
    freq_diag_->tick();
  }

  /**
   * @brief InterruptionTest Test if the serial port has other connections
   * @param status node status
   */
  void interruptionTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (imu_pub.getNumSubscribers() > 0 || mag_pub.getNumSubscribers() > 0 || imu_no_grab_pub.getNumSubscribers() > 0 || (euler_pub != nullptr && euler_pub.getNumSubscribers() > 0))
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Interruption Text: There were active subscribers.");
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "No operation interrupted.");
  }

  /**
   * @brief ConnectTest Test if the connection throght the serial port is possible
   * @param status node status
   */
  void connectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    try {
      xbow.openPort(port_name.c_str(), baudrate);
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Connected successfully.");
    }
    catch(const char* error) {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "It is not possible to connect: "+string(error));
    }
  }

  /**
   * @brief ReadIDTest Test if it is possible to read the IMU info (Model, Serial number, ...)
   * @param status node status
   */
  void readIDTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    string id;

    try{
      xbow.sendCommand('R');
      id=getID();

      if(id.length()!=32) {
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Imposible to read IMU ID");
      }
      else {
        self_test_.setID(id);
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Read id Successfully");
      }
    }
    catch(const char* error) {
      ROS_ERROR("%s",error);
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not read de IMU ID: "+string(error));
    }
  }

  /**
   * @brief StreamedDataTest Test if the continuous mode of the IMU works correctly
   * @param status node status
   */
  void streamedDataTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    double angularRate[3]={0.0, 0.0, 0.0},accel[3]={0.0, 0.0, 0.0},magnetic[3]={0.0, 0.0, 0.0};
    int it=200;

    try {
      xbow.sendCommand('R');
      xbow.sendCommand((u_int8_t)measurementMode);
      xbow.sendCommand('C');
    }
    catch(const char* error) {
      ROS_ERROR("%s",error);
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
    }

    try {
      for(int i=0;i<it;i++){
        xbow4x::ImuData data = xbow.readSerialPort();

        angularRate[X]+=data.rollrate;
        angularRate[Y]+=data.pitchrate;
        angularRate[Z]+=data.yawrate;

        accel[X]+=data.ax;
        accel[Y]+=data.ay;
        accel[Z]+=data.az;

        magnetic[X]+=data.xmag;
        magnetic[Y]+=data.ymag;
        magnetic[Z]+=data.zmag;

      }
    }
    catch(const char* error) {
      ROS_ERROR("%s",error);
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
    }
    angularRate[X]/=it;
    angularRate[Y]/=it;
    angularRate[Z]/=it;

    accel[X]/=it;
    accel[Y]/=it;
    accel[Z]/=it;

    magnetic[X]/=it;
    magnetic[Y]/=it;
    magnetic[Z]/=it;
    try {
      xbow.sendCommand('P');
    }
    catch(const char* error) {
      ROS_ERROR("%s",error);
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
    }

    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Data streamed successfully.");

    status.add("Agular Rate X",angularRate[X]);
    status.add("Agular Rate Y",angularRate[Y]);
    status.add("Agular Rate Z",angularRate[Z]);

    status.add("Acceleration X",accel[X]);
    status.add("Acceleration Y",accel[Y]);
    status.add("Acceleration Z",accel[Z]);

    status.add("Magnetic Field X",accel[X]);
    status.add("Magnetic Field Y",accel[Y]);
    status.add("Magnetic Field Z",accel[Z]);
  }


  /**
   * @brief GyroBiasTest Test if it is possible to set the Giro Biass
   * @param status node status
   */
  void biasTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    ros::Time start_time;


    is_initialized=false;
    imu_started=false;

    try {
      xbow.sendCommand('C');
    }
    catch(const char* error) {
      ROS_ERROR("%s",error);
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
    }
    start_time = ros::Time::now();
    while(ros::Time::now() - start_time < ros::Duration(10.0)){
      publishImuData();
    }
    timerCallback(ros::TimerEvent());
    try{
      xbow.sendCommand('P');

      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Successfully calculated gyro biases.");

      status.add("Gyro Bias X",ang_vel_mean[X]);
      status.add("Gyro Bias Y", ang_vel_mean[Y]);
      status.add("Gyro Bias Z", ang_vel_mean[Z]);

      status.add("Linear Accel Bias X",lin_acc_mean_no_grab[X]);
      status.add("Linear Accel Bias Y", lin_acc_mean_no_grab[Y]);
      status.add("Linear Accel Bias Z", lin_acc_mean_no_grab[Z]);
    }
    catch(const char* error) {
      ROS_ERROR("%s",error);
      status.summary(2, "Could not start streaming data: "+string(error));
    }
  }

  /**
   * @brief GravityTest Test if the imu acceleration works correctly
   * @param status node status
   */
  void gravityTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {

    double grav = 0.0;

    double grav_x = 0.0;
    double grav_y = 0.0;
    double grav_z = 0.0;

    try {
      xbow.sendCommand('C');
      int num = 200;
      for (int i = 0; i < num; i++)
      {
        xbow4x::ImuData data = xbow.readSerialPort();

        grav_x += data.ax;
        grav_y += data.ay;
        grav_z += data.az;
      }
      xbow.sendCommand('P');

      grav += sqrt( pow(grav_x / (double)(num), 2.0) +
                    pow(grav_y / (double)(num), 2.0) +
                    pow(grav_z / (double)(num), 2.0));

      //      double err = (grav - microstrain_3dmgx2_imu::G);
      double err = (grav - 9.81);

      if (fabs(err) < .05)
      {
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Gravity detected correctly.");
      } else {
        status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Measured gravity deviates by %f", err);
      }

      status.add("Measured gravity", grav);
      status.add("Gravity error", err);
    }
    catch(const char* error) {
      ROS_ERROR("%s",error);
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not be done the gravity test: "+string(error));
    }
  }


  /**
   * @brief DisconnectTest Test if the IMU is disconnected correctly
   * @param status
   */
  void disconnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    xbow.closePort();

    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Disconnected successfully.");
  }


  /**
   * @brief ResumeTest Test that check that, after disconnect the imu, it is possible to reconnect it again
   * @param status node status
   */
  void resumeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (is_initialized)
    {
      try {
        xbow.openPort(port_name.c_str(), baudrate);
        xbow.sendCommand('R');
        xbow.sendCommand((u_int8_t)measurementMode);
        xbow.sendCommand((u_int8_t)messageMode);
        freq_diag_->clear();
      }
      catch(const char *error) {
        ROS_ERROR("%s",error);
        status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not be done the resume test.");
      }

    }

    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Previous operation resumed successfully.");
  }

  /**
    * @brief start_calibrate Service to start the magnetometer calibration
    * @param req Request object
    * @param resp Response object
    * @return True if the service is running correctly or false if not
    */
   bool start_calibrate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {

     if(is_initialized) {
       try {
         calibrated=false;
         wallTimer.stop();
         ROS_INFO("Clearing hard iron offset");
         xbow.calibrateCommand('h');
         ROS_INFO("Clearing soft iron offset");
         xbow.calibrateCommand('t');
         ROS_INFO("Starting magnetometer calibration");
         xbow.calibrateCommand('s');
         calibrating=true;
       }
       catch (const char* error) {
         error_count_++;
         ROS_ERROR("Exception thrown while calibrating IMU %s", error);
         diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Exception thrown while calibrating IMU" + string(error));
         return false;
       }
     }
     else {
       ROS_INFO("IMU Calculating bias");
     }
     return true;
   }

   /**
    * @brief stop_calibrate Service to stop the magnetometer calibration
    * @param req Request object
    * @param resp Response object
    * @return True if the service is running correctly or false if not
    */
   bool stop_calibrate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {

     if(is_initialized) {
       try {
         ROS_INFO("Stoping magnetometer calibration");
         xbow.calibrateCommand('u');
         xbow.sendCommand((u_int8_t)measurementMode);
         if(messageMode == xbow4x::MessageMode::Continous) {
           xbow.sendCommand((u_int8_t)xbow4x::MessageMode::Continous);
           wallTimer.start();
         }
         calibrated=true;
         calibrating=false;
       }
       catch (const char* error) {
         error_count_++;
         ROS_ERROR("Exception thrown while calibrating IMU %s", error);
         diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Exception thrown while calibrating IMU" + string(error));
         return false;
       }
     }
     else {
       ROS_INFO("IMU Calculating bias");
     }
     return true;
   }

   /**
    * @brief Poll Service to poll data to the IMU
    * @param req Request object
    * @param resp Response object
    * @return True if the service is running correctly or false if not
    */
   bool poll(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
   {
     try {
       if(xbow.getReadingStatus())
         xbow.stopContinousReading();
       xbow.sendCommand('G');
     }
     catch(const char* error) {
       error_count_++;
       ROS_ERROR("Problem stoping continous mode %s", error);
       diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Problem stoping continous mode " + string(error));
     }

     if(wallTimer.hasStarted())
       wallTimer.stop();

     messageMode = xbow4x::MessageMode::Poll;
     node_handle.setParam("message_mode", u_int8_t(xbow4x::MessageMode::Poll));

     wallTimerCallback(ros::WallTimerEvent());

     return true;
   }

   /**
    * @brief Continuous Service change the node to continuous mode
    * @param req Request object
    * @param resp Response object
    * @return True if the service is running correctly or false if not
    */
   bool continuous(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
   {
     if(wallTimer.hasStarted())
       return true;
     else
       wallTimer.start();

     try {
       xbow.sendCommand('C');
       xbow.startContinuousReading();
     }
     catch(const char* error) {
       error_count_++;
       ROS_ERROR("Problem starting continous mode %s", error);
       diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN,"Problem starting continous mode " + string(error));
     }

     messageMode = xbow4x::MessageMode::Continous;
     node_handle.setParam("message_mode", u_int8_t(xbow4x::MessageMode::Continous));

     return true;
   }


  /**
   * @brief status Checks and return a string with the imu status
   * @return
   */
  string status() {

    string strMeasurementMode;
    string strMessageMode;

    switch ((char)measurementMode) {
    case 'c':
      strMeasurementMode = string("Scaled Mode-c");
      break;
    case 'a':
      strMeasurementMode = string("Angle Mode-a");
      break;
    case 'r':
      strMeasurementMode = string("Voltage Mode-r");
      break;
    default:
      break;
    }

    switch ((char)messageMode) {
    case 'C':
      strMessageMode = string("Continous Mode-C");
      break;
    case 'P':
      strMessageMode = string("Polled Mode-P");
      break;
    default:
      break;
    }

    return string("Measurement Mode:")+ strMeasurementMode + string(" Message Mode:")+strMessageMode;
  }

  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status) {

    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "IMU STATUS:" + this->status());

    status.add("Device", port_name);
    status.add("TF frame", frame_id);
    status.add("Base frame", base_frame);
    status.add("Error count", error_count_);
    status.add("Measurement and Message Modes",this->status());
    status.add("ENU Frame",use_enu_frame);
    status.add("Publis TF",broadcats_tf);
    status.add("Calibrating",!is_initialized);
    status.add("Temperature",temperature);
  }

  /**
      * @brief biasStatus Check the status of the imu bias
      * @param status node status
      */
   void biasStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
   {
     if(imu_started && !is_initialized)
       status.summary(diagnostic_msgs::DiagnosticStatus::OK,"IMU is capturing bias");
     else if(is_initialized) {
       status.summary(diagnostic_msgs::DiagnosticStatus::OK,"IMU bias captured");

       status.add("Gyro Bias X",ang_vel_mean[X]);
       status.add("Gyro Bias Y", ang_vel_mean[Y]);
       status.add("Gyro Bias Z", ang_vel_mean[Z]);

       status.add("Linear Accel Bias X",lin_acc_mean_no_grab[X]);
       status.add("Linear Accel Bias Y", lin_acc_mean_no_grab[Y]);
       status.add("Linear Accel Bias Z", lin_acc_mean_no_grab[Z]);
     }
     else
       status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU imu not captured");
   }


   /**
   * @brief calibrationStatus Check the status of the imu calibration
   * @param status node status
   */
   void calibrationStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
   {
     if (calibrated)
     {
       status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Magnetometer calibrated in the actual execution");
     }
     else if(calibrating) {
       status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Magnetometer is calibrating");
     }
     else
       status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Magnetometer not calibrated in the actual execution");
   }



  /*!
   * Returns the status (Measurement Mode and Message Mode) of the IMU. It uses the format Mode:[Mode Name]-[Command] in order to facilitate its parsing.
   *
   * \brief statusSrv Returns the status (Measurement Mode and Message Mode) of the IMU.
   * \param req Server Requets object
   * \param res Server Response object
   * \return The status in format Mode:[Mode Name]-[Command]
   */
  bool statusSrv(xbow4x::statusRequest &req, xbow4x::statusResponse &res) {

    res.response = status();
    return true;
  }


  /**
   * @brief transformRPY Transforms the euler angles (roll, pitch and yaw) to the ENU frame.
   * @param roll_in input roll in NED frame
   * @param pitch_in input pitch in NED frame
   * @param yaw_in input yaw in NED frame
   * @param roll_out output roll in ENU frame
   * @param pitch_out output pitch in ENU frame
   * @param yaw_out output yaw in ENU frame
   */
  void transformRPY(tf2Scalar roll_in,tf2Scalar pitch_in, tf2Scalar yaw_in, tf2Scalar &roll_out,tf2Scalar &pitch_out, tf2Scalar &yaw_out) {
    tf2::Quaternion q;
    tf2::Matrix3x3 m;

    q.setRPY(roll_in,pitch_in,yaw_in);
    q.normalize();

    q=transformQuaternion(q);

    m.setRotation(q);
    m.getRPY(roll_out,pitch_out,yaw_out);

    roll_out = roll_out;
    pitch_out = pitch_out;
    yaw_out = yaw_out;
  }

  /*!
   * Service broadcast_tf [true/false]: Enables or disables broadcasting of the IMU reference system.
   *
   * \brief broadcatsTf Service broadcast_tf [true/false]: Enables or disables broadcasting of the IMU reference system.
   * \param req Server Requets object
   * \param res Server Response object
   * \return The broadcast status
   */
  bool broadcatsTf(xbow4x::broadcast_tfRequest &req, xbow4x::broadcast_tfResponse &res) {
    broadcats_tf = req.active;
    req.active ? res.response="TF Active" : res.response="TF Inactive";
    node_handle.setParam("broadcast_tf", broadcats_tf);
    return true;
  }


  /**
   * @brief transformVector Transforms a vector from de NED IMU reference frame to the ENU reference frame
   * @param vIn Input vector to be transformed
   * @param vOut Output transformed vector
   */
  void transformVector(double* vIn,double* vOut){
    //Transforms has singular problems!!!
    tf2::Vector3 in,out;

    tf2::Quaternion qOut,qIn(vIn[X],vIn[Y],vIn[Z],0.0);


    qOut=q_rot*qIn*q_rot.inverse();

    vOut[X] = qOut.x();
    vOut[Y] = qOut.y();
    vOut[Z] = qOut.z();
  }

  /**
   * @brief transformQuaternion Transform a Quaternion from the IMU frame (NED) to ENU reference Frame
   * @param qIn Input quaternion
   * @return transformed quaternion
   */
  tf2::Quaternion transformQuaternion(tf2::Quaternion qIn) {
    tf2::Quaternion qOut;

    qOut=q_rot*qIn*q_rot.inverse();
    qOut.normalize();

    return qOut;
  }

  /*!
   * Broadcasts the IMU data in Angle Mode in the topics data_raw and data_no_grab. If broadcast_tf is enabled,
   * it also broadcasts the local reference system to the IMU.
   *
   * \brief publishImuDataAngleMode Broadcasts the IMU data in Angle Mode
   * \param data ImuData struct
   */
  void publishImuData() {

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    tf2::Vector3 g_w={0.0,0.0,-9.81},g_b,a_b;
    tf2::Quaternion q(0.0,0.0,0.0,1.0);
    double angularRate[3],accel[3],magnetic[3];
    xbow4x::ImuData data;
    ros::Time time;

    if(!imu_started) {
      imu_started = true;
      timer.start();
      counter=0;
      ang_vel_mean[X] = ang_vel_mean[Y] = ang_vel_mean[Z] = 0;
      lin_acc_mean_no_grab[X] = lin_acc_mean_no_grab[Y] = lin_acc_mean_no_grab[Z] = 0;
      ROS_INFO("CACULATING BIAS: DO NOT MOVE THE IMU. THIS WILL TAKE 10 SECONDS");
    }

    try {
      data = xbow.readSerialPort();
    }
    catch(const char* error) {
      ROS_WARN("%s",error);
      error_count_++;
      diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
    }
    temperature = data.boardtemp;

    time = ros::Time::now();
    angularRate[X] = data.rollrate;
    angularRate[Y] = data.pitchrate;
    angularRate[Z] = data.yawrate;

    accel[X] = data.ax;
    accel[Y] = data.ay;
    accel[Z] = data.az;

    magnetic[X] = data.xmag;
    magnetic[Y] = data.ymag;
    magnetic[Z] = data.zmag;

    if(measurementMode==MeasurementType::AngleMode) {
      q.setRPY(data.roll,data.pitch,data.yaw);
      q.normalize();
    }

    if(use_enu_frame) {
      transformVector(angularRate,angularRate);
      transformVector(accel,accel);
      transformVector(magnetic,magnetic);

      if(measurementMode==MeasurementType::AngleMode) {
        g_w[Z] = 9.81;
        q=transformQuaternion(q);
        if(euler)
          transformRPY(data.roll,data.pitch,data.yaw,data.roll,data.pitch,data.yaw);
      }

    }
    if(measurementMode==MeasurementType::AngleMode) {
      tf2::Transform transform(q.inverse());

      g_b= transform*g_w;


      a_b[X] = accel[X] - g_b[X];
      a_b[Y] = accel[Y] - g_b[Y];
      a_b[Z] = accel[Z] - g_b[Z];
    }


    if(is_initialized) {
      imu_msg.header.stamp = time;
      imu_msg.header.frame_id = frame_id;

      imu_msg.angular_velocity.x = angularRate[X] - ang_vel_mean[X];
      imu_msg.angular_velocity.y = angularRate[Y] - ang_vel_mean[Y];
      imu_msg.angular_velocity.z = angularRate[Z] - ang_vel_mean[Z];

      imu_msg.linear_acceleration.x = accel[X];
      imu_msg.linear_acceleration.y = accel[Y];
      imu_msg.linear_acceleration.z = accel[Z];

      imu_msg.orientation = tf2::toMsg(q);
      msgmag.header.stamp = time;
      msgmag.header.frame_id = frame_id;

      msgmag.magnetic_field.x = magnetic[X];
      msgmag.magnetic_field.y = magnetic[Y];
      msgmag.magnetic_field.z = magnetic[Z];

      imu_pub.publish(imu_msg);
      mag_pub.publish(msgmag);

      if(measurementMode==MeasurementType::AngleMode) {
        imu_msg_no_grab.header.stamp = time;
        imu_msg_no_grab.header.frame_id = frame_id;
        imu_msg_no_grab.angular_velocity.x = angularRate[X] - ang_vel_mean[X];
        imu_msg_no_grab.angular_velocity.y = angularRate[Y] - ang_vel_mean[Y];
        imu_msg_no_grab.angular_velocity.z = angularRate[Z] - ang_vel_mean[Z];

        imu_msg_no_grab.linear_acceleration.x = a_b[X] - lin_acc_mean_no_grab[X];
        imu_msg_no_grab.linear_acceleration.y = a_b[Y] - lin_acc_mean_no_grab[Y];
        imu_msg_no_grab.linear_acceleration.z = a_b[Z] - lin_acc_mean_no_grab[Z];

        imu_msg_no_grab.orientation = tf2::toMsg(q);

        imu_no_grab_pub.publish(imu_msg_no_grab);

        if(euler && measurementMode == xbow4x::MeasurementType::AngleMode) {
          eulerMsg.roll=data.roll*180/M_PI;
          eulerMsg.pitch=data.pitch*180/M_PI;
          eulerMsg.yaw=data.yaw*180/M_PI;
          eulerMsg.header.frame_id = frame_id;
          eulerMsg.header.stamp = time;

          euler_pub.publish(eulerMsg);
        }


        if(broadcats_tf ) {
          transformStamped.header.stamp = time;
          transformStamped.header.frame_id = base_frame;
          transformStamped.child_frame_id = frame_id;
          transformStamped.transform.translation.x = tf_translation[X];
          transformStamped.transform.translation.y = tf_translation[Y];
          transformStamped.transform.translation.z = tf_translation[Z];

          transformStamped.transform.rotation.x = q.x();
          transformStamped.transform.rotation.y = q.y();
          transformStamped.transform.rotation.z = q.z();
          transformStamped.transform.rotation.w = q.w();

          br.sendTransform(transformStamped);
        }
      }
    }
    else {
      ang_vel_mean[X] += angularRate[X];
      ang_vel_mean[Y] += angularRate[Y];
      ang_vel_mean[Z] += angularRate[Z];

      if(measurementMode==MeasurementType::AngleMode) {
        lin_acc_mean_no_grab[X] += a_b[X];
        lin_acc_mean_no_grab[Y] += a_b[Y];
        lin_acc_mean_no_grab[Z] += a_b[Z];
      }
      counter++;
      //      ROS_INFO("%d %lf %lf %lf",counter,ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
    }
  }

  /*!
   *
   * Service calibrate [command]: Send commands to the IMU to different actions related with the calibration. The commands are:
   *
   *  - s: starts calibration mode. Stops the other modes (continuous or polling). While the calibration is active the other modes do not work, it is necessary to stop the calibration.
   *  - u: stops calibration mode. The message mode active after it is polling mode.
   *  - h: clear hard iron calibration data. The message mode active after it is polling mode.
   *  - t: clear soft iron calibration data. The message mode active after it is polling mode.
   *
   * \brief calibrateSrv Send commands to the IMU to different actions related with the calibration.
   * \param xbow Reference to the xbow reader
   * \param req Server Request object
   * \param res Serever Response object
   * \return True if the server is running correctly or false if not
   */
  bool calibrateSrv(xbow4x::calibrateRequest &req, xbow4x::calibrateResponse &res) {
    string s;

    if(is_initialized) {
      if(req.command.length()>1)
        res.response="Incorrect command";
      else {
        try {
          if((req.command=="s" || req.command=="t" || req.command=="h") && wallTimer.hasStarted()) {
            wallTimer.stop();
            calibrating=true;
            calibrated=false;
          }

          res.response = xbow.calibrateCommand(req.command.c_str()[0]);
          ROS_INFO("Calibrate Service: %s", res.response.c_str());
          if(req.command=="u") {
            calibrated=true;
            calibrating=false;
          }
        }
        catch(const char* error) {
          error_count_++;
          diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU ERROR:: "+string(error));
          ROS_ERROR("%s",error);
          throw std::runtime_error(error);
          return false;
        }
      }
    }
    else
      res.response="The IMU is initializing, please try again later";
    return true;
  }


  /*!
  * Service send_Command [command]: Send commands to the IMU. The commands are:
  *   - R: Reset. Stops all calculations and communications, reset the IMU keeps the measurement mode and leaves the IMU in polled mode.
  *   - S: Returns the IMU serial number.
  *   - v: Returns the version of the IMU in use.
  *   - G: Returns a data packet in polling mode. For continuous mode, switching to polling mode.
  *   - P: Polling mode. If continuous mode is active, stops it and activates polling mode. The G command is used to poll packets.
  *   - C: Continuous mode. Returns data packets at a frequency of 60 Hz.
  *   - a: Angle mode. Returns angular velocity, linear acceleration, magnetic field and the angles (roll, pitch and yaw). In this mode a Kalman Filter is used to filter the signal.
  *   - c: Scaled Mode. Returns angular velocity, linear acceleration and magnetic field. This mode corrects the measurements.
  *   - r: Voltage mode. Returns angular velocity, linear acceleration and magnetic field. The data are raw, uncorrected and uncalibrated.
  *
  *
  * \param node_handle Reference to the node
  * \param xbow Reference to the xbow reader
  * \param req Server Request object
  * \param res Serever Response object
  * \return True if the server is running correctly or false if not
  */
  bool sendCommand(xbow4x::send_commandRequest &req, xbow4x::send_commandResponse &res) {
    string s;

    if(is_initialized) {
      if(req.command.length()>1)
        res.response="Incorrect command";
      else {
        try {
          res.response = xbow.sendCommand(req.command.c_str()[0]);

          if(req.command == string(1,(char)xbow4x::MeasurementType::AngleMode) || req.command == string(1,(char)xbow4x::MeasurementType::ScaledMode) || req.command == string(1,(char)xbow4x::MeasurementType::VoltageMode)) {
            measurementMode = static_cast<xbow4x::MeasurementType>(req.command.c_str()[0]);
            node_handle.setParam("measurement_mode", req.command.c_str()[0]);
            setWorkFrecuency();
            is_initialized = false;
            imu_started = false;
            if(measurementMode == xbow4x::MeasurementType::AngleMode && !euler_pub) {
              euler_pub = node_handle.advertise<xbow4x::EulerStamped>("euler", 1);
              imu_no_grab_pub = node_handle.advertise<sensor_msgs::Imu>("imu/data_no_grab", 1);
            }
            else if(measurementMode != xbow4x::MeasurementType::AngleMode && euler_pub) {
              euler_pub.shutdown();
              imu_no_grab_pub.shutdown();
            }
            if(messageMode != xbow4x::MessageMode::Continous) {
              xbow.sendCommand((u_int8_t)xbow4x::MessageMode::Continous);
              node_handle.setParam("message_mode", u_int8_t(xbow4x::MessageMode::Continous));
              wallTimer.start();
            }
          }
          else if(req.command == string(1,(char)xbow4x::MessageMode::Continous)) {
            messageMode = xbow4x::MessageMode::Continous;
            wallTimer.start();
          }
          else if(req.command == "R" || req.command == string(1,(char)xbow4x::MessageMode::Poll)|| req.command == "G") {
            messageMode = xbow4x::MessageMode::Poll;
            node_handle.setParam("message_mode", u_int8_t(xbow4x::MessageMode::Poll));
            wallTimer.stop();
            if(req.command == "G")
              wallTimerCallback(ros::WallTimerEvent());
          }
        }
        catch(const char* error) {
          diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU ERROR:: "+string(error));
          ROS_ERROR("%s",res.response.c_str());
          return true;
        }
      }
    }
    else
      res.response="The IMU is initializing, please try again later";
    return true;
  }

  /*!
   *
   * Callback of the timer that is activated in order to obtain data from the IMU to calculate the bias.
   * Although in Scaled and Angle modes the IMU itself already does this job, they are still calculated to
   * try to get a little more accuracy.
   *
   * \brief timerCallback Callback of the timer that is activated in order to obtain data from the IMU to calculate the bias.
   * \param node_handle Reference to the node
   * \param xbow Reference to the xbow reader
   */
  void timerCallback(const ros::TimerEvent&) {
    string s;

    is_initialized = true;
    timer.stop();
    ang_vel_mean[X] /=counter;
    ang_vel_mean[Y] /=counter;
    ang_vel_mean[Z] /=counter;
    ROS_INFO("ANG_RATE_BIAS: %lf %lf %lf",ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
    if(measurementMode == xbow4x::MeasurementType::AngleMode) {
      lin_acc_mean_no_grab[X] /= counter;
      lin_acc_mean_no_grab[Y] /= counter;
      lin_acc_mean_no_grab[Z] /= counter;
      ROS_INFO("NO_GRAB_ACCEL_BIAS %lf %lf %lf",lin_acc_mean_no_grab[X],lin_acc_mean_no_grab[Y],lin_acc_mean_no_grab[Z]);
    }
    if(messageMode != xbow4x::MessageMode::Continous) {
      try {
        xbow.sendCommand((u_int8_t)messageMode);
        wallTimer.stop();
      }
      catch(const char* error) {
        diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN, "IMU WARN: "+string(error));
        ROS_ERROR("%s",s.c_str());
      }
    }
  }
};

int main(int argc, char **argv)
{ 
//  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
//  {
//      ros::console::notifyLoggerLevelsChanged();
//  }
  string s;

  ros::init(argc, argv, "xbow4x_node");

  Xbow4xNode node;

  ros::spin();

  return 0;
}
