#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_srvs/srv/empty.hpp"

#include "diagnostic_msgs/srv/self_test.hpp"
#include "self_test/test_runner.hpp"
#include "diagnostic_updater/update_functions.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "xbow400/xbow4x.hpp"

#include "xbow400_interfaces/srv/broadcast_tf.hpp"
#include "xbow400_interfaces/srv/calibrate.hpp"
#include "xbow400_interfaces/srv/send_command.hpp"
#include "xbow400_interfaces/srv/set_baudrate.hpp"
#include "xbow400_interfaces/srv/status.hpp"

#include "xbow400_interfaces/msg/euler_stamped.hpp"

#define X 0 //! Index of the first element of array that corresponds to x component of a point
#define Y 1 //! Index of the first element of array that corresponds to y component of a point
#define Z 2 //! Index of the first element of array that corresponds to z component of a point

class Xbow4xNode : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr wallTimer;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub; //! Publisher object of the IMU data (angular rate, acceleration, orientation)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_no_grab_pub; //! Publisher object of the IMU data (angular rate whiout gravity, acceleration, orientation)
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub; //! Publisher object of the magnetometer (magnetic field vector)
  rclcpp::Publisher<xbow400_interfaces::msg::EulerStamped>::SharedPtr euler_pub; //! //! Publisher of the euler angles

  sensor_msgs::msg::Imu imu_msg, imu_msg_no_grab; //! Message to public the IMU topic
  sensor_msgs::msg::MagneticField msgmag; //! Message to public the Magnetometec topic
  xbow400_interfaces::msg::EulerStamped eulerMsg; //! Message type for the orientation data (Euler Stamped)


  self_test::TestRunner self_test_; //! Object to test the IMU
  diagnostic_updater::Updater diagnostic_; //! Object that allows the IMU diagnostic
  int error_count_; //! Counter of IMU errors

  double desired_freq_; //! Working frecuency of the IMU to pass to self_test
  std::unique_ptr<diagnostic_updater::FrequencyStatus> freq_diag_; //! Self_test object to monitoring the IMU diagnosis

  std::vector<double> ang_vel_mean={0.0,0.0,0.0}; //! Array of the algular rate bias
  std::vector<double>  lin_acc_mean_no_grab={0.0,0.0,0.0}; //! Array of the linear acceleration whiout gravity

  std::vector<double> tf_translation; //! Vector of the traslation vector of the TF translation matrix

  xbow4x::MeasurementType measurementMode; //! Enum object of the IMU measurement type (AngleMode, ScaledMode, VoltageMode) active
  xbow4x::MessageMode messageMode; //! Enum object of the IMU message mode (Continous or Poll) active

  bool is_initialized=false; //! Flag that indicates if the xbow is initialized
  bool imu_started=false; //! Flag that indicates if the xbow is strated
  bool broadcats_tf; //! Flag that indicates if the IMU reference frame is published or not
  bool euler; //! Publish euler data in euler topic
  string frame_id; //! Name of the IMU frame
  int counter=0;
  bool use_enu_frame; //! Determines if the ENU frame is used or not

  string base_frame; //! Name of the BASE frame
  int baudrate;

  tf2::Quaternion q_rot;
  string port_name; //! Name of the device port

  std::unique_ptr<tf2_ros::TransformBroadcaster> br;
  std::shared_ptr<xbow4x::XBOW4X> xbow;
  rclcpp::Service<xbow400_interfaces::srv::BroadcastTf>::SharedPtr broadcastTfSrv;
  rclcpp::Service<xbow400_interfaces::srv::Calibrate>::SharedPtr calibrateSrv;
  rclcpp::Service<xbow400_interfaces::srv::SendCommand>::SharedPtr sendCommandSrv;
  rclcpp::Service<xbow400_interfaces::srv::SetBaudrate>::SharedPtr setBaudrateSrv;
  rclcpp::Service<xbow400_interfaces::srv::Status>::SharedPtr statusSrv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_calibrate_serv; //! Service to start magnetometer calibration
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_calibrate_serv; //! Service to stop magnetometer calibration
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr poll_serv; //! Service to change the node to poll mode
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr continuous_serv; //!Service to change the node to continuous mode

  rclcpp::Clock clock;

  double temperature = 0.0;

  bool calibrated=false;
  bool calibrating=false;

public:

  ~Xbow4xNode() {
    RCLCPP_INFO(this->get_logger(),"Serial Port Disconnected");
    xbow->closePort();
  }

  Xbow4xNode() : Node("xbow4x_node"),
    self_test_(get_node_base_interface(), get_node_services_interface(), get_node_logging_interface()),
    diagnostic_(this),
    error_count_(0)
  {
    string s;
    int baudrate;
    std::vector<double> orientation_cov(9);
    std::vector<double> lin_acc_cov(9);
    std::vector<double> ang_vel_cov(9);
    std::vector<double> lin_acc_cov_no_grab(9);
    std::vector<double> mag_cov(9);


    ang_vel_mean = vector<double>(3);
    lin_acc_mean_no_grab = vector<double>(3);

    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
    mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 1);
    imu_no_grab_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_no_grab", 1);
    br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->declare_parameter("port", "/dev/ttyUSB0");
    port_name = this->get_parameter("port").get_parameter_value().get<string>();


    this->declare_parameter("baudrate", 38400);//38400
    baudrate = this->get_parameter("baudrate").get_parameter_value().get<int>();


    this->declare_parameter("broadcast_tf", true);
    broadcats_tf = this->get_parameter("broadcast_tf").get_parameter_value().get<bool>();

    this->declare_parameter("use_enu_frame", false);

    use_enu_frame = this->get_parameter("use_enu_frame").get_parameter_value().get<bool>();

    this->declare_parameter("frame_id","imu");
    frame_id = this->get_parameter("frame_id").get_parameter_value().get<string>();

    this->declare_parameter("base_frame","base");
    base_frame = this->get_parameter("base_frame").get_parameter_value().get<string>();

    //Angle Mode: a Scaled Mode: c Voltage Mode: r
    this->declare_parameter("measurement_mode","a"); //AngleMode
    measurementMode = static_cast<xbow4x::MeasurementType>(this->get_parameter("measurement_mode").get_parameter_value().get<string>().c_str()[0]);

    //Continuous Mode: C Poll Mode: P Poll/Stop Continuous Mode: G
    this->declare_parameter("message_mode","C"); //Continuous Mode
    messageMode = static_cast<xbow4x::MessageMode>(this->get_parameter("message_mode").get_parameter_value().get<string>().c_str()[0]);

    this->declare_parameter("euler", true);
    euler = this->get_parameter("euler").get_parameter_value().get<bool>();

    if(euler && measurementMode == xbow4x::MeasurementType::AngleMode) {
      euler_pub = this->create_publisher<xbow400_interfaces::msg::EulerStamped>("euler", 1);
    }

    this->declare_parameter("tf_translation",vector<double>({1.0,  1.0,    0.0}));

    tf_translation = this->get_parameter("tf_translation").get_parameter_value().get<vector<double>>();

    this->declare_parameter("orientation_cov",vector<double>({1.0e6,  0.0,    0.0,
                                                              0.0, 1.0e6, 0.0,
                                                              0.0, 0.0, 1.0e6}));


    orientation_cov = this->get_parameter("orientation_cov").get_parameter_value().get<vector<double>>();


    std::copy_n(orientation_cov.begin(),orientation_cov.size(),imu_msg.orientation_covariance.begin());
    std::copy_n(orientation_cov.begin(),orientation_cov.size(),imu_msg_no_grab.orientation_covariance.begin());


    this->declare_parameter("ang_vel_cov",vector<double>({1.0e-2, 0.0,    0.0,
                                                          0.0,    1.0e-2, 0.0,
                                                          0.0,    0.0,    1.0e-2}));
    ang_vel_cov = this->get_parameter("ang_vel_cov").get_parameter_value().get<vector<double>>();

    std::copy_n(ang_vel_cov.begin(),ang_vel_cov.size(),imu_msg.angular_velocity_covariance.begin());
    std::copy_n(ang_vel_cov.begin(),ang_vel_cov.size(),imu_msg_no_grab.angular_velocity_covariance.begin());


    this->declare_parameter("lin_acc_cov",vector<double>({1.0e-2, 0.0,    0.0,
                                                          0.0,    1.0e-2, 0.0,
                                                          0.0,    0.0,    1.0e-2}));
    lin_acc_cov = this->get_parameter("lin_acc_cov").get_parameter_value().get<vector<double>>();

    std::copy_n(lin_acc_cov.begin(),lin_acc_cov.size(),imu_msg.linear_acceleration_covariance.begin());


    this->declare_parameter("lin_acc_cov_no_grab",vector<double>({1.0e-2, 0.0,    0.0,
                                                                  0.0,    1.0e-2, 0.0,
                                                                  0.0,    0.0,    1.0e-2}));
    lin_acc_cov_no_grab = this->get_parameter("lin_acc_cov_no_grab").get_parameter_value().get<vector<double>>();

    std::copy_n(lin_acc_cov_no_grab.begin(),lin_acc_cov_no_grab.size(),imu_msg_no_grab.linear_acceleration_covariance.begin());

    this->declare_parameter("mag_cov",vector<double>({1.0e-2, 0.0,    0.0,
                                                      0.0,    1.0e-2, 0.0,
                                                      0.0,    0.0,    1.0e-2}));
    mag_cov = this->get_parameter("mag_cov").get_parameter_value().get<vector<double>>();

    std::copy_n(mag_cov.begin(),mag_cov.size(),msgmag.magnetic_field_covariance.begin());


    broadcastTfSrv = this->create_service<xbow400_interfaces::srv::BroadcastTf>("broadcast_tf",std::bind(&Xbow4xNode::broadcatsTf_srv,this,std::placeholders::_1,std::placeholders::_2));
    statusSrv = this->create_service<xbow400_interfaces::srv::Status>("status",std::bind(&Xbow4xNode::status_srv,this,std::placeholders::_1,std::placeholders::_2));
//    setBaudrateSrv = this->create_service<xbow400_interfaces::srv::SetBaudrate>("set_baudrate",std::bind(&Xbow4xNode::setBaudrate_srv,this,std::placeholders::_1,std::placeholders::_2));
    calibrateSrv = this->create_service<xbow400_interfaces::srv::Calibrate>("calibrateCommand",std::bind(&Xbow4xNode::calibrate_srv,this,std::placeholders::_1,std::placeholders::_2));
    sendCommandSrv = this->create_service<xbow400_interfaces::srv::SendCommand>("send_command",std::bind(&Xbow4xNode::sendCommand_srv,this,std::placeholders::_1,std::placeholders::_2));
    poll_serv = this->create_service<std_srvs::srv::Empty>("poll",std::bind(&Xbow4xNode::poll, this,std::placeholders::_1,std::placeholders::_2));
    continuous_serv = this->create_service<std_srvs::srv::Empty>("continuous",std::bind(&Xbow4xNode::continuous, this,std::placeholders::_1,std::placeholders::_2));
    start_calibrate_serv = this->create_service<std_srvs::srv::Empty>("start_calibrate",std::bind(&Xbow4xNode::start_calibrate, this,std::placeholders::_1,std::placeholders::_2));
    stop_calibrate_serv = this->create_service<std_srvs::srv::Empty>("stop_calibrate", std::bind(&Xbow4xNode::stop_calibrate, this,std::placeholders::_1,std::placeholders::_2));


    timer = rclcpp::create_timer(this,this->get_clock(),10s, std::bind(&Xbow4xNode::timerCallback,this));

    q_rot.setRPY(M_PI,0,M_PI/2); //ENU
    q_rot.normalize();

    wallTimer = this->create_wall_timer(0ms,std::bind(&Xbow4xNode::wallTimerCallback,this));
    wallTimer->cancel();

    diagnostic_.add( "Bias Status", this, &Xbow4xNode::biasStatus);
    diagnostic_.add( "IMU Status", this, &Xbow4xNode::deviceStatus);
    diagnostic_.add( "Calibration Status", this, &Xbow4xNode::calibrationStatus);
    setWorkFrecuency();

    self_test_.add("Interruption Test", this, &Xbow4xNode::interruptionTest);
    self_test_.add("Connect Test", this, &Xbow4xNode::connectTest);
    self_test_.add("Read ID Test", this, &Xbow4xNode::readIDTest);
    self_test_.add("Streamed Data Test", this, &Xbow4xNode::streamedDataTest);
    self_test_.add("Gyro Bias Test", this, &Xbow4xNode::biasTest);
    self_test_.add("Gravity Test", this, &Xbow4xNode::gravityTest);
    self_test_.add("Disconnect Test", this, &Xbow4xNode::disconnectTest);
    self_test_.add("Resume Test", this, &Xbow4xNode::resumeTest);

//    this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    xbow = std::make_shared<xbow4x::XBOW4X>(this->get_logger());

    try{
      xbow->openPort(port_name.c_str(), baudrate);

      xbow->sendCommand('R');

      diagnostic_.setHardwareID(getID());

      xbow->sendCommand((u_int8_t)measurementMode);


      //First in continous mode to calculate de bias
      xbow->sendCommand((u_int8_t)xbow4x::MessageMode::Continous);
      wallTimer->reset();
    }
    catch(const char* error) {
      RCLCPP_ERROR(this->get_logger(),"%s",error);
      error_count_++;
      diagnostic_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU ERROR: "+string(error));
      rclcpp::shutdown();
      exit(1);
    }
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
        xbow->openPort(port_name.c_str(), baudrate);
        xbow->sendCommand('R');
        xbow->sendCommand((u_int8_t)measurementMode);
        xbow->sendCommand((u_int8_t)messageMode);
        freq_diag_->clear();
      }
      catch(const char *error) {
        RCLCPP_ERROR(this->get_logger(),"%s",error);
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Could not be done the resume test.");
      }

    }

    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Previous operation resumed successfully.");
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
      dmu_ver = xbow->sendCommand('v');
      serial_number = xbow->sendCommand('S');
      id = dmu_ver + "_" + serial_number;

      RCLCPP_INFO(this->get_logger(),"Connected to IMU %s",id.c_str());

      return id;
    }
    catch(const char* error) {
      RCLCPP_ERROR(this->get_logger(),"%s",error);
      error_count_++;
      diagnostic_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, "IMU ERROR:: "+string(error));
    }
    return "";
  }

  /**
   * @brief Callback that reads the data stream from de IMU sensor
   * @return
   */
  void wallTimerCallback() {
    publishImuData();

    freq_diag_->tick();
  }


  /**
    * @brief InterruptionTest Test if the serial port has other connections
    * @param status node status
    */
  void interruptionTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (imu_pub->get_subscription_count() > 0 ||
        mag_pub->get_subscription_count() > 0 ||
        imu_no_grab_pub->get_subscription_count() > 0 ||
        (euler_pub != nullptr && euler_pub->get_subscription_count() > 0))
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Interruption Text: There were active subscribers.");
    else
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No operation interrupted.");
  }

  /**
              * @brief ConnectTest Test if the connection throght the serial port is possible
              * @param status node status
              */
  void connectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    try {
      xbow->openPort(port_name.c_str(), baudrate);
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Connected successfully.");
    }
    catch(const char* error) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "It is not possible to connect: "+string(error));
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
      xbow->sendCommand('R');
      id=getID();

      if(id.length()!=32) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Imposible to read IMU ID");
      }
      else {
        self_test_.setID(id);
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Read id Successfully");
      }
    }
    catch(const char* error) {
      RCLCPP_ERROR(this->get_logger(),"%s",error);
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Could not read de IMU ID: "+string(error));
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
      xbow->sendCommand('R');
      xbow->sendCommand((u_int8_t)measurementMode);
      xbow->sendCommand('C');
    }
    catch(const char* error) {
      RCLCPP_ERROR(this->get_logger(),"%s",error);
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
    }
    try {
      for(int i=0;i<it;i++){

        xbow4x::ImuData data = xbow->readSerialPort();

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
      RCLCPP_ERROR(this->get_logger(),"%s",error);
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
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
      xbow->sendCommand('P');
    }
    catch(const char* error) {
      RCLCPP_ERROR(this->get_logger(),"%s",error);
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
    }

    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Data streamed successfully.");

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
    rclcpp::Time start_time;


    is_initialized=false;
    imu_started=false;

    try {
      xbow->sendCommand('C');
    }
    catch(const char* error) {
      RCLCPP_ERROR(this->get_logger(),"%s",error);
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
    }
    start_time = now();
    while(now() - start_time < rclcpp::Duration::from_nanoseconds(10.0e9)){
      publishImuData();
    }
    timerCallback();
    try{
      xbow->sendCommand('P');

      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Successfully calculated gyro biases.");

      status.add("Gyro Bias X",ang_vel_mean[X]);
      status.add("Gyro Bias Y", ang_vel_mean[Y]);
      status.add("Gyro Bias Z", ang_vel_mean[Z]);

      status.add("Linear Accel Bias X",lin_acc_mean_no_grab[X]);
      status.add("Linear Accel Bias Y", lin_acc_mean_no_grab[Y]);
      status.add("Linear Accel Bias Z", lin_acc_mean_no_grab[Z]);
    }
    catch(const char* error) {
      RCLCPP_ERROR(this->get_logger(),"%s",error);
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
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
      xbow->sendCommand('C');
      int num = 200;
      for (int i = 0; i < num; i++)
      {
        xbow4x::ImuData data = xbow->readSerialPort();

        grav_x += data.ax;
        grav_y += data.ay;
        grav_z += data.az;
      }
      xbow->sendCommand('P');

      grav += sqrt( pow(grav_x / (double)(num), 2.0) +
                    pow(grav_y / (double)(num), 2.0) +
                    pow(grav_z / (double)(num), 2.0));

      double err = (grav - 9.81);

      if (fabs(err) < .05)
      {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Gravity detected correctly.");
      } else {
        status.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Measured gravity deviates by %f", err);
      }

      status.add("Measured gravity", grav);
      status.add("Gravity error", err);
    }
    catch(const char* error) {
      RCLCPP_ERROR(this->get_logger(),"%s",error);
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Could not be done the gravity test: "+string(error));
    }
  }


  /**
   * @brief DisconnectTest Test if the IMU is disconnected correctly
   * @param status
   */
  void disconnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    xbow->closePort();

    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Disconnected successfully.");
  }

  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status) {

    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "IMU STATUS:" + this->status());

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
     * @brief calibrationStatus Check the status of the imu calibration
     * @param status node status
     */
  void biasStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if(imu_started && !is_initialized)
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,"IMU is capturing bias");
    else if(is_initialized) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,"IMU bias captured");

      status.add("Gyro Bias X",ang_vel_mean[X]);
      status.add("Gyro Bias Y", ang_vel_mean[Y]);
      status.add("Gyro Bias Z", ang_vel_mean[Z]);

      status.add("Linear Accel Bias X",lin_acc_mean_no_grab[X]);
      status.add("Linear Accel Bias Y", lin_acc_mean_no_grab[Y]);
      status.add("Linear Accel Bias Z", lin_acc_mean_no_grab[Z]);
    }
    else
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU imu not captured");
  }

  /**
   * @brief calibrationStatus Check the status of the imu calibration
   * @param status node status
   */
  void calibrationStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (calibrated)
    {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Magnetometer calibrated in the actual execution");
    }
    else if(calibrating) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Magnetometer is calibrating");
    }
    else
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Magnetometer not calibrated in the actual execution");
  }

  /*!
   * Returns the status (Measurement Mode and Message Mode) of the IMU. It uses the format Mode:[Mode Name]-[Command] in order to facilitate its parsing.
   *
   * \brief statusSrv Returns the status (Measurement Mode and Message Mode) of the IMU.
   * \param req Server Requets object
   * \param res Server Response object
   * \return The status in format Mode:[Mode Name]-[Command]
   */
  bool status_srv(const std::shared_ptr<xbow400_interfaces::srv::Status::Request> req, const std::shared_ptr<xbow400_interfaces::srv::Status::Response> res) {

    res->response = status();
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
     * Broadcasts the IMU data in Angle Mode in the topics data_raw and data_no_grab. If broadcast_tf is enabled,
     * it also broadcasts the local reference system to the IMU.
     *
     * \brief publishImuDataAngleMode Broadcasts the IMU data in Angle Mode
     * \param data ImuData struct
     */
  void publishImuData() {
    string s;
    geometry_msgs::msg::TransformStamped transformStamped;

    tf2::Vector3 g_w(0.0,0.0,-9.81),g_b,a_b;
    tf2::Quaternion q(0.0,0.0,0.0,1.0);
    double angularRate[3],accel[3],magnetic[3];
    xbow4x::ImuData data;

    if(!imu_started) {
      imu_started = true;
      timer->reset();
      counter=0;
      ang_vel_mean[X] = ang_vel_mean[Y] = ang_vel_mean[Z] = 0;
      lin_acc_mean_no_grab[X] = lin_acc_mean_no_grab[Y] = lin_acc_mean_no_grab[Z] = 0;
      RCLCPP_INFO(this->get_logger(),"CACULATING BIAS: DO NOT MOVE THE IMU. THIS WILL TAKE 10 SECONDS");
    }

    try {
      data = xbow->readSerialPort();
    }
    catch(const char* error) {
      RCLCPP_WARN(this->get_logger(),"%s",error);
      diagnostic_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Could not start streaming data: "+string(error));
    }
    temperature = data.boardtemp;

    data.receive_time = clock.now();
    angularRate[X] = data.rollrate;
    angularRate[Y] = data.pitchrate;
    angularRate[Z] = data.yawrate;

    accel[X] = data.ax;
    accel[Y] = data.ay;
    accel[Z] = data.az;

    magnetic[X] = data.xmag;
    magnetic[Y] = data.ymag;
    magnetic[Z] = data.zmag;

    if(measurementMode==xbow4x::MeasurementType::AngleMode) {
      q.setRPY(data.roll,data.pitch,data.yaw);
      q.normalize();
    }
    if(use_enu_frame) {

      transformVector(angularRate,angularRate);
      transformVector(accel,accel);
      transformVector(magnetic,magnetic);
      if(measurementMode==xbow4x::MeasurementType::AngleMode) {
        g_w[Z] = 9.81;
        q=transformQuaternion(q);
        if(euler)
          transformRPY(data.roll,data.pitch,data.yaw,data.roll,data.pitch,data.yaw);
      }
    }


    if(measurementMode==xbow4x::MeasurementType::AngleMode) {
      tf2::Transform transform(q.inverse());

      g_b= transform*g_w;


      a_b[X] = accel[X] - g_b[X];
      a_b[Y] = accel[Y] - g_b[Y];
      a_b[Z] = accel[Z] - g_b[Z];
    }

    if(is_initialized) {
      imu_msg.header.stamp = data.receive_time;
      imu_msg.header.frame_id = frame_id;
      imu_msg.angular_velocity.x = angularRate[X] - ang_vel_mean[X];
      imu_msg.angular_velocity.y = angularRate[Y] - ang_vel_mean[Y];
      imu_msg.angular_velocity.z = angularRate[Z] - ang_vel_mean[Z];
      imu_msg.linear_acceleration.x = accel[X];
      imu_msg.linear_acceleration.y = accel[Y];
      imu_msg.linear_acceleration.z = accel[Z];
      imu_msg.orientation = tf2::toMsg(q);
      msgmag.header.stamp = data.receive_time;
      msgmag.header.frame_id = frame_id;
      msgmag.magnetic_field.x = magnetic[X];
      msgmag.magnetic_field.y = magnetic[Y];
      msgmag.magnetic_field.z = magnetic[Z];

      imu_pub->publish(imu_msg);
      mag_pub->publish(msgmag);

      if(measurementMode==xbow4x::MeasurementType::AngleMode) {
        imu_msg_no_grab.header.stamp = data.receive_time;
        imu_msg_no_grab.header.frame_id = frame_id;
        imu_msg_no_grab.angular_velocity.x = angularRate[X] - ang_vel_mean[X];
        imu_msg_no_grab.angular_velocity.y = angularRate[Y] - ang_vel_mean[Y];
        imu_msg_no_grab.angular_velocity.z = angularRate[Z] - ang_vel_mean[Z];

        imu_msg_no_grab.linear_acceleration.x = a_b[X] - lin_acc_mean_no_grab[X];
        imu_msg_no_grab.linear_acceleration.y = a_b[Y] - lin_acc_mean_no_grab[Y];
        imu_msg_no_grab.linear_acceleration.z = a_b[Z] - lin_acc_mean_no_grab[Z];

        imu_msg_no_grab.orientation = tf2::toMsg(q);
        imu_no_grab_pub->publish(imu_msg_no_grab);

        if(euler && measurementMode == xbow4x::MeasurementType::AngleMode) {
          eulerMsg.roll=data.roll*180/M_PI;
          eulerMsg.pitch=data.pitch*180/M_PI;
          eulerMsg.yaw=data.yaw*180/M_PI;
          eulerMsg.header.frame_id = frame_id;
          eulerMsg.header.stamp = data.receive_time;

          euler_pub->publish(eulerMsg);
        }


        //          RCLCPP_INFO(this->get_logger(),"%lf %lf %lf",data.ax,data.ay,data.az);
        if(broadcats_tf) {
          transformStamped.header.stamp = data.receive_time;
          transformStamped.header.frame_id = base_frame;
          transformStamped.child_frame_id = frame_id;
          transformStamped.transform.translation.x = tf_translation[X];
          transformStamped.transform.translation.y = tf_translation[Y];
          transformStamped.transform.translation.z = tf_translation[Z];

          transformStamped.transform.rotation.x = q.x();
          transformStamped.transform.rotation.y = q.y();
          transformStamped.transform.rotation.z = q.z();
          transformStamped.transform.rotation.w = q.w();

          br->sendTransform(transformStamped);
        }
      }
    }
    else {
      ang_vel_mean[X] += angularRate[X];
      ang_vel_mean[Y] += angularRate[Y];
      ang_vel_mean[Z] += angularRate[Z];

      if(measurementMode==xbow4x::MeasurementType::AngleMode) {
        lin_acc_mean_no_grab[X] += a_b[X];
        lin_acc_mean_no_grab[Y] += a_b[Y];
        lin_acc_mean_no_grab[Z] += a_b[Z];
      }
      counter++;
    }
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
   * Returns the status (Measurement Mode and Message Mode) of the IMU. It uses the format Mode:[Mode Name]-[Command] in order to facilitate its parsing.
   *
   * \brief statusSrv Returns the status (Measurement Mode and Message Mode) of the IMU.
   * \param req Server Requets object
   * \param res Server Response object
   * \return The status in format Mode:[Mode Name]-[Command]
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

  /**
   * @brief start_calibrate Service to start the magnetometer calibration
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool start_calibrate(const std::shared_ptr<std_srvs::srv::Empty::Request> &req, const std::shared_ptr<std_srvs::srv::Empty::Response> &resp) {

    if(is_initialized) {
      try {
        calibrated=false;
        wallTimer->cancel();
        RCLCPP_INFO(this->get_logger(),"Clearing hard iron offset");
        xbow->calibrateCommand('h');
        RCLCPP_INFO(this->get_logger(),"Clearing soft iron offset");
        xbow->calibrateCommand('t');
        RCLCPP_INFO(this->get_logger(),"Starting magnetometer calibration");
        xbow->calibrateCommand('s');
        calibrating=true;
      }
      catch (const char* error) {
        error_count_++;
        RCLCPP_ERROR(this->get_logger(),"Exception thrown while calibrating IMU %s", error);
        diagnostic_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN,"Exception thrown while calibrating IMU" + string(error));
        return false;
      }
    }
    else {
      RCLCPP_INFO(this->get_logger(),"IMU Calculating bias");
    }
    return true;
  }

  /**
   * @brief stop_calibrate Service to stop the magnetometer calibration
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool stop_calibrate(const std::shared_ptr<std_srvs::srv::Empty::Request> &req, const std::shared_ptr<std_srvs::srv::Empty::Response> &resp) {

    if(is_initialized) {
      try {
        RCLCPP_INFO(this->get_logger(),"Stoping magnetometer calibration");
        xbow->calibrateCommand('u');
        xbow->sendCommand((u_int8_t)measurementMode);
        if(messageMode == xbow4x::MessageMode::Continous) {
          xbow->sendCommand((u_int8_t)xbow4x::MessageMode::Continous);
          wallTimer->reset();
        }
        calibrated=true;
        calibrating=false;
      }
      catch (const char* error) {
        error_count_++;
        RCLCPP_ERROR(this->get_logger(),"Exception thrown while calibrating IMU %s", error);
        diagnostic_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN,"Exception thrown while calibrating IMU" + string(error));
        return false;
      }
    }
    else {
      RCLCPP_INFO(this->get_logger(),"IMU Calculating bias");
    }
    return true;
  }

  /**
   * @brief Poll Service to poll data to the IMU
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool poll(const std::shared_ptr<std_srvs::srv::Empty::Request> &req, const std::shared_ptr<std_srvs::srv::Empty::Response> &resp)
  {
    try {
      if(xbow->getReadingStatus())
        xbow->stopContinousReading();
      xbow->sendCommand('G');
    }
    catch(const char* error) {
      error_count_++;
      RCLCPP_ERROR(this->get_logger(),"Problem stoping continous mode %s", error);
      diagnostic_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN,"Problem stoping continous mode " + string(error));
    }

    if(wallTimer->is_ready())
      wallTimer->cancel();

    messageMode = xbow4x::MessageMode::Poll;
    this->set_parameter(rclcpp::Parameter("message_mode", u_int8_t(xbow4x::MessageMode::Poll)));

    wallTimerCallback();

    return true;
  }

  /**
   * @brief Continuous Service change the node to continuous mode
   * @param req Request object
   * @param resp Response object
   * @return True if the service is running correctly or false if not
   */
  bool continuous(const std::shared_ptr<std_srvs::srv::Empty::Request> &req, const std::shared_ptr<std_srvs::srv::Empty::Response> &resp)
  {
    if(wallTimer->is_ready())
      return true;
    else
      wallTimer->reset();

    try {
      xbow->sendCommand('C');
      xbow->startContinuousReading();
    }
    catch(const char* error) {
      error_count_++;
      RCLCPP_ERROR(this->get_logger(),"Problem starting continous mode %s", error);
      diagnostic_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN,"Problem starting continous mode " + string(error));
    }

    messageMode = xbow4x::MessageMode::Continous;
    this->set_parameter(rclcpp::Parameter("message_mode", u_int8_t(xbow4x::MessageMode::Continous)));

    return true;
  }

  /*!
   * Service broadcast_tf [true/false]: Enables or disables broadcasting of the IMU reference system.
   *
   * \brief broadcatsTf Service broadcast_tf [true/false]: Enables or disables broadcasting of the IMU reference system.
   * \param req Server Requets object
   * \param res Server Response object
   * \return The broadcast status
   */
  bool broadcatsTf_srv(const std::shared_ptr<xbow400_interfaces::srv::BroadcastTf::Request> req, const std::shared_ptr<xbow400_interfaces::srv::BroadcastTf::Response> res) {
    broadcats_tf = req->active;
    req->active ? res->response="TF Active" : res->response="TF Inactive";
    
    this->set_parameter(rclcpp::Parameter("broadcast_tf", broadcats_tf));
    return true;
  }


//IMU NOT REPLY
//  /*!
//   * Sets a new baudrate in the serial port RS232 communication with the IMU. Be carreful with this service, it is possible to lost
//   * the communication with the IMU. To reset the baudrate to 38400 disconnect the IMU from the power and from de USB.
//   *
//   * \brief setBaudrate Sets a new baudrate in the serial port RS232 communication with the IMU.
//   * \param xbow Reference to the xbow reader
//   * \param req Server Request object
//   * \param res Serever Response object
//   * \return True if the server is running correctly or false if not
//   */
//  bool setBaudrate_srv(const std::shared_ptr<xbow400_interfaces::srv::SetBaudrate::Request> req, const std::shared_ptr<xbow400_interfaces::srv::SetBaudrate::Response> res) {
//    try {
//      wallTimer->cancel();
//      res->response = xbow->setBaudrate(req->baudrate);
//    }
//    catch(const char* error) {
//      error_count_++;
//      diagnostic_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU ERROR: "+string(error));
//      RCLCPP_ERROR(this->get_logger(),"%s",error);
//      return false;
//    }
//    wallTimer->reset();
//    this->set_parameter(rclcpp::Parameter("baudrate", req->baudrate));
//    return true;
//  }

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
  bool calibrate_srv(const std::shared_ptr<xbow400_interfaces::srv::Calibrate::Request> req, const std::shared_ptr<xbow400_interfaces::srv::Calibrate::Response> res) {
    string s;

    if(is_initialized) {
      if(req->command.length()>1)
        res->response="Incorrect command";
      else {
        try {
          if((req->command=="s" || req->command=="t" || req->command=="h") && wallTimer->is_ready()) {
            wallTimer->cancel();
            calibrating=true;
            calibrated=false;
          }

          res->response = xbow->calibrateCommand(req->command.c_str()[0]);
          RCLCPP_INFO(this->get_logger(),"Calibrate Service: %s", res->response.c_str());
          if(req->command=="u") {
            calibrated=true;
            calibrating=false;
          }
        }
        catch(const char* error) {
          error_count_++;
          diagnostic_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU ERROR:: "+string(error));
          RCLCPP_ERROR(this->get_logger(),"%s",error);
          throw std::runtime_error(error);
          return false;
        }
      }
    }
    else
      res->response="The IMU is initializing, please try again later";
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
  bool sendCommand_srv(const std::shared_ptr<xbow400_interfaces::srv::SendCommand::Request> req, const std::shared_ptr<xbow400_interfaces::srv::SendCommand::Response> res) {
    string s;

    if(is_initialized) {
      if(req->command.length()>1)
        res->response="Incorrect command";
      else {
        try {
          res->response = xbow->sendCommand(req->command.c_str()[0]);

          if(req->command == string(1,(char)xbow4x::MeasurementType::AngleMode) || req->command == string(1,(char)xbow4x::MeasurementType::ScaledMode) || req->command == string(1,(char)xbow4x::MeasurementType::VoltageMode)) {
            measurementMode = static_cast<xbow4x::MeasurementType>(req->command.c_str()[0]);
            this->set_parameter(rclcpp::Parameter("measurement_mode", req->command.c_str()[0]));
            setWorkFrecuency();
            is_initialized = false;
            imu_started = false;
            if(measurementMode == xbow4x::MeasurementType::AngleMode && !euler_pub) {
              euler_pub = this->create_publisher<xbow400_interfaces::msg::EulerStamped>("euler", 1);
              imu_no_grab_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_no_grab", 1);
            }
            else if(measurementMode != xbow4x::MeasurementType::AngleMode && euler_pub) {
              euler_pub.reset();
              imu_no_grab_pub.reset();
            }

            if(messageMode != xbow4x::MessageMode::Continous) {
              xbow->sendCommand((u_int8_t)xbow4x::MessageMode::Continous);
              wallTimer->reset();
            }
          }
          else if(req->command == string(1,(char)xbow4x::MessageMode::Continous)) {
            messageMode = xbow4x::MessageMode::Continous;
            this->set_parameter(rclcpp::Parameter("message_mode", u_int8_t(xbow4x::MessageMode::Continous)));
            wallTimer->reset();
          }
          else if(req->command == "R" || req->command == string(1,(char)xbow4x::MessageMode::Poll)|| req->command == "G") {
            messageMode = xbow4x::MessageMode::Poll;
            this->set_parameter(rclcpp::Parameter("message_mode", u_int8_t(xbow4x::MessageMode::Poll)));
            wallTimer->cancel();
            if(req->command == "G")
              wallTimerCallback();
          }
        }
        catch(const char* error) {
          diagnostic_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU ERROR:: "+string(error));
          RCLCPP_ERROR(this->get_logger(),"%s",error);
          res->response = error;
          return true;
        }
      }
    }
    else
      res->response="The IMU is initializing, please try again later";
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
  void timerCallback() {
    //ROS_INFO("%d %lf %lf %lf",counter,ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
    string s;
    is_initialized = true;
    timer->cancel();
    ang_vel_mean[X] /=counter;
    ang_vel_mean[Y] /=counter;
    ang_vel_mean[Z] /=counter;
    RCLCPP_INFO(this->get_logger(),"ANG_RATE_BIAS: %lf %lf %lf",ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
    if(measurementMode == xbow4x::MeasurementType::AngleMode) {
      lin_acc_mean_no_grab[X] /= counter;
      lin_acc_mean_no_grab[Y] /= counter;
      lin_acc_mean_no_grab[Z] /= counter;
      RCLCPP_INFO(this->get_logger(),"NO_GRAB_ACCEL_BIAS %lf %lf %lf",lin_acc_mean_no_grab[X],lin_acc_mean_no_grab[Y],lin_acc_mean_no_grab[Z]);
    }
    if(messageMode != xbow4x::MessageMode::Continous) {
      try {
        xbow->sendCommand((u_int8_t)messageMode);
        wallTimer->cancel();
      }
      catch(const char* error) {
        diagnostic_.broadcast(1, "IMU ERROR:: "+string(error));
        RCLCPP_ERROR(this->get_logger(),"%s",s.c_str());
      }
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Xbow4xNode>());
  rclcpp::shutdown();

  return 0;
}

