#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <signal.h>

#include <boost/bind.hpp>

#include "xbow4x/broadcast_tf.h"
#include "xbow4x/send_command.h"
#include "xbow4x/status.h"
#include "xbow4x/calibrate.h"
#include "xbow4x/set_baudrate.h"

#include "xbow4x/xbow4x.h"
using namespace xbow4x;
using std::string;

#define X 0 //! Index of the first element of array that corresponds to x component of a point
#define Y 1 //! Index of the first element of array that corresponds to y component of a point
#define Z 2 //! Index of the first element of array that corresponds to z component of a point

ros::Publisher imu_pub; //! Publisher object of the IMU data (angular rate, acceleration, orientation)
ros::Publisher mag_pub; //! Publisher object of the magnetometer (magnetic field vector)
ros::Publisher imu_no_grab_pub; //! Publisher object of the IMU data (angular rate whiout gravity, acceleration, orientation)
string frame_id; //! Name of the IMU frame
sensor_msgs::Imu msg, msg_no_grab; //! Message to public the IMU topic
sensor_msgs::MagneticField msgmag; //! Message to public the Magnetometec topic

std::vector<double> ang_vel_mean; //! Array of the algular rate bias
std::vector<double>  lin_acc_mean_no_grab; //! Array of the linear acceleration whiout gravity
bool is_initialized=false; //! Flag that indicates if the xbow is initialized
bool imu_started=false; //! Flag that indicates if the xbow is strated
int counter=0;
ros::Timer timer; //! Timer to calculate de angular rate and the acceleration gravity
bool broadcats_tf; //! Flag that indicates if the IMU reference frame is published or not
xbow4x::MeasurementType measurementMode; //! Enum object of the IMU measurement type (AngleMode, ScaledMode, VoltageMode) active
xbow4x::MessageMode messageMode; //! Enum object of the IMU message mode (Continous or Poll) active

/*!
 * Returns the status (Measurement Mode and Message Mode) of the IMU. It uses the format Mode:[Mode Name]-[Command] in order to facilitate its parsing.
 *
 * \brief statusSrv Returns the status (Measurement Mode and Message Mode) of the IMU.
 * \param req Server Requets object
 * \param res Server Response object
 * \return The status in format Mode:[Mode Name]-[Command]
 */
bool statusSrv(xbow4x::statusRequest &req, xbow4x::statusResponse &res) {

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

  res.response = string("Measurement Mode:")+ strMeasurementMode + string(" Message Mode:")+strMessageMode;
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
bool broadcatsTf(xbow4x::broadcast_tfRequest &req, xbow4x::broadcast_tfResponse &res) {
  broadcats_tf = req.active;
  req.active ? res.response="TF Active" : res.response="TF Inactive";
  return true;
}

/*!
 * Broadcasts the IMU data in Angle Mode in the topics data_raw and data_no_grab. If broadcast_tf is enabled,
 * it also broadcasts the local reference system to the IMU.
 *
 * \brief publishImuDataAngleMode Broadcasts the IMU data in Angle Mode
 * \param data ImuData struct
 */
void publishImuDataAngleMode(const ImuData& data) {

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    tf2::Vector3 g_w(0.0,0.0,-9.81),g_b,a_b;
    tf2::Quaternion q, q_rot, q2;

    if(!imu_started) {
      imu_started = true;
      timer.start();
      counter=0;
      ang_vel_mean[X] = ang_vel_mean[Y] = ang_vel_mean[Z] = 0;
      lin_acc_mean_no_grab[X] = lin_acc_mean_no_grab[Y] = lin_acc_mean_no_grab[Z] = 0;
      ROS_INFO("CACULATING BIAS: DO NOT MOVE THE IMU. THIS WILL TAKE 10 SECONDS");
    }

    q.setRPY(data.roll,data.pitch,data.yaw);
    q.normalize();
    q_rot.setRPY(M_PI,0,0);
    q2=q_rot*q;
    q2.normalize();

    tf2::Transform transform(q2.inverse());

    g_b= transform*g_w;


    a_b[X] = data.ax - g_b[X];
    a_b[Y] = data.ay - g_b[Y];
    a_b[Z] = data.az - g_b[Z];

    if(is_initialized) {
      msg.header.stamp = data.receive_time;
      msg.header.frame_id = frame_id;
      msg.angular_velocity.x = data.rollrate - ang_vel_mean[X];
      msg.angular_velocity.y = data.pitchrate - ang_vel_mean[Y];
      msg.angular_velocity.z = data.yawrate - ang_vel_mean[Z];
      msg.linear_acceleration.x = data.ax;
      msg.linear_acceleration.y = data.ay;
      msg.linear_acceleration.z = data.az;
      msg.orientation = tf2::toMsg(q);
      msgmag.header.stamp = data.receive_time;
      msgmag.header.frame_id = frame_id;
      msgmag.magnetic_field.x = data.xmag;
      msgmag.magnetic_field.y = data.ymag;
      msgmag.magnetic_field.z = data.zmag;




      msg_no_grab.header.stamp = data.receive_time;
      msg_no_grab.header.frame_id = frame_id;
      msg_no_grab.angular_velocity.x = data.rollrate - ang_vel_mean[X];
      msg_no_grab.angular_velocity.y = data.pitchrate - ang_vel_mean[Y];
      msg_no_grab.angular_velocity.z = data.yawrate - ang_vel_mean[Z];
      msg_no_grab.linear_acceleration.x = a_b[X] - lin_acc_mean_no_grab[X];
      msg_no_grab.linear_acceleration.y = a_b[Y] - lin_acc_mean_no_grab[Y];
      msg_no_grab.linear_acceleration.z = a_b[Z] - lin_acc_mean_no_grab[Z];
      q.setRPY(data.roll,data.pitch,data.yaw);
      q.normalize();

      msg_no_grab.orientation = tf2::toMsg(q);


      imu_pub.publish(msg);
      imu_no_grab_pub.publish(msg_no_grab);
      mag_pub.publish(msgmag);
      if(broadcats_tf) {
        transformStamped.header.stamp = data.receive_time;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = frame_id;
        transformStamped.transform.translation.x = 2.0;
        transformStamped.transform.translation.y = 2.0;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = q2.x();
        transformStamped.transform.rotation.y = q2.y();
        transformStamped.transform.rotation.z = q2.z();
        transformStamped.transform.rotation.w = q2.w();

        br.sendTransform(transformStamped);
      }
    }
    else {
      ang_vel_mean[X] += data.rollrate;
      ang_vel_mean[Y] += data.pitchrate;
      ang_vel_mean[Z] += data.yawrate;
      lin_acc_mean_no_grab[X] += a_b[X];
      lin_acc_mean_no_grab[Y] += a_b[Y];
      lin_acc_mean_no_grab[Z] += a_b[Z];      
      counter++;
//      ROS_INFO("%d %lf %lf %lf",counter,ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
    }
}

/*!
 * Broadcasts the IMU data in Scaled Mode and Voltage Mode in the topic data_raw.
 *
 * \brief publishImuDataAngleMode Broadcasts the IMU data in Scaled Mode and Voltage Mode
 * \param data ImuData struct
 */
void publishImuDataScaledVoltageMode(const ImuData& data) {

    geometry_msgs::TransformStamped transformStamped;

    if(!imu_started) {
      imu_started = true;
      timer.start();
      counter=0;
      ang_vel_mean[X] = ang_vel_mean[Y] = ang_vel_mean[Z] = 0;
      msg.orientation = tf2::toMsg(tf2::Quaternion(0,0,0,1));
      ROS_INFO("CACULATING BIAS: DO NOT MOVE THE IMU. THIS WILL TAKE 10 SECONDS");
    }

    if(is_initialized) {
      msg.header.stamp = data.receive_time;
      msg.header.frame_id = frame_id;
      msg.angular_velocity.x = data.rollrate - ang_vel_mean[X];
      msg.angular_velocity.y = data.pitchrate - ang_vel_mean[Y];
      msg.angular_velocity.z = data.yawrate - ang_vel_mean[Z];
//      ROS_INFO("%lf %lf %lf %lf %lf %lf",data.rollrate,data.pitchrate,data.yawrate,ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
      msg.linear_acceleration.x = data.ax;
      msg.linear_acceleration.y = data.ay;
      msg.linear_acceleration.z = data.az;

      msgmag.header.stamp = data.receive_time;
      msgmag.header.frame_id = frame_id;
      msgmag.magnetic_field.x = data.xmag;
      msgmag.magnetic_field.y = data.ymag;
      msgmag.magnetic_field.z = data.zmag;

      imu_pub.publish(msg);
      mag_pub.publish(msgmag);
    }
    else {
      ang_vel_mean[X] += data.rollrate;
      ang_vel_mean[Y] += data.pitchrate;
      ang_vel_mean[Z] += data.yawrate;
      counter++;
     //ROS_INFO("%d %lf %lf %lf",counter,ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
     // ROS_INFO("%d %lf %lf %lf",counter,data.rollrate,data.pitchrate,data.yawrate);
    }
}

/*!
 * Sets a new baudrate in the serial port RS232 communication with the IMU. Be carreful with this service, it is possible to lost
 * the communication with the IMU. To reset the baudrate to 38400 disconnect the IMU from the power and from de USB.
 *
 * \brief setBaudrate Sets a new baudrate in the serial port RS232 communication with the IMU.
 * \param xbow Reference to the xbow reader
 * \param req Server Request object
 * \param res Serever Response object
 * \return True if the server is running correctly or false if not
 */
bool setBaudrate(XBOW4X &xbow, xbow4x::set_baudrateRequest &req, xbow4x::set_baudrateResponse &res) {

  xbow.setBaudrate(req.baudrate,res.response);

  return true;
}

/*!
 *
 * Service calibrate [command]: Send commands to the IMU to different actions related with the calibration. The commands are:
 *
 *  - s: starts calibration mode. Stops the other modes (continuous or polling). While the calibration is active the other modes do not work, it is necessary to stop the calibration.
 *  - u: stops calibration mode. The message mode active after it is polling mode.
 *  - h: clear hard iron calibration data
 *  - t: clear soft iron calibration data
 *
 * \brief calibrateSrv Send commands to the IMU to different actions related with the calibration.
 * \param xbow Reference to the xbow reader
 * \param req Server Request object
 * \param res Serever Response object
 * \return True if the server is running correctly or false if not
 */
bool calibrateSrv(XBOW4X &xbow, xbow4x::calibrateRequest &req, xbow4x::calibrateResponse &res) {
  string s;

  if(is_initialized) {
    if(req.command.length()>1)
      res.response="Incorrect command";
    else
      if(xbow.calibrateCommand(req.command.c_str()[0],res.response)) {
        ROS_ERROR("%s",res.response.c_str());
        return true;
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
bool sendCommand(ros::NodeHandle &node_handle, XBOW4X &xbow, xbow4x::send_commandRequest &req, xbow4x::send_commandResponse &res) {
    string s;

    if(is_initialized) {
      if(req.command.length()>1)
        res.response="Incorrect command";
      else {
        if(xbow.sendCommand(req.command.c_str()[0],res.response)) {
          ROS_ERROR("%s",res.response.c_str());
          return true;
        }
        else {
          if(req.command == string(1,(char)xbow4x::MeasurementType::AngleMode) || req.command == string(1,(char)xbow4x::MeasurementType::ScaledMode) || req.command == string(1,(char)xbow4x::MeasurementType::VoltageMode)) {
            measurementMode = static_cast<xbow4x::MeasurementType>(req.command.c_str()[0]);
            is_initialized = false;
            imu_started = false;
            if(measurementMode == xbow4x::MeasurementType::AngleMode)
              xbow.set_data_handler(publishImuDataAngleMode);
            else
              xbow.set_data_handler(publishImuDataScaledVoltageMode);
            if(messageMode != xbow4x::MessageMode::Continous)
              if(xbow.sendCommand((u_int8_t)xbow4x::MessageMode::Continous,s))
                ROS_ERROR("%s",s.c_str());
          }
          else if(req.command == string(1,(char)xbow4x::MessageMode::Continous))
            messageMode = xbow4x::MessageMode::Continous;
          else if( req.command == string(1,(char)xbow4x::MessageMode::Poll)|| req.command == "G")
            messageMode = xbow4x::MessageMode::Poll;
          else if(req.command == "R")
            messageMode = xbow4x::MessageMode::Poll;
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
void timerCallback(ros::NodeHandle &node_handle,XBOW4X &xbow,const ros::TimerEvent&) {
  //ROS_INFO("%d %lf %lf %lf",counter,ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
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
    if(xbow.sendCommand((u_int8_t)messageMode,s))
      ROS_ERROR("%s",s.c_str());
  }
}

int main(int argc, char **argv)
{ 
  string s;

  ros::init(argc, argv, "xbow4x_node");

  ros::NodeHandle node_handle("xbow/xbow4x_node");


  imu_pub = node_handle.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  mag_pub = node_handle.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
  imu_no_grab_pub = node_handle.advertise<sensor_msgs::Imu>("imu/data_no_grab", 1);

  string port_name;
  if(!node_handle.getParam("port", port_name));
    port_name = string("/dev/ttyUSB0");
  
  int baudrate;
  if(!node_handle.getParam("baudrate", baudrate))//38400
    baudrate = 38400;

  if(!node_handle.getParam("broadcast_tf", broadcats_tf))
    broadcats_tf =true;
  
  if(!node_handle.getParam("frame_id", frame_id))
    frame_id = string("cbimu_frame");

  //Angle Mode: a Scaled Mode: c Voltage Mode: r
  if(node_handle.getParam("measurement_mode",s)) {//AngleMode
       measurementMode =static_cast<xbow4x::MeasurementType>(s.c_str()[0]);
       ROS_INFO("%s",s.c_str());
  }
  else
    measurementMode = xbow4x::MeasurementType::AngleMode;

  //Continuous Mode: C Poll Mode: P Poll/Stop Continuous Mode: G
  if(node_handle.getParam("message_mode", s)) { //Continuous Mode
    ROS_INFO("%s",s.c_str());
    messageMode =static_cast<xbow4x::MessageMode>(s.c_str()[0]);
  }
  else
    messageMode = xbow4x::MessageMode::Continous;

  //node_handle.getParam("ang_vel_mean", ang_vel_mean);
  if(!node_handle.getParam("ang_vel_mean", ang_vel_mean))
    ang_vel_mean = {0.0,0.0,0.0};
//  node_handle.param("lin_acc_mean", lin_acc_mean,{0.0,0.0,0.0});

  if(!node_handle.getParam("lin_acc_mean_no_grab", lin_acc_mean_no_grab))
    lin_acc_mean_no_grab ={0.0,0.0,0.0};
  
  std::vector<double> orientation_cov;
  if(!node_handle.getParam("orientation_cov", orientation_cov))
    orientation_cov = {1.0e6,  0.0,    0.0,
                            0.0, 1.0e6, 0.0,
                            0.0, 0.0, 1.0e6};

  for(int i = 0; i < 9; i++) {
    msg.orientation_covariance[i] = orientation_cov[i];
  }
  
  std::vector<double> ang_vel_cov;
  if(!node_handle.getParam("ang_vel_cov", ang_vel_cov))
    ang_vel_cov = {1.0e-2, 0.0,    0.0,
                   0.0,    1.0e-2, 0.0,
                   0.0,    0.0,    1.0e-2};

  for(int i = 0; i < 9; i++) {
    msg.angular_velocity_covariance[i] = ang_vel_cov[i];
  }

  std::vector<double> lin_acc_cov;
  if(!node_handle.getParam("lin_acc_cov", lin_acc_cov))
    lin_acc_cov = {1.0e-2, 0.0,    0.0,
                   0.0,    1.0e-2, 0.0,
                   0.0,    0.0,    1.0e-2};


  for(int i = 0; i < 9; i++) {
    msg.linear_acceleration_covariance[i] = lin_acc_cov[i];
  }
  
  std::vector<double> lin_acc_cov_no_grab;
  if(!node_handle.getParam("lin_acc_cov_no_grab", lin_acc_cov_no_grab))
    lin_acc_cov_no_grab = {1.0e-2, 0.0,    0.0,
                   0.0,    1.0e-2, 0.0,
                   0.0,    0.0,    1.0e-2};


  for(int i = 0; i < 9; i++) {
    msg_no_grab.linear_acceleration_covariance[i] = lin_acc_cov[i];
  }

  XBOW4X xbow;

  if(measurementMode == xbow4x::MeasurementType::AngleMode)
    xbow.set_data_handler(publishImuDataAngleMode);
  else
    xbow.set_data_handler(publishImuDataScaledVoltageMode);

  //Service declaration
  ros::ServiceServer serviceTf = node_handle.advertiseService("broadcast_tf", broadcatsTf);
  ros::ServiceServer serviceStatus = node_handle.advertiseService("status",statusSrv);
  ros::ServiceServer serviceCommand = node_handle.advertiseService<xbow4x::send_commandRequest,xbow4x::send_commandResponse>("send_command", boost::bind(&sendCommand,boost::ref(node_handle),boost::ref(xbow),_1,_2));
  ros::ServiceServer serviceCalibrate = node_handle.advertiseService<xbow4x::calibrateRequest,xbow4x::calibrateResponse>("calibrate", boost::bind(&calibrateSrv,boost::ref(xbow),_1,_2));
  ros::ServiceServer serviceSetBaudrate = node_handle.advertiseService<xbow4x::set_baudrateRequest,xbow4x::set_baudrateResponse>("set_baudrate", boost::bind(&setBaudrate,boost::ref(xbow),_1,_2));
  timer = node_handle.createTimer(ros::Duration(10), boost::bind(&timerCallback,boost::ref(node_handle),boost::ref(xbow),_1),false,false);

  if (xbow.connect(port_name, baudrate)) {

//    xbow.setBaudrate(38400,s);
//    ROS_INFO("%s",s.c_str());
    if(xbow.sendCommand('R',s)) {
      ROS_ERROR("%s",s.c_str());
      node_handle.shutdown();
      exit(1);
    }
    if(xbow.sendCommand((u_int8_t)measurementMode,s)) {
      ROS_ERROR("%s",s.c_str());
      xbow.disconnect();
      node_handle.shutdown();
      exit(1);
    }

    //First in continous mode to calculate de bias
    if(xbow.sendCommand('C',s)) {
      ROS_ERROR("%s",s.c_str());
      node_handle.shutdown();
      exit(1);
    }
    ros::spin();
    xbow.disconnect();
  } else {
    ROS_ERROR("The xbow did not connect, "
              "using port %s at baudrate %i",
              port_name.c_str(), baudrate);
  }
  return 0;
}
