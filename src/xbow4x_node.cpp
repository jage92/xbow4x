#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "xbow400/xbow4x.hpp"

#include "xbow400/srv/broadcast_tf.hpp"
#include "xbow400/srv/calibrate.hpp"
#include "xbow400/srv/send_command.hpp"
#include "xbow400/srv/set_baudrate.hpp"
#include "xbow400/srv/status.hpp"

#define X 0 //! Index of the first element of array that corresponds to x component of a point
#define Y 1 //! Index of the first element of array that corresponds to y component of a point
#define Z 2 //! Index of the first element of array that corresponds to z component of a point

class Xbow4xNode : public rclcpp::Node
{
  public:

    ~Xbow4xNode() {
      RCLCPP_INFO(this->get_logger(),"Serial Port Disconnected");

      xbow->disconnect();
    }

    Xbow4xNode() : Node("xbow4x_node")
    {
      //Xbow4x xbow(this->get_logger());
      string s;
      int baudrate;
      string port_name = "";
      std::vector<double> orientation_cov(9);
      std::vector<double> lin_acc_cov(9);
      std::vector<double> ang_vel_cov(9);
      std::vector<double> lin_acc_cov_no_grab(9);
      std::vector<double> mag_cov(9);


      ang_vel_mean = vector<double>(3);
      lin_acc_mean_no_grab = vector<double>(3);

      imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);
      mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 1);
      imu_no_grab_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_no_grab", 1);
      br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      this->declare_parameter("port", "/dev/ttyUSB0");
      port_name = this->get_parameter("port").get_parameter_value().get<string>();


      this->declare_parameter("baudrate", 38400);//38400
      baudrate = this->get_parameter("baudrate").get_parameter_value().get<int>();


      this->declare_parameter("broadcast_tf", true);
      broadcats_tf = this->get_parameter("broadcast_tf").get_parameter_value().get<bool>();

      this->declare_parameter("frame_id","cbimu_frame");
      frame_id = this->get_parameter("frame_id").get_parameter_value().get<string>();

      //Angle Mode: a Scaled Mode: c Voltage Mode: r
      this->declare_parameter("measurement_mode","a"); //AngleMode
      measurementMode = static_cast<xbow4x::MeasurementType>(this->get_parameter("measurement_mode").get_parameter_value().get<string>().c_str()[0]);

      //Continuous Mode: C Poll Mode: P Poll/Stop Continuous Mode: G
      this->declare_parameter("message_mode","C"); //Continuous Mode
      messageMode = static_cast<xbow4x::MessageMode>(this->get_parameter("message_mode").get_parameter_value().get<string>().c_str()[0]);

      this->declare_parameter("orientation_cov",vector<double>({1.0e6,  0.0,    0.0,
                                                                0.0, 1.0e6, 0.0,
                                                                0.0, 0.0, 1.0e6}));
      orientation_cov = this->get_parameter("orientation_cov").get_parameter_value().get<vector<double>>();

      std::copy_n(orientation_cov.begin(),orientation_cov.size(),msg.orientation_covariance.begin());
      std::copy_n(orientation_cov.begin(),orientation_cov.size(),msg_no_grab.orientation_covariance.begin());


      this->declare_parameter("ang_vel_cov",vector<double>({1.0e-2, 0.0,    0.0,
                                                                0.0,    1.0e-2, 0.0,
                                                                0.0,    0.0,    1.0e-2}));
      ang_vel_cov = this->get_parameter("ang_vel_cov").get_parameter_value().get<vector<double>>();

      std::copy_n(ang_vel_cov.begin(),ang_vel_cov.size(),msg.angular_velocity_covariance.begin());
      std::copy_n(ang_vel_cov.begin(),ang_vel_cov.size(),msg_no_grab.angular_velocity_covariance.begin());


      this->declare_parameter("lin_acc_cov",vector<double>({1.0e-2, 0.0,    0.0,
                                                                0.0,    1.0e-2, 0.0,
                                                                0.0,    0.0,    1.0e-2}));
      lin_acc_cov = this->get_parameter("lin_acc_cov").get_parameter_value().get<vector<double>>();

      std::copy_n(lin_acc_cov.begin(),lin_acc_cov.size(),msg.linear_acceleration_covariance.begin());


      this->declare_parameter("lin_acc_cov_no_grab",vector<double>({1.0e-2, 0.0,    0.0,
                                                                0.0,    1.0e-2, 0.0,
                                                                0.0,    0.0,    1.0e-2}));
      lin_acc_cov_no_grab = this->get_parameter("lin_acc_cov_no_grab").get_parameter_value().get<vector<double>>();

      std::copy_n(lin_acc_cov_no_grab.begin(),lin_acc_cov_no_grab.size(),msg_no_grab.linear_acceleration_covariance.begin());

      this->declare_parameter("mag_cov",vector<double>({1.0e-2, 0.0,    0.0,
                                                                0.0,    1.0e-2, 0.0,
                                                                0.0,    0.0,    1.0e-2}));
      mag_cov = this->get_parameter("mag_cov").get_parameter_value().get<vector<double>>();

      std::copy_n(mag_cov.begin(),mag_cov.size(),msgmag.magnetic_field_covariance.begin());


      broadcastTfSrv = this->create_service<xbow400::srv::BroadcastTf>("broadcast_tf",std::bind(&Xbow4xNode::broadcatsTf_srv,this,std::placeholders::_1,std::placeholders::_2));
      statusSrv = this->create_service<xbow400::srv::Status>("status",std::bind(&Xbow4xNode::status_srv,this,std::placeholders::_1,std::placeholders::_2));
      setBaudrateSrv = this->create_service<xbow400::srv::SetBaudrate>("set_baudrate",std::bind(&Xbow4xNode::setBaudrate_srv,this,std::placeholders::_1,std::placeholders::_2));
      calibrateSrv = this->create_service<xbow400::srv::Calibrate>("calibrate",std::bind(&Xbow4xNode::calibrate_srv,this,std::placeholders::_1,std::placeholders::_2));
      sendCommandSrv = this->create_service<xbow400::srv::SendCommand>("send_command",std::bind(&Xbow4xNode::sendCommand_srv,this,std::placeholders::_1,std::placeholders::_2));

      xbow = std::make_shared<xbow4x::XBOW4X>(this->get_logger());

      if (xbow->connect(port_name, baudrate)) {
          if(xbow->sendCommand('R',s)) {
            RCLCPP_ERROR(this->get_logger(),"%s",s.c_str());
            rclcpp::shutdown();
            exit(1);
          }
          if(xbow->sendCommand((u_int8_t)measurementMode,s)) {
            RCLCPP_ERROR(this->get_logger(),"%s",s.c_str());
            xbow->disconnect();
            rclcpp::shutdown();
            exit(1);
          }

          //First in continous mode to calculate de bias
          if(xbow->sendCommand((u_int8_t)xbow4x::MessageMode::Continous,s)) {
            RCLCPP_ERROR(this->get_logger(),"%s",s.c_str());
            rclcpp::shutdown();
            exit(1);
          }
          startTimer();
      }
}

    /*!
     * Broadcasts the IMU data in Angle Mode in the topics data_raw and data_no_grab. If broadcast_tf is enabled,
     * it also broadcasts the local reference system to the IMU.
     *
     * \brief publishImuDataAngleMode Broadcasts the IMU data in Angle Mode
     * \param data ImuData struct
     */
    void publishImuDataAngleMode() {
        string s;
        geometry_msgs::msg::TransformStamped transformStamped;

        tf2::Vector3 g_w(0.0,0.0,-9.81),g_b,a_b;
        tf2::Quaternion q, q_rot, q2;

        if(!imu_started) {
          imu_started = true;
          counter=0;
          ang_vel_mean[X] = ang_vel_mean[Y] = ang_vel_mean[Z] = 0;
          lin_acc_mean_no_grab[X] = lin_acc_mean_no_grab[Y] = lin_acc_mean_no_grab[Z] = 0;
          RCLCPP_INFO(this->get_logger(),"CACULATING BIAS: DO NOT MOVE THE IMU. THIS WILL TAKE 10 SECONDS");
        }

        xbow4x::ImuData data = xbow->readSerialPortPollMode();

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

          imu_pub->publish(msg);
          imu_no_grab_pub->publish(msg_no_grab);
          mag_pub->publish(msgmag);
//          RCLCPP_INFO(this->get_logger(),"%lf %lf %lf",data.ax,data.ay,data.az);
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

            br->sendTransform(transformStamped);
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
    //      RCLCPP_INFO(this->get_logger(),"%d %lf %lf %lf",counter,ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
          if(counter == 1000) {
            is_initialized = true;

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
              if(xbow->sendCommand((u_int8_t)messageMode,s))
                RCLCPP_ERROR(this->get_logger(),"%s",s.c_str());
              stopTimer();
            }
          }
        }
    }

    /*!
     * Broadcasts the IMU data in Scaled Mode and Voltage Mode in the topic data_raw.
     *
     * \brief publishImuDataAngleMode Broadcasts the IMU data in Scaled Mode and Voltage Mode
     * \param data ImuData struct
     */
    void publishImuDataScaledVoltageMode() {

        string s;

        if(!imu_started) {
          imu_started = true;
          counter=0;
          ang_vel_mean[X] = ang_vel_mean[Y] = ang_vel_mean[Z] = 0;
          msg.orientation = tf2::toMsg(tf2::Quaternion(0,0,0,1));
          RCLCPP_INFO(this->get_logger(),"CACULATING BIAS: DO NOT MOVE THE IMU. THIS WILL TAKE 10 SECONDS");
        }

        xbow4x::ImuData data = xbow->readSerialPortPollMode();

        if(is_initialized) {
          msg.header.stamp = data.receive_time;
          msg.header.frame_id = frame_id;
          msg.angular_velocity.x = data.rollrate - ang_vel_mean[X];
          msg.angular_velocity.y = data.pitchrate - ang_vel_mean[Y];
          msg.angular_velocity.z = data.yawrate - ang_vel_mean[Z];
    //RCLCPP_INFO(this->get_logger(),"%lf %lf %lf %lf %lf %lf",data.rollrate,data.pitchrate,data.yawrate,ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
          msg.linear_acceleration.x = data.ax;
          msg.linear_acceleration.y = data.ay;
          msg.linear_acceleration.z = data.az;

          msgmag.header.stamp = data.receive_time;
          msgmag.header.frame_id = frame_id;
          msgmag.magnetic_field.x = data.xmag;
          msgmag.magnetic_field.y = data.ymag;
          msgmag.magnetic_field.z = data.zmag;

          imu_pub->publish(msg);
          mag_pub->publish(msgmag);
        }
        else {
          ang_vel_mean[X] += data.rollrate;
          ang_vel_mean[Y] += data.pitchrate;
          ang_vel_mean[Z] += data.yawrate;
          counter++;
    //RCLCPP_INFO(this->get_logger(),"%d %lf %lf %lf",counter,ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
         // RCLCPP_INFO(this->get_logger(),"%d %lf %lf %lf",counter,data.rollrate,data.pitchrate,data.yawrate);

          //ROS_INFO("%d %lf %lf %lf",counter,ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);
          if(counter == 1000) {
            is_initialized = true;

            ang_vel_mean[X] /=counter;
            ang_vel_mean[Y] /=counter;
            ang_vel_mean[Z] /=counter;
            RCLCPP_INFO(this->get_logger(),"ANG_RATE_BIAS: %lf %lf %lf",ang_vel_mean[X],ang_vel_mean[Y],ang_vel_mean[Z]);

            if(messageMode != xbow4x::MessageMode::Continous) {
              if(xbow->sendCommand((u_int8_t)messageMode,s))
                RCLCPP_ERROR(this->get_logger(),"%s",s.c_str());
              stopTimer();
            }
          }
        }
    }

    /*!
     * Returns the status (Measurement Mode and Message Mode) of the IMU. It uses the format Mode:[Mode Name]-[Command] in order to facilitate its parsing.
     *
     * \brief statusSrv Returns the status (Measurement Mode and Message Mode) of the IMU.
     * \param req Server Requets object
     * \param res Server Response object
     * \return The status in format Mode:[Mode Name]-[Command]
     */
    bool status_srv(const std::shared_ptr<xbow400::srv::Status::Request> req, const std::shared_ptr<xbow400::srv::Status::Response> res) {

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

      res->response = string("Measurement Mode:")+ strMeasurementMode + string(" Message Mode:")+strMessageMode;
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
    bool broadcatsTf_srv(const std::shared_ptr<xbow400::srv::BroadcastTf::Request> req, const std::shared_ptr<xbow400::srv::BroadcastTf::Response> res) {
      broadcats_tf = req->active;
      req->active ? res->response="TF Active" : res->response="TF Inactive";
      return true;
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
    bool setBaudrate_srv(const std::shared_ptr<xbow400::srv::SetBaudrate::Request> req, const std::shared_ptr<xbow400::srv::SetBaudrate::Response> res) {
      stopTimer();
      if(!xbow->setBaudrate(req->baudrate,res->response)) {
        if(messageMode == xbow4x::MessageMode::Continous)
          startTimer();
      }
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
    bool calibrate_srv(const std::shared_ptr<xbow400::srv::Calibrate::Request> req, const std::shared_ptr<xbow400::srv::Calibrate::Response> res) {
      string s;

      if(is_initialized) {
        if(req->command.length()>1)
          res->response="Incorrect command";
        else
          if(xbow->calibrateCommand(req->command.c_str()[0],res->response)) {
            RCLCPP_INFO(this->get_logger(),"%s",res->response.c_str());
            return true;
          }
          stopTimer();
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
    bool sendCommand_srv(const std::shared_ptr<xbow400::srv::SendCommand::Request> req, const std::shared_ptr<xbow400::srv::SendCommand::Response> res) {
        string s;

        if(is_initialized) {
          if(req->command.length()>1)
            res->response="Incorrect command";
          else {
            if(xbow->sendCommand(req->command.c_str()[0],res->response)) {
              RCLCPP_ERROR(this->get_logger(),"%s",res->response.c_str());
              return true;
            }
            else {
              if(req->command == string(1,(char)xbow4x::MeasurementType::AngleMode) || req->command == string(1,(char)xbow4x::MeasurementType::ScaledMode) || req->command == string(1,(char)xbow4x::MeasurementType::VoltageMode)) {
                measurementMode = static_cast<xbow4x::MeasurementType>(req->command.c_str()[0]);
                is_initialized = false;
                imu_started = false;
                startTimer();
                if(messageMode != xbow4x::MessageMode::Continous)
                  if(xbow->sendCommand((u_int8_t)xbow4x::MessageMode::Continous,s))
                    RCLCPP_ERROR(this->get_logger(),"%s",s.c_str());
              }
              else if(req->command == string(1,(char)xbow4x::MessageMode::Continous)) {
                messageMode = xbow4x::MessageMode::Continous;
                startTimer();
              }
              else if( req->command == string(1,(char)xbow4x::MessageMode::Poll)|| req->command == "G") {
                messageMode = xbow4x::MessageMode::Poll;
                stopTimer();
                if(measurementMode == xbow4x::MeasurementType::AngleMode)
                  publishImuDataAngleMode();
                else
                  publishImuDataScaledVoltageMode();
              }
              else if(req->command == "R")
                messageMode = xbow4x::MessageMode::Poll;
            }
          }
        }
        else
          res->response="The IMU is initializing, please try again later";
        return true;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub; //! Publisher object of the IMU data (angular rate, acceleration, orientation)
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_no_grab_pub; //! Publisher object of the IMU data (angular rate whiout gravity, acceleration, orientation)
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub; //! Publisher object of the magnetometer (magnetic field vector)

    sensor_msgs::msg::Imu msg, msg_no_grab; //! Message to public the IMU topic
    sensor_msgs::msg::MagneticField msgmag; //! Message to public the Magnetometec topic

    std::vector<double> ang_vel_mean; //! Array of the algular rate bias
    std::vector<double>  lin_acc_mean_no_grab; //! Array of the linear acceleration whiout gravity

    xbow4x::MeasurementType measurementMode; //! Enum object of the IMU measurement type (AngleMode, ScaledMode, VoltageMode) active
    xbow4x::MessageMode messageMode; //! Enum object of the IMU message mode (Continous or Poll) active

    bool is_initialized=false; //! Flag that indicates if the xbow is initialized
    bool imu_started=false; //! Flag that indicates if the xbow is strated
    bool broadcats_tf; //! Flag that indicates if the IMU reference frame is published or not
    string frame_id; //! Name of the IMU frame
    int counter=0;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    std::shared_ptr<xbow4x::XBOW4X> xbow;
    rclcpp::Service<xbow400::srv::BroadcastTf>::SharedPtr broadcastTfSrv;
    rclcpp::Service<xbow400::srv::Calibrate>::SharedPtr calibrateSrv;
    rclcpp::Service<xbow400::srv::SendCommand>::SharedPtr sendCommandSrv;
    rclcpp::Service<xbow400::srv::SetBaudrate>::SharedPtr setBaudrateSrv;
    rclcpp::Service<xbow400::srv::Status>::SharedPtr statusSrv;


    void startTimer() {
      if(measurementMode == xbow4x::MeasurementType::AngleMode)
        timer_ = this->create_wall_timer(8ms, std::bind(&Xbow4xNode::publishImuDataAngleMode,this));
      else
        timer_ = this->create_wall_timer(8ms, std::bind(&Xbow4xNode::publishImuDataScaledVoltageMode,this));
    }

    void stopTimer() {
      timer_->cancel();
      //timer_= nullptr;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Xbow4xNode>());
  rclcpp::shutdown();

  return 0;
}

