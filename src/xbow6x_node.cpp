#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/Float64.h>

#include "xbow6x/xbow6x.h"
using namespace xbow6x;
using std::string;

ros::Publisher imu_pub;
string frame_id;
sensor_msgs::Imu msg;
std::vector<double> ang_vel_mean;
std::vector<double> lin_acc_mean;

void PublishImuData(const ImuData& data) {
    msg.header.stamp = data.receive_time;
    msg.header.frame_id = frame_id;
    msg.angular_velocity.x = data.rollrate - ang_vel_mean[0];
    msg.angular_velocity.y = data.pitchrate - ang_vel_mean[1];
    msg.angular_velocity.z = data.yawrate - ang_vel_mean[2];
    msg.linear_acceleration.x = data.ax - lin_acc_mean[0];
    msg.linear_acceleration.y = data.ay - lin_acc_mean[1];
    msg.linear_acceleration.z = data.az - lin_acc_mean[2];
    msg.orientation.x = 1;
    imu_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xbow6x_node");
  ros::NodeHandle node_handle("~");

  imu_pub = node_handle.advertise<sensor_msgs::Imu>("imu/data_raw", 1);

  string port_name;
  node_handle.param("port", port_name, string("/dev/ttyUSB0"));
  
  int baudrate;
  node_handle.param<int>("baudrate", baudrate, 38400);
  
  node_handle.param<string>("frame_id", frame_id, string("cbimu_frame"));
  
  node_handle.getParam("ang_vel_mean", ang_vel_mean);

  node_handle.getParam("lin_acc_mean", lin_acc_mean);
  
  std::vector<double> orientation_cov;
  node_handle.getParam("orientation_cov", orientation_cov);
  for(int i = 0; i < 9; i++) {
    msg.orientation_covariance[i] = orientation_cov[i];
  }
  
  std::vector<double> ang_vel_cov;
  node_handle.getParam("ang_vel_cov", ang_vel_cov);
  for(int i = 0; i < 9; i++) {
    msg.angular_velocity_covariance[i] = ang_vel_cov[i];
  }
  
  std::vector<double> lin_acc_cov;
  node_handle.getParam("lin_acc_cov", lin_acc_cov);
  for(int i = 0; i < 9; i++) {
    msg.linear_acceleration_covariance[i] = lin_acc_cov[i];
  }
  
  XBOW6X xbow;
  xbow.set_data_handler(PublishImuData);
  
  if (xbow.Connect(port_name, baudrate)) {
    ros::spin();
    xbow.Disconnect();
  } else {
    ROS_ERROR("The xbow did not connect, "
              "using port %s at baudrate %i",
              port_name.c_str(), baudrate);
  }

  return 0;
}
