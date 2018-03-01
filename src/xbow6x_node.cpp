#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include "xbow6x/xbow6x.h"
using namespace xbow6x;
using std::string;

ros::Publisher imu_pub;
string frame_id;

void PublishImuData(const ImuData& data) {
    sensor_msgs::Imu msg;
    msg.header.stamp.fromSec(data.receive_time);
    msg.header.frame_id = frame_id;
    msg.angular_velocity.x = data.rollrate;
    msg.angular_velocity.y = data.pitchrate;
    msg.angular_velocity.z = data.yawrate;
    msg.linear_acceleration.x = data.ax;
    msg.linear_acceleration.y = data.ay;
    msg.linear_acceleration.z = data.az;
    msg.orientation.x = 1;
    imu_pub.publish(msg);
}

double GetTime() {
    return ros::Time::now().toSec();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xbow6x_node");
  ros::NodeHandle node_handle("~");

  imu_pub = node_handle.advertise<sensor_msgs::Imu>("imu/data", 1);

  string port_name;
  node_handle.param("port", port_name, string("/dev/ttyUSB0"));
  int baudrate;
  node_handle.param<int>("baudrate", baudrate, 38400);
  node_handle.param<string>("frame_id", frame_id, string("imu_frame"));

  XBOW6X xbow;
  xbow.set_time_handler(GetTime);
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
