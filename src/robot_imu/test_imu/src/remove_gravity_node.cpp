#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <iostream>

using namespace std;

ros::Publisher newPub;

void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn){
  if( imuIn->orientation.x == 0 && imuIn->orientation.y == 0 &&
      imuIn->orientation.z == 0 && imuIn->orientation.w == 0 )
  {
    ROS_WARN_THROTTLE(1, "invalid IMU orientation. rejected");
    return;
  }
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  float accY = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.79;
  float accZ = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.79;
  float accX = imuIn->linear_acceleration.x + sin(pitch) * 9.79;
  sensor_msgs::Imu imuOut;
  imuOut.header = imuIn->header;
  imuOut.orientation = imuIn->orientation;
  imuOut.angular_velocity = imuIn->angular_velocity;
  imuOut.linear_acceleration.x = accX;
  imuOut.linear_acceleration.y = accY;
  imuOut.linear_acceleration.z = accZ;
  newPub.publish(imuOut);
}

int main(int argc, char** argv){
  ros::init(argc,argv,"remove_gravity_node");
  ros::NodeHandle nh;

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>(
      "/imu/data", 50, &imuHandler);

  newPub = nh.advertise<sensor_msgs::Imu>("/imu/no_gravity", 10);
  ros::spin();
  return 0;
}
