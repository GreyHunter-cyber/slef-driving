#include <ros/ros.h>
#include <vector>
#include <string>

#include "csv/csv.h"

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Path.h"
#include "tf/transform_datatypes.h"

#include "plan_msgs/PointSYK.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#define PI 3.14159265

#define dist(x0, y0, x1, y1) sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0))

namespace csv = io;

nav_msgs::Path path;
nav_msgs::Path Gpath;
bool tarFlag = false;
ros::Publisher reset_point_pub;
void adjustAngle(double yaw)
{
  if(yaw > PI)
    yaw -= 2*PI;
  if(yaw < -PI)
    yaw += 2*PI;
}
bool getTargetTraj(nav_msgs::Path& Gtraj, geometry_msgs::PoseStamped tarPos){
  Gtraj.poses.clear();
  int trajSize = path.poses.size();
  if(trajSize < 1) return false;
  double closest_distance = std::numeric_limits<double>::max();
  int closest_waypoint_index = -1;
  
  double Troll, Tpitch, Tyaw;
  geometry_msgs::Quaternion geoQuat = tarPos.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(Troll, Tpitch, Tyaw);
  adjustAngle(Tyaw);
  std::cout<< "targetPose: x "<<tarPos.pose.position.x
                       <<" y "<<tarPos.pose.position.y
                       <<" yaw "<<Tyaw<<std::endl;
  tarPos.pose.orientation.z = Tyaw;

  for(int i = 0; i < trajSize; ++i)
  {
  	double traj_x = path.poses[i].pose.position.x;
  	double traj_y = path.poses[i].pose.position.y;  
  	double current_distance = dist(tarPos.pose.position.x,tarPos.pose.position.y,traj_x,traj_y);  
  	if(current_distance < closest_distance)
  	{
  		closest_distance = current_distance;
  		closest_waypoint_index = i;
  	}
  }
  if(closest_waypoint_index == (trajSize - 1))
  {
    Gtraj = path;
    Gtraj.poses.push_back(tarPos);
  }else
  {
    ROS_INFO("yes");
    for(int i = 0; i <= closest_waypoint_index; ++i)
    {
      Gtraj.poses.push_back(path.poses[i]);
    }
    Gtraj.poses.push_back(tarPos);
  }
  
  return true;
}

geometry_msgs::PoseStamped get_near_trajpoint(nav_msgs::Path& local_plan, double cx, double cy){
  std::vector<geometry_msgs::PoseStamped>& _local_plan = local_plan.poses;
  if(_local_plan.size()<1) return _local_plan[0];
  double closest_distance = std::numeric_limits<double>::max();
  int closest_waypoint_index = -1;  
  for(int i = 0; i < _local_plan.size();++i)
  {
  	double traj_x = _local_plan[i].pose.position.x;
  	double traj_y = _local_plan[i].pose.position.y;  
  	double current_distance = dist(cx,cy,traj_x,traj_y);  
  	if(current_distance < closest_distance)
  	{
  		closest_distance = current_distance;
  		closest_waypoint_index = i;
  	}
  }
  return _local_plan[closest_waypoint_index];
}

void reset_point_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    plan_msgs::PointSYK repoint;
    geometry_msgs::PoseStamped tpoint = get_near_trajpoint(path, msg.pose.pose.position.x, msg.pose.pose.position.y);
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = msg.pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    repoint.x = msg.pose.pose.position.x;
    repoint.y = msg.pose.pose.position.y;
    repoint.s = tpoint.pose.position.z;
    repoint.yaw = yaw;
    repoint.ks = 1.0;
    reset_point_pub.publish(repoint);

}
void tarFlag_callback(std_msgs::Bool msg)
{
  tarFlag = msg.data;
}
void targetPos_callback(geometry_msgs::PoseStamped msg)
{
  if(!getTargetTraj(Gpath,msg))
  {
    ROS_WARN("NO USABLE PATH DATA!");
  }
  ROS_INFO("is target: %d",tarFlag);
}
//可以考虑用共享内存来处理全局路径的问题
int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_path_sim");
  std::string global_path = "";
  // double s_resolution = 0.5;
  ros::NodeHandle nhPrivate("~");
  

  nhPrivate.param<std::string>("global_path_csv", global_path, "/home/zxkj/zxkjCode/c++/iau_robot_indoor2/map/junce_zxkj_garage002.csv");
  ros::NodeHandle nh;
  ros::Subscriber tarPos_sub = nh.subscribe("/move_base_simple/goal", 5, &targetPos_callback);/////////////////////////////////TODO
  ros::Subscriber tarFlag_sub = nh.subscribe("/isMission", 5, &tarFlag_callback);/////////////////////////////////TODO
  ros::Publisher path_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 1, true);
  ros::Subscriber reset_point_sub = nh.subscribe("/initialpose", 1, &reset_point_callback);
  reset_point_pub = nh.advertise<plan_msgs::PointSYK>("/reset_point", 1, true);

  //读取路径文件中的点
  std::vector<double> points_x_origin, points_y_origin,points_z_origin,points_yaw_origin,points_s_origin;
  csv::CSVReader<4, io::trim_chars<>, io::no_quote_escape<' '>> hd_map(global_path);
  hd_map.read_header(io::ignore_extra_column, "x", "y","z","yaw");
  // parse map:
  double s=0, x, y,z,yaw;
  points_s_origin.push_back(s);
  int count = 0;
  while(hd_map.read_row(x, y,z,yaw)){
    points_x_origin.push_back(x);
    points_y_origin.push_back(y);
    points_z_origin.push_back(z);
    adjustAngle(yaw);
    points_yaw_origin.push_back(yaw);

    if(count >=1){
      s += sqrt((x -points_x_origin[count-1])*(x -points_x_origin[count-1])+(y -points_y_origin[count-1])*(y -points_y_origin[count-1]));
      points_s_origin.push_back(s);
    }
    count++;
  }
  int sz = points_s_origin.size();
  if(sz == 0)
  {
    std::cout << "no data read from file!"<<std::endl;
    return -1;
  }
  //抽希(如果不抽希，容易过拟合，导致曲线不够平滑)
  std::vector<double> points_x, points_y, points_s,points_yaw;
  double prev_s = points_s_origin[0];
  points_s.push_back(points_s_origin[0]);
  points_x.push_back(points_x_origin[0]);
  points_y.push_back(points_y_origin[0]);
  points_yaw.push_back(points_yaw_origin[0]);
  bool need_add_lastpoint = true;
  for(int i = 1; i< sz; ++i)
  {
    if(points_s_origin[i]-prev_s > 1.5)//5m一个点
    {
      points_s.push_back(points_s_origin[i]);
      points_x.push_back(points_x_origin[i]);
      points_y.push_back(points_y_origin[i]);
      points_yaw.push_back(points_yaw_origin[i]);
      prev_s = points_s_origin[i];      
    }
    if(i == sz-1){
        need_add_lastpoint = false;
    }
  }
  if(need_add_lastpoint){
      points_s.push_back(points_s_origin[sz-1]);
      points_x.push_back(points_x_origin[sz-1]);
      points_y.push_back(points_y_origin[sz-1]);
      points_yaw.push_back(points_yaw_origin[sz-1]);
  }
  sz = points_s.size();
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  for(int i = 0; i< sz; ++i)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = points_x[i];
    pose_msg.pose.position.y = points_y[i];
    pose_msg.pose.position.z = 0;
    pose_msg.pose.orientation.z = points_yaw[i];
    path.poses.push_back(pose_msg);
  }
  Gpath = path;
  std::cout << sz <<std::endl;
  ros::Rate loop_rate(3);
  while(ros::ok())
  {
    if(path_pub_.getNumSubscribers()){
      if(!tarFlag)
        {path_pub_.publish(path);}
      else
       { path_pub_.publish(Gpath);}
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
