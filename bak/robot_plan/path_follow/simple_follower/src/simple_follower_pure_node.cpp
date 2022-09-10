#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Bool.h>

#include <mutex>
#include <math.h>
#include <iostream>
#include <stdio.h> 
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include "plan_msgs/PointXY.h"

#include <deque>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#define dist(x1, y1, x2, y2) sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
#define PI 3.14159265
float max_angle_vel = 1.57;
float max_linear_vel = 1.0;
//float sk = 0.15;
//todo
bool pause_flag = false;
bool follow_flag = false;
float sk = 0.1;//速度p控制
float Lfc_S = 2.0;//前视距离 m
float Lfc_L =4.0;
float k = 0.1;//前视距离系数
float car_L = 0.68;//车辆轴距 m
std::vector<geometry_msgs::PoseStamped> local_plan;

int traj_idx = 0;
std::mutex  plan_mutex;
int traj_lenth = 0;
double prev_x = 0.0;
double prev_y = 0.0;
double curr_x = 0.0;
double curr_y = 0.0;
double curr_yaw = 0.0;
double curr_speed = 0.0;

double last_odom_time = 0.0;
double last_plan_time = 0.0;
double dt = 0.0;
double acc = 0.5;

double aim_velocity = 0;
double velocity = 0;
double omega = 0;
float Lf = 0;
int id=0;
int no_traj_count = 0;
ros::Publisher ctrl_pub;
ros::Publisher vis_pub;
ros::Publisher pub;
visualization_msgs::Marker marker;

bool remoteFlag = false;

bool firstOdom = false;
std::deque<geometry_msgs::Point> curPoses;
double first_odom_time = 0;
bool get_next_trajpoint(std::vector<geometry_msgs::PoseStamped>& _local_plan, int& c_idx, double cx, double cy){
  if(_local_plan.size()<2) return false;
  double closest_distance = std::numeric_limits<double>::max();
  int closest_waypoint_index = -1;  
  for(int i = 0; i < _local_plan.size(); i++)
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
  if(closest_waypoint_index == local_plan.size()-1)  return false;//false 终点停车,c_idx = 0,不停
  else c_idx = closest_waypoint_index;

  float L = 0.0;
  Lf = Lfc_L;
  while(Lf > L && (c_idx + 1) < _local_plan.size()){
      L += dist(_local_plan[c_idx].pose.position.x,_local_plan[c_idx].pose.position.y,
            _local_plan[c_idx + 1].pose.position.x,_local_plan[c_idx + 1].pose.position.y);
      c_idx += 1;
      }
 float d_y = fabs(_local_plan[c_idx].pose.position.y - _local_plan[closest_waypoint_index].pose.position.y);
 float d_x = fabs(_local_plan[c_idx].pose.position.x - _local_plan[closest_waypoint_index].pose.position.x);
 
  // std::cout<<"dx = "<<d_x<<"dy = "<<d_y<<std::endl;
  if(d_y>0.4 && d_x>0.4)
  {
    L = 0.0;
    Lf = Lfc_S;
    c_idx = closest_waypoint_index;
    while(Lf > L && (c_idx + 1) < _local_plan.size()){
      L += dist(_local_plan[c_idx].pose.position.x,_local_plan[c_idx].pose.position.y,
            _local_plan[c_idx + 1].pose.position.x,_local_plan[c_idx + 1].pose.position.y);
      c_idx += 1;
      }
  }
  
//发布预瞄点
geometry_msgs::Point p;
p.x =  _local_plan[c_idx].pose.position.x;
p.y =  _local_plan[c_idx].pose.position.y;
p.z =  _local_plan[c_idx].pose.position.z;
 
marker.points.clear();
//marker.pose.position.x = _local_plan[c_idx].pose.position.x;
//marker.pose.position.y = _local_plan[c_idx].pose.position.y;
marker.scale.x = 0.2;
marker.scale.y = 0.2;
marker.scale.z = 0.2;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 1.0;
marker.color.g = 0.0;
marker.color.b = 0.0;
marker.points.push_back(p);
//only if using a MESH_RESOURCE marker type:
vis_pub.publish( marker );
id++;
  return true;
}
void pause_callback(const std_msgs::Bool::ConstPtr& bool_msg)
{
  pause_flag = bool_msg->data;
}
void navFlag_callback(const std_msgs::Bool::ConstPtr& bool_msg)
{
  follow_flag = bool_msg->data;
}
void remoteControlCallback(const geometry_msgs::Twist msg)
{
  follow_flag = false;
  remoteFlag = true;
  ctrl_pub.publish(msg);
}

void traj_callback(const nav_msgs::Path::ConstPtr& traj_msg){
    last_plan_time = ros::Time::now().toSec();
  //  std::lock_guard<std::mutex> lock(plan_mutex);
    local_plan = traj_msg->poses;
    traj_idx = 0;
}

void adjust_speed()
{
  if(fabs(aim_velocity) <= 1.0e-3)
  {
   velocity = 0;
  }
  else
  {
    if((aim_velocity-curr_speed) > 0.15)//加速
    {
      velocity += sk*max_linear_vel*dt;
      if( velocity > aim_velocity ) 
      {
        velocity = aim_velocity;
      }
    }
    else if((aim_velocity-curr_speed) < -0.15)//减速
    {
      velocity -= sk*max_linear_vel*dt;

      if(velocity < aim_velocity ) 
      {
        velocity = aim_velocity;
      }
    }
    else 
    {
      velocity = aim_velocity;
    }
  }
  std::cout<<"aim v: "<<aim_velocity<<" v: "<<velocity<<"curSpeed: "<<curr_speed<<std::endl;

}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  if(!firstOdom)
  {
    first_odom_time =last_odom_time= odom_msg->header.stamp.toSec();
    curPoses.push_back(odom_msg->pose.pose.position);
    firstOdom = true;
    return;
  }
  if((odom_msg->header.stamp.toSec() - first_odom_time) < 1.0)
  {
    curPoses.push_back(odom_msg->pose.pose.position);
    last_odom_time = odom_msg->header.stamp.toSec();
    return;
  }

  {
    dt = odom_msg->header.stamp.toSec() - last_odom_time;
    curr_speed = dist(curPoses.front().x,curPoses.front().y,curPoses.back().x,curPoses.back().y);

    last_odom_time = odom_msg->header.stamp.toSec();
    curPoses.push_back(odom_msg->pose.pose.position);
    curPoses.pop_front();
    // ROS_INFO("speed: %f",curr_speed);
  }

  curr_x = odom_msg->pose.pose.position.x;
  curr_y = odom_msg->pose.pose.position.y;
  Eigen::Quaterniond quaternion(odom_msg->pose.pose.orientation.w,odom_msg->pose.pose.orientation.x,
                                odom_msg->pose.pose.orientation.y,odom_msg->pose.pose.orientation.z);
  Eigen::Matrix3d rotMat = quaternion.matrix();
  double a12 = rotMat(0, 1);
  double a22 = rotMat(1, 1);
  curr_yaw = atan2(a22, a12) - PI/2.0;//与头朝向夹角, 即与x轴夹角(顺时针为负，逆时针为正)
  if(curr_yaw > PI) curr_yaw -= 2*PI;
  if(curr_yaw <-PI) curr_yaw += 2*PI;
  
  bool ret = false;
  //根据是否有障碍物判断是否跟随frenet

    //std::lock_guard<std::mutex> lock(plan_mutex);
    ret = get_next_trajpoint(local_plan, traj_idx, curr_x, curr_y);
    //traj_idx = 2;
    //std::cout <<local_plan.size()<<", "<<traj_idx <<std::endl;
    if(ret){
        
      //遇到无路径可行时多停两秒,no_traj_count=20
      int stop_time = 10;
      if(no_traj_count != 0 && no_traj_count < stop_time)
      {
        no_traj_count++;
        omega = 0;
        aim_velocity = 0;
      }
      else
      {
        no_traj_count = 0;
 
      geometry_msgs::Pose tar_pose = local_plan[traj_idx].pose;

      float alpha = atan2(tar_pose.position.y - curr_y, tar_pose.position.x - curr_x ) - curr_yaw;
      if(curr_speed<0) alpha = PI - alpha;
      omega = atan2(2.0*car_L*sin(alpha)/Lf,1.0);
      
      aim_velocity = max_linear_vel;
      if(fabs(omega)>0.05 || pause_flag)
	    {
        aim_velocity=0.5*max_linear_vel;
      }
      // std::cout<<"max speed = "<<max_linear_vel<<std::endl;
    }
  }
  else
  {
    omega = 0.0;
    aim_velocity = 0.0;
    no_traj_count++;
  }
//调速
  adjust_speed();

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = velocity;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = omega;//

 if(!follow_flag)
 {
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;// 
 }
  if(follow_flag)
  {  
    ctrl_pub.publish(cmd_vel);
  }
  //  fprintf(outfile,"%f %f %f\n",cmd_vel.angular.z,tj_angle[9],angle_error);//调试用


}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_pure_pursuit");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("/combined_to_init", 1, &odom_callback);
  //ros::Subscriber traj_sub = nh.subscribe("/follow_path", 5, &traj_callback);
  ros::Subscriber traj2_sub = nh.subscribe("/best_traj", 5, &traj_callback);
  ros::Subscriber pause_sub = nh.subscribe("/flag_obstacle", 5, &pause_callback);/////////////////////////////////TODO
  ros::Subscriber stop_sub = nh.subscribe("/Navgation", 5, &navFlag_callback);/////////////////////////////////TODO
  ros::Subscriber remote_sub = nh.subscribe("/remote/cmd_vel", 5, &remoteControlCallback);/////////////////////////////////TODO

  ctrl_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  vis_pub = nh.advertise<visualization_msgs::Marker>( "/follow_node", 1 );
  ros::NodeHandle private_nh("~");
  private_nh.param<float>("max_angle_velocity", max_angle_vel, 0.75);
  private_nh.param<float>("max_linear_velocity", max_linear_vel, 1.0);
  private_nh.param<float>("Lfc_L", Lfc_L, 4.0);
  private_nh.param<float>("Lfc_S", Lfc_S, 2.0);

  private_nh.param<float>("k", k, 0.1);

  private_nh.param<float>("sk", sk, 0.15);

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  last_odom_time = ros::Time::now().toSec();
  
  ros::spin();
  return 0;   
}
