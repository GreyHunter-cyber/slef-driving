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


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#define dist(x1, y1, x2, y2) sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
#define PI 3.1415926
float max_angle_vel =0.9 ;
float max_linear_vel = 1.0;
float  kp=2.0, ki=1.0, kd=12.0;
//todo sk = 0.15,
bool follow_flag = true;
bool pause_flag = true;

double intg_err = 0;//
int count =0;//
int count_num=0;
double angle_error_bef=0;//
double tj_angle[6];

double tj_best=0.0,sum_omega=0;//
double first_tj_angle=0,sec_tj_angle=0,cur_tj_angle=0;
 double angle_error=0;

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

bool first_odom= true;
double last_odom_time = 0.0;
double last_plan_time = 0.0;
double dt = 0.0;
double acc = 0.5;

double aim_velocity = 0;
double velocity = 0;
double omega = 0;
int no_traj_count = 0;
ros::Publisher  ctrl_pub;

 //调试用
// char *fname = "/home/luoluolll/songling_project/test_can/iau_robot_indoor2/map/omega.csv";
// FILE *outfile=fopen(fname,"w");

//计算规划路径距离当前位置最近的点的索引
bool get_next_trajpoint(std::vector<geometry_msgs::PoseStamped>& _local_plan, int& c_idx, double cx, double cy){
  if(_local_plan.size()<1) return false;
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
  if(closest_waypoint_index == local_plan.size()-1)  c_idx=0;//false 终点停车,c_idx = 0,不停
  else c_idx = closest_waypoint_index;
  return true;
}
void pause_callback(const std_msgs::Bool::ConstPtr& bool_msg)
{
  pause_flag = bool_msg->data;
}
void work_callback(const std_msgs::Bool::ConstPtr& bool_msg)
{
  follow_flag = bool_msg->data;
}

void traj_callback(const nav_msgs::Path::ConstPtr& traj_msg){
    last_plan_time = ros::Time::now().toSec();
  //  std::lock_guard<std::mutex> lock(plan_mutex);
    local_plan = traj_msg->poses;
    traj_idx = 0;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  dt = odom_msg->header.stamp.toSec()*1000 - last_odom_time;
  //printf("%f",dt);
  dt = dt/1000;
  last_odom_time = odom_msg->header.stamp.toSec()*1000;

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
  if(first_odom)
  {
    curr_speed = 0.0;
    first_odom = false;
  }
  else
  {
    curr_speed = sqrt(pow(curr_x -prev_x,2)+pow(curr_y-prev_y, 2));
  }
  prev_x = curr_x;
  prev_y = curr_y;
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
        //std::lock_guard<std::mutex> lock(plan_mutex);
        //取最近点前面20%的点todo:
        int next_idx = traj_idx+1;
        // double f_dis = dist(local_plan[traj_idx].pose.position.x,local_plan[traj_idx].pose.position.y,local_plan[next_idx].pose.position.x,local_plan[next_idx].pose.position.y);
        // while(f_dis < 1.0 && next_idx < local_plan.size())
        // {
        //   next_idx++;
        //   f_dis = dist(local_plan[traj_idx].pose.position.x,local_plan[traj_idx].pose.position.y,local_plan[next_idx].pose.position.x,local_plan[next_idx].pose.position.y);
        // }
        //int next_idx = floor(local_plan.size()/5+1)+traj_idx;
        if(next_idx > local_plan.size()-1) next_idx = local_plan.size()-1;
        geometry_msgs::Pose prev_trajpoint;
        geometry_msgs::Pose next_trajpoint;
        double next_pre ;
        //尝试根据距离选下一个跟踪点.
        for(int i = 0; i < floor(local_plan.size()/5+1);++i)
        {
          prev_trajpoint = local_plan[traj_idx].pose;
          next_trajpoint = local_plan[next_idx+i].pose;
          next_pre = dist(prev_trajpoint.position.x,prev_trajpoint.position.y,
                        next_trajpoint.position.x,next_trajpoint.position.y)+1.0e-15;
          if(next_pre>=0.9)
          {
            next_idx +=i;
            break; 
          }
        }
        /*横向距离误差判断*/
        // double next_cur = dist(curr_x,curr_y,next_trajpoint.position.x,next_trajpoint.position.y);
        // double cur_pre = dist(prev_trajpoint.position.x,prev_trajpoint.position.y,curr_x,curr_y);
        // double p = (next_pre+next_cur+cur_pre)/2.0;
        // double s = sqrt(p*(p-cur_pre)*(p-next_cur)*(p-next_pre));
        // double dlta_l = 2 * s / next_pre;
        // double left_or_right_s = (prev_trajpoint.position.x - curr_x)*(next_trajpoint.position.y - curr_y)-
        //                         (prev_trajpoint.position.y - curr_y)*(next_trajpoint.position.x - curr_x);
        // if(dlta_l>0.75)dlta_l=0.75;
        // if (left_or_right_s<0) dlta_l = -dlta_l;
        // //printf("dd = %f",dlta_l);
        Eigen::Vector2d vec_prevnext, vec_prevrob, vec_nextrob;
        vec_prevnext << next_trajpoint.position.x - prev_trajpoint.position.x,
                      next_trajpoint.position.y - prev_trajpoint.position.y;
        vec_prevrob  << curr_x - prev_trajpoint.position.x,
                      curr_y - prev_trajpoint.position.y;
        vec_nextrob  << next_trajpoint.position.x - curr_x,
                        next_trajpoint.position.y - curr_y;
        
        // double traj_angle = atan2(vec_prevnext[1],vec_prevnext[0]);//可以使用轨迹中自带的yaw
        double tar_angle = atan2(vec_nextrob[1],vec_nextrob[0]);//目标角度
        angle_error = curr_yaw - tar_angle;//
        if(angle_error > PI) angle_error -=2*PI;
        if(angle_error < -PI) angle_error +=2*PI;

        /*+++++++++++++++++++++++++++++pid误差项+++++++++++++++++++++++++++++++++++*/
        double err_rate = 0.0;//
        err_rate = angle_error-angle_error_bef;//微分项

        intg_err+= angle_error*dt;//积分项
        angle_error_bef=angle_error;//
        if(intg_err>=0.75)
          intg_err=0.75;
        else if(intg_err<=-0.75)
          intg_err=-0.75;
        /*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

        // if(fabs(angle_error) >= max_angle_vel)
        // {
        //   omega = 0.4;
        //   if(angle_error > 0)
        //   {
        //     omega = -omega;
        //   }
        //   aim_velocity = 0.5*max_linear_vel;
        // } 
        // else
        // {
          // double theta_e = atan2(sk*path_error, fabs(curr_speed)+0.5);
          omega = -( kd*err_rate/(dt+1.0e-15) + kp*angle_error + ki*intg_err );//PID
          // std::cout<<"err_rate = "<<err_rate/(dt+1.0e-15)<<std::endl;
          if(omega >= max_angle_vel)
            omega = max_angle_vel;
          if(omega <= -max_angle_vel)
            omega = -max_angle_vel;
          // std::cout << angle_error << ", "<< theta_e << ", "<< omega <<std::endl;<< ", theta_e: "<< theta_e
          // std::cout<< "err_rate: "<< err_rate<< ", angle_error: "<< angle_error<<", intg_err: "<<intg_err<<",omega: "<<omega<<", dt: "<<dt<<std::endl;
          if(fabs(omega)>max_angle_vel)
          {
            aim_velocity = 0.7*max_linear_vel;
          }
          else
          {
            aim_velocity = max_linear_vel;
          }
        //}
      }
    } else {
        omega = 0.0;
        aim_velocity = 0.0;
        no_traj_count++;
    }
     
  if(fabs(aim_velocity) <= 1.0e-3)  velocity = 0;
  else
  {
    if((aim_velocity-curr_speed) > 0.15)
      velocity += 0.2*dt;
    else if((aim_velocity-curr_speed) < -0.15)
      velocity -= 0.2*dt;
    else velocity = aim_velocity;
    if(velocity > aim_velocity ) 
      velocity = aim_velocity;
       // velocity = aim_velocity;
  }
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = velocity;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = omega;//
  if(pause_flag)
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
  ros::init(argc, argv, "simple_path_follower");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("/wheel_to_init", 1, &odom_callback);
  ros::Subscriber traj_sub = nh.subscribe("/follow_path", 5, &traj_callback);
  //ros::Subscriber traj2_sub = nh.subscribe("/best_traj", 5, &traj_callback_2);
  ros::Subscriber pause_sub = nh.subscribe("/pause_flag", 5, &pause_callback);
  ros::Subscriber work_sub = nh.subscribe("/follow_control", 5, &work_callback);
  ctrl_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::NodeHandle private_nh("~");
  private_nh.param<float>("max_angle_velocity", max_angle_vel, 0.8);
  private_nh.param<float>("max_linear_velocity", max_linear_vel, 1.0);
  //private_nh.param<float>("sk", sk, 0.15);
  private_nh.param<float>("kp", kp, 0.15);
  private_nh.param<float>("kd", kd, 0.15);
  private_nh.param<float>("ki", ki, 0.15);
  std::cout<<"kp = "<<kp<<", kd = "<< kd <<", ki = "<< ki <<std::endl;
  last_odom_time = ros::Time::now().toSec();
 // if(ros::isShuttingDown()) fclose(outfile);
  ros::spin();
  
  return 0;   
}
