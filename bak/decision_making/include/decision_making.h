/**
 * @brief decision_making
 * @author luohai
 * @date 2020-07-24
 */
#ifndef DECISION_MAKING
#define DECISION_MAKING
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include "csv/csv.h"
#include "plan_msgs/HmiControl.h"
#include "plan_msgs/PointSYK.h"
#include "plan_msgs/RobotState.h"

#include "spline/spline.h"
#include "path.h"

#define PI 3.14159265
enum Situation
{
	REGULAR,		//正常，会避障
  FOLLOWING,  //跟车,遇障停车
	STOP,       //停车
  AUTOPARKING, //泊车
  FOLLOW_DUANG, //跟车,撞上去!!别停!!!

	START,
	STOPPED,
	JAM,
	AVOIDANCE,
	DANGER,
	DANGERCAR,
	SHARP,
	UTurn,
};
namespace csv = io;

class Decision_making{

public: 
  Decision_making();
  virtual ~Decision_making();

/**
   * @brief load global path
   */
  void load_globalpath(std::vector<geometry_msgs::PoseStamped>& local_plan);

/**
   * @brief decison the next traj point
   */
  void get_near_traj(std::vector<geometry_msgs::PoseStamped>& _local_plan, int& c_idx, double cx, double cy);

/**
   * @brief make decision
   */
  void decision_make();

  bool check_arrive_mission(int i, bool move_to);


private:

/**
   * @brief callback plan best traj messge
   */
  void traj_callback(const nav_msgs::Path::ConstPtr& traj_msg);

/**
   * @brief obstacle or not
   */
  void obs_flag_callback(const std_msgs::Bool::ConstPtr& bool_msg);
/**
   * @brief callback HmiControl messge
   */
  void HmiControl_msg_callback(const plan_msgs::HmiControl::ConstPtr& msg);
/**
   * @brief callback HmiControl messge
   */


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

public:
  ros::NodeHandle nh;
  ros::Subscriber traj_sub;
  ros::Subscriber flag_sub;
  ros::Subscriber HmiControl_msg_sub;
  ros::Subscriber odom_sub;

  ros::Subscriber arrive_sub;

  ros::Publisher globalpath_pub_;
  ros::Publisher showpath_pub_;

  ros::Publisher _path_pub;
  ros::Publisher  ctrl_pub;
  ros::Publisher  nav_pub;
  ros::Publisher  follow_pub;
  ros::Publisher point_pub;
  ros::Publisher _state_pub;

  //全局路径的文件地址
  std::string global_path;
  //下发轨迹
  std::vector<geometry_msgs::PoseStamped> local_plan;
  //全局轨迹
  std::vector<geometry_msgs::PoseStamped> global_traj;
//局部轨迹
  std::vector<geometry_msgs::PoseStamped> local_traj;
  //规划轨迹
  std::vector<geometry_msgs::PoseStamped> plan_traj;
  //障碍物标志 
  bool ob_flag = false;
  bool stop_nav_flag = true;
  bool mition_point_flag = false;
  bool arrive_flag = false;
	Situation m_Situation;

  int mition_time_count = 0;
  int wait_time = 5;
  //决策消息
  plan_msgs::HmiControl HmiControl_msg;

  nav_msgs::Path globalpath;

nav_msgs::Path show_path;
//int count_n = 0;
int global_count=0;
int c_idx = 0;
int j = 0;
bool pause_flag = true;
bool next_flag = false;
bool restart;
bool first_odom =true;
double curr_x,curr_y,curr_yaw,curr_speed,prev_x,prev_y;
double last_odom_time =0.0;
std_msgs::Bool Parking_flag;
GlobalPath way_path;
std::string mission_file;
std::vector<double> points_x_origin, points_y_origin,points_z_origin,points_yaw_origin, points_s_origin;

plan_msgs::RobotState robotstate;
bool Repeat;//是否循环巡逻
};
#endif



