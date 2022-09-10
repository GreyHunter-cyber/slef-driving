/**
 * @brief frenet coordinate based trajectory planner definition
 * @author Renjie Zhu
 * @date 2019-08-14
 */

#ifndef FRENET_PLANNER
#define FRENET_PLANNER
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"

#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <vector>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/Dense>
//#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include "../trajectory/trajectory.h"
#include "../trajectory/path.h"

extern float max_speed;    
extern float max_accel;    
extern float max_curvature;
extern float max_road_width;
extern float d_road_w;     
extern float maxT;         
extern float minT;         
extern float dT;           
extern float d_t_s;      
extern int n_s_sample;     
extern float robot_radius; 

//损失函数权重
extern float KJ;
extern float KT;
extern float KD;
//
extern float KLAT;    
extern float KLON;
//前后路径一致性权重
extern float KCON;
//障碍物权重
extern float KOBS;

extern float g_plan_rate;

//是否显示所有规划线
extern bool g_show_all_traj;

//是否显示样调拟合线
extern bool g_show_spline;


extern int g_obs_start_layer;
extern int g_obs_end_layer;
extern float g_lidar_height;

int  patch_width = 40;
int patch_height = 30;
float d_width = 0.25;
float d_height = 0.5;

namespace local_planner{

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> OdomLaserSyncPolicy;

class FrenetPlanner{
public: 
  FrenetPlanner(ros::NodeHandle &private_nh);
  virtual ~FrenetPlanner();

private:
  /**
   * @brief receive global way_points and using spline to smooth the way_points
   */
  void way_callback(const nav_msgs::Path::ConstPtr& pmsg);
  /**
   * @brief receive imu data, get acceleration and omega of vehicle
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::ImuPtr& imu_msg);
  /**
   * @brief local obstacles to global obstacles representation
   *  odom and scan msg synchronized callback
   * @param odom_msg scan_msg
   */
  void odom_scan_callback(const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);

  /**
   * @brief receive odom data, get relative pose of robot
   * @param odom_msg
   */
  //void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

  /**
   * @brief  scan msg synchronized callback, get road surface grid
   * @param scan_msg
   */
  //void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  bool checkout_collision(std::vector<plan_msgs::PointSYK> points,std::vector<obstacle> obs);


    void publish_all_trajs_markers(trajectory_set & traj_set);

    void publish_path_spline_markers(std::vector<plan_msgs::PointSYK> &points);
private:
  /**
   * @brief trajectory planning thread 
   */
  void plan_thread();

private:
  ros::NodeHandle nh;

  ros::Subscriber path_sub;
  ros::Subscriber imu_sub;
  // ros::Subscriber scan_sub;
  // ros::Subscriber odom_sub;

  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_filter_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> scan_filter_sub;
  std::unique_ptr<message_filters::Synchronizer<OdomLaserSyncPolicy>> sync;

  ros::Publisher  traj_pub;
  ros::Publisher  obs_pub;
  ros::Publisher  alltraj_pub;
  ros::Publisher  spline_pub;

private:
  double                  s_resolution;
  Spline2D               *way_spline;
  GlobalPath              way_path;

  double last_odom_time = 0.0;//ms
  double first_odom_time = 0.0;//ms
  std::vector<obstacle> obs;//障碍物
  std::mutex            obs_mutex;

  std::mutex  pos_mutex;
  std::mutex  flag_mutex;
  double curr_x;
  double curr_y;
  double curr_yaw;
  double curr_s;
  double curr_d;
  double curr_speed;
  double curr_speed_s;
  double curr_speed_d;
  double curr_acc;
  double curr_omega;
  double curr_ds;   //与目的地的s方向距离
  int next_idx;
  int prev_idx;
  double prev_s = 0.0;
  double prev_d = 0.0;
  double d_theta;

  boost::thread* planner_thread_;
};
}

#endif
