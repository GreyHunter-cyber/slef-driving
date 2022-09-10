/**
 * @brief frenet coordinate based trajectory planner definition
 * @author Renjie Zhu
 * @date 2019-08-14
 */

#include "frenet_planner.h"
#include "tf/transform_datatypes.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <std_msgs/Bool.h>
using namespace local_planner;


float g_plan_rate = 3.0;
bool g_show_all_traj = false;

//是否显示样调拟合线
bool g_show_spline = false;
bool ob_flag = false;//障碍物判断
int g_obs_start_layer = 0;
int g_obs_end_layer = 8;
float g_lidar_height = 1.155;
float stop_ds = 4.0;
ros::Publisher flag_ob;
bool firstOdom = false;
std::deque<geometry_msgs::Point> curPoses;
double endDS = 9999;

FrenetPlanner::FrenetPlanner(ros::NodeHandle &private_nh){
  private_nh.param<float>("max_speed", max_speed, 5.3);
  max_speed/=3.6;
  private_nh.param<float>("max_accel", max_accel, 2.0);  
  private_nh.param<float>("max_curvature", max_curvature, 1.5);
  private_nh.param<float>("max_road_width", max_road_width, 2.5);
  private_nh.param<float>("d_road_w", d_road_w, 0.25);
  private_nh.param<float>("maxT", maxT, 18.0);
  private_nh.param<float>("minT", minT, 10.0);
  private_nh.param<float>("dT", dT, 1.5);
  private_nh.param<float>("d_t_s", d_t_s, 0.10);
  private_nh.param<int>("n_s_sample", n_s_sample, 0);
  private_nh.param<float>("robot_radius", robot_radius, 0.35);
  private_nh.param<float>("KJ"  , KJ  , 0.1);
  private_nh.param<float>("KT"  , KT  , 0.1);
  private_nh.param<float>("KD"  , KD  , 1.0);
  private_nh.param<float>("KLAT", KLAT, 1.0);
  private_nh.param<float>("KLON", KLON, 0.5);
  private_nh.param<float>("KCON", KCON, 0);
  private_nh.param<int>("patch_width", patch_width, 80);
  private_nh.param<int>("patch_height", patch_height, 80);
  private_nh.param<float>("d_width", d_width, 0.1);
  private_nh.param<float>("d_height", d_height, 0.1);
  private_nh.param<float>("plan_rate",g_plan_rate,3.0);
  private_nh.param<bool>("show_all_trajs",g_show_all_traj,false);
  private_nh.param<bool>("show_spline",g_show_spline,false);
  private_nh.param<float>("height_from_lidar_to_ground",g_lidar_height,1.155);
  private_nh.param<float>("collision_distance",stop_ds,4.0);//碰撞距离
  path_sub = nh.subscribe("/global_path", 1, &FrenetPlanner::way_callback, this);
  imu_sub = nh.subscribe("/IAU/IMU", 1, &FrenetPlanner::imu_callback, this);
  // scan_sub = nh.subscribe("/scan", 1, &FrenetPlanner::scan_callback, this);
  // odom_sub = nh.subscribe("/odom", 1, &FrenetPlanner::odom_callback, this);
  odom_filter_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/integrated_to_init", 10));
  scan_filter_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/velodyne_points", 10));
  sync.reset(new message_filters::Synchronizer<OdomLaserSyncPolicy>(OdomLaserSyncPolicy(10), *odom_filter_sub, *scan_filter_sub));
  sync->registerCallback(boost::bind(&FrenetPlanner::odom_scan_callback, this, _1, _2));
  traj_pub = nh.advertise<nav_msgs::Path>("/best_traj", 5, true);
  obs_pub = nh.advertise<visualization_msgs::MarkerArray>("/obstacles", 5, true);
  alltraj_pub = nh.advertise<visualization_msgs::MarkerArray>("/all_trajs", 5, true);
  spline_pub = nh.advertise<visualization_msgs::MarkerArray>("/path_spline", 5, true);
  flag_ob = nh.advertise<std_msgs::Bool>("/flag_obstacle", 5, true);
  curr_x = 0.0; curr_y = 0.0;
  curr_yaw = 0.0;
  curr_s = 0.0; curr_d = 0.0;
  curr_speed = 0.0; curr_acc = 0.0;
  curr_speed_s= 0.0; curr_speed_d= 0.0;
  curr_omega = 0.0;
  curr_ds = 0.0;
  d_theta = 0.0;

  next_idx = 0;
  prev_idx = -1;
  
  way_spline = NULL;
  s_resolution = 0.5;

  planner_thread_ = new boost::thread(boost::bind(&FrenetPlanner::plan_thread, this));
}

FrenetPlanner::~FrenetPlanner(){
  planner_thread_->interrupt();
  planner_thread_->join();
  delete planner_thread_;
  if(NULL != way_spline) delete way_spline;
}

void FrenetPlanner::way_callback(const nav_msgs::Path::ConstPtr& pmsg){
  const int pt_sz = pmsg->poses.size();
  std::vector<double> points_x(pt_sz), points_y(pt_sz);
  std::vector<double> points_s;

  for(int i = 0; i< pt_sz; ++i){
    points_x[i] = pmsg->poses[i].pose.position.x;
    points_y[i] = pmsg->poses[i].pose.position.y;
  
  }
//  ROS_INFO("points=%s", s.c_str());
  if(NULL != way_spline) delete way_spline;
  way_spline = new Spline2D(points_x, points_y, points_s);
  way_path.reset();
  int sz = points_s.size();
  way_path.smooth_path(way_spline, points_s[0], points_s[sz-1], s_resolution);
  if(g_show_spline){
    publish_path_spline_markers(way_path.getWayPoints());
  }
}

void FrenetPlanner::imu_callback(const sensor_msgs::ImuPtr& imu_msg){									    
  curr_omega = imu_msg->angular_velocity.z;							    //angular_velocity.x
  curr_acc = sqrt(pow(imu_msg->linear_acceleration.x, 2)+pow(imu_msg->linear_acceleration.y, 2));				    //linear_acceleration.x	
  //ROS_INFO("imu_updated");
}
//全局碰撞检测,遇障停车用
bool FrenetPlanner::checkout_collision(std::vector<plan_msgs::PointSYK> points,std::vector<obstacle> obs)
{
    int sz = obs.size();
    if(0 == points.size()||0 == sz) 
    {
      return false;
    }
//#pragma omp for
    float min_obstacle_dis = 999999.0;
    bool is_collision = false;
    for(int i = 0; i< sz; ++i){
        float minDist = 999999.0;
        int minIdx = 0;
        float minPoseDist = 999999.0;
        int minPoseIdx = 0;
        //周围检测
        if(fabs(obs[i].lx) < 0.35 && fabs(obs[i].ly) <0.35)
        {
          continue;
        }
        if(fabs(obs[i].lx) < 1.5 && fabs(obs[i].ly) <(robot_radius + obs[i].rd)){
          return true;
        }
        //轨迹上检测
        for(int j = 0; j< points.size(); ++j){
            float tempDist = dist(points[j].x, points[j].y, obs[i].gx, obs[i].gy);
            float poseDist = dist(points[j].x, points[j].y, curr_x, curr_y);
            if(tempDist < minDist){
                minDist = tempDist;
                minIdx = j;
            }
            if(poseDist < minPoseDist){
              minPoseDist = poseDist;
              minPoseIdx = j;
            }
        }
        //统计距离障碍物最近的距离
        if(minDist < min_obstacle_dis){
            min_obstacle_dis = minDist;
            if(min_obstacle_dis == 0){
                min_obstacle_dis = 0.00001;
            }
        }
        //碰撞距离小于5m才有效
        //todo 条件待优化
        // 去掉 obs[i].lx >= robot_radius + obs[i].rd &&
        float collision_length = 0.0;
        if(minDist <= obs[i].rd + robot_radius ){
         // if(curr_speed >= 0)//向前向后
            float collision_length = points[minIdx].s - points[minPoseIdx].s;
         // else
          //  float collision_length = points[minIdx].s - points[minPoseIdx].s;
            if(collision_length < stop_ds ){
                is_collision = true;
                break;
            }
        }
        //std::cout<<"minobsDist = "<<minDist<<", collision_length = "<<collision_length<<", obsIdx = "<<minIdx<<", minPoseIdx = "<<minPoseIdx<<std::endl;
    }
    return is_collision;
}

//用于获取激光数据和odom的数据,得到当前机器人的障碍物信息
void FrenetPlanner::odom_scan_callback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                       const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg){
  if(!firstOdom)
  {
    first_odom_time =last_odom_time= odom_msg->header.stamp.toSec();
    curPoses.push_back(odom_msg->pose.pose.position);
    firstOdom = true;
  }
  if((odom_msg->header.stamp.toSec() - first_odom_time) < 1.0)
  {
    curPoses.push_back(odom_msg->pose.pose.position);
    last_odom_time = odom_msg->header.stamp.toSec();
  }else
  {
    double dt = odom_msg->header.stamp.toSec() - last_odom_time;
    curr_speed = dist(curPoses.front().x,curPoses.front().y,curPoses.back().x,curPoses.back().y);
    curr_speed_s = dist(curPoses.front().x,curPoses.front().y,curPoses.back().x,curPoses.back().y)*dt;

    last_odom_time = odom_msg->header.stamp.toSec();
    curPoses.push_back(odom_msg->pose.pose.position);
    curPoses.pop_front();
    ROS_INFO("speed: %f",curr_speed);
  }
   
  {
    std::lock_guard<std::mutex> lock(pos_mutex);
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
  }
plan_msgs::PointSYK end_path = way_path.getWayPoints().back();
  endDS = dist(curr_x, curr_y, end_path.x, end_path.y);
  tf::Transform baselink2odom;
  tf::poseMsgToTF(odom_msg->pose.pose, baselink2odom);

  std::vector<int> grid(patch_width*patch_height);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*pointcloud_msg,*cloud_data);
  for(int i=0;i<cloud_data->size();i++){
    pcl::PointXYZI t_pt = cloud_data->points[i];
    float lx = t_pt.x;
    float ly = t_pt.y;
    float lz = t_pt.z;
    if(lx<0||lx>d_height*patch_height||ly<-d_width*patch_width/2.0||ly>d_width*patch_width/2.0 || lz + g_lidar_height <= 0.1 || lz > 0.2){
      ++i;
      continue;
    }
    float ls = sqrt(lx * lx +ly * ly +lz * lz);
    if (ls < robot_radius)
    {
      ++i;
      continue;
    }
    int ix = floor(lx/d_height);
    int iy = floor(ly/d_width+patch_width/2);

    grid[ix*patch_width+iy] += 1;
  }

  //格网障碍物表达转为obstacle表达, 并转换到全局坐标系
  std::vector<obstacle> temp_obs;
  //todo 没看懂障碍物半径的计算公式 (d_height+d_width)/(2*sqrt(2.0))
  float rd = sqrt(d_height*d_height + d_width*d_width)/2.0;
#pragma omp parallel for num_threads(2)
  for(int j = 0; j< patch_height; ++j){
    for(int k = 0; k< patch_width; ++k){
      if(grid[j*patch_width+k] > 0){
        obstacle ob;
        ob.lx = j*d_height+d_height/2.0;
        ob.ly = k*d_width+d_width/2.0 - d_width*patch_width/2.0;
        tf::Vector3 vecPoint(ob.lx, ob.ly,0);
        tf::Transform point2, obs2odom;
	      point2.setOrigin(vecPoint);
	      obs2odom= baselink2odom * point2;//转全局坐标系
        ob.gx = obs2odom.getOrigin().x();
        ob.gy = obs2odom.getOrigin().y();
        ob.rd = rd;
        temp_obs.push_back(ob);
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(obs_mutex);
    obs.clear();
    obs = temp_obs;
//    std::cout <<"obstacles size: "<<obs.size() <<std::endl;
  }

//碰撞检测
  ob_flag = checkout_collision(way_path.getWayPoints(),obs);


  //obstacle visualization
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(1);
  visualization_msgs::Marker& obs_marker = marker_array.markers[0];;
  obs_marker.header.frame_id = "map";
  obs_marker.header.stamp = ros::Time::now();
  obs_marker.ns = "markers";
  obs_marker.points.reserve(temp_obs.size());
  obs_marker.colors.reserve(temp_obs.size());

  obs_marker.pose.position.z = 0.0f;
  obs_marker.pose.orientation.w = 1.0f;
  obs_marker.lifetime = ros::Duration(1.0);
  obs_marker.pose.position.z = 0;
  obs_marker.pose.orientation.w = 1.0;
  obs_marker.scale.x = obs_marker.scale.y = obs_marker.scale.z = rd;
  //obs_marker.frame_locked = true;
  obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  obs_marker.action = visualization_msgs::Marker::ADD;
  for(int j = 0; j< temp_obs.size(); ++j){
    geometry_msgs::Point point;
    point.x = temp_obs[j].gx;
    point.y = temp_obs[j].gy;
    point.z = 0;
    obs_marker.points.push_back(point);

    std_msgs::ColorRGBA rgba;
    rgba.r = 1.0f;
    rgba.g = 0.0f;
    rgba.b = 0.0f;
    rgba.a = 0.6f;
    obs_marker.colors.push_back(rgba);    
  }
  obs_pub.publish(marker_array);
}


void FrenetPlanner::publish_path_spline_markers(std::vector<plan_msgs::PointSYK> &points) {
  visualization_msgs::MarkerArray marker_array;

  marker_array.markers.resize(1);

  visualization_msgs::Marker& t_marker = marker_array.markers[0];;
  t_marker.header.frame_id = "map";
  t_marker.header.stamp = ros::Time::now();
  t_marker.ns = "markers";
  t_marker.id = 1;
  t_marker.points.reserve(points.size());
  t_marker.colors.reserve(points.size());

  t_marker.pose.position.z = 0.0f;
  t_marker.pose.orientation.w = 1.0f;
  t_marker.lifetime = ros::Duration(1.0);
  t_marker.pose.position.z = 0;
  t_marker.pose.orientation.w = 1.0;
  t_marker.scale.x = t_marker.scale.y = t_marker.scale.z = 0.1;
  //obs_marker.frame_locked = true;
  t_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  t_marker.action = visualization_msgs::Marker::ADD;

  for(int j = 0; j< points.size(); ++j) {
    geometry_msgs::Point point;
    point.x = points[j].x;
    point.y = points[j].y;
    point.z = 0;
  
    t_marker.points.push_back(point);
    std_msgs::ColorRGBA rgba;
    rgba.r = 1.0f;
    rgba.g = 1.0f;
    rgba.b = 0.0f;
    rgba.a = 0.6f;
    t_marker.colors.push_back(rgba);
  }
//  ROS_INFO("x=%s", s.c_str());
  spline_pub.publish(marker_array);
}


void FrenetPlanner::publish_all_trajs_markers(trajectory_set & traj_set) {
  visualization_msgs::MarkerArray marker_array;

  std::vector<trajectory> trajs = traj_set.get_trajs();
  marker_array.markers.resize(trajs.size());

  for(int i=0; i<trajs.size(); i++){
    visualization_msgs::Marker& t_marker = marker_array.markers[i];;
    t_marker.header.frame_id = "map";
    t_marker.header.stamp = ros::Time::now();
    t_marker.ns = "markers";
    t_marker.id = i;
    t_marker.points.reserve(trajs[i].points.size());
    t_marker.colors.reserve(trajs[i].points.size());

    t_marker.pose.position.z = 0.0f;
    t_marker.pose.orientation.w = 1.0f;
    t_marker.lifetime = ros::Duration(1.0);
    t_marker.pose.position.z = 0;
    t_marker.pose.orientation.w = 1.0;
    t_marker.scale.x = t_marker.scale.y = t_marker.scale.z = 0.05;
    //obs_marker.frame_locked = true;
    t_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    t_marker.action = visualization_msgs::Marker::ADD;
    std_msgs::ColorRGBA rgba;
    rgba.r = rand()%100/100.0;
    rgba.g = rand()%100/100.0;
    rgba.b = rand()%100/100.0;
    rgba.a = 0.6f;

    for(int j = 0; j< trajs[i].points.size(); ++j){
      geometry_msgs::Point point;
      point.x = trajs[i].points[j].x;
      point.y = trajs[i].points[j].y;
      point.z = 0;
      t_marker.points.push_back(point);
      t_marker.colors.push_back(rgba);
    }
    //ROS_INFO("index:%d,x=%s",i, s.c_str());
  }
  //std::cout <<"trajs marker size: "<<marker_array.markers.size() <<std::endl;

  alltraj_pub.publish(marker_array);
}

void FrenetPlanner::plan_thread()
{
  ROS_DEBUG_NAMED("frenet_planner_plan_thread","starting planner thread...");
  ros::Rate loop_rate(g_plan_rate);//规划频率
  std::cout<<"plan rate:"<<g_plan_rate<<std::endl;
  trajectory_set traj_set;
  bool  flag_arrive_end = false;
  while(ros::ok())
  {
    //boost::this_thread::interruption_point();
    double t1 = ros::Time::now().toSec()*1000;
    nav_msgs::Path traj_msg;
    if(way_spline != NULL)//&& !flag_arrive_end
    { 
      {
        std::lock_guard<std::mutex> lock(pos_mutex);
        way_path.get_frenet_pos(curr_x, curr_y, curr_yaw, curr_s, curr_d, d_theta);
        std::cout<<"s_end:"<<way_path.s_end<<" curr_s:"<<curr_s<<std::endl;
        curr_ds = way_path.s_end - curr_s;
      }

      {
        std::lock_guard<std::mutex> lock(flag_mutex);
        if(curr_ds < 5.0)
        {
          ob_flag = true;
        }
        std_msgs::Bool flag;
        flag.data = ob_flag;
        flag_ob.publish(flag);
      }


      if(endDS <= 0.5 ){
        ROS_INFO("Destination arrived!");
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.header.frame_id = "map";
        // flag_arrive_end = true;
      }
      else{
        //get frenet s, d
        // double ss[6] = {curr_s, curr_speed_s, curr_acc*cos(d_theta), 
        //                 curr_d, curr_speed_d, 0};
        
        double ss[6] = {curr_s, curr_speed_s, 0, 
                        curr_d, 0, 0};
        // std::cout <<" curr_s:"<<curr_s<<" curr_speed_s:" << curr_speed_s <<" curr_d:"<<curr_d<<std::endl;
        
        double tv = 1.0;
        if(curr_ds < 1.0) tv = 1.0;
        traj_set.init(ss, tv);
        traj_set.generate_trajectory_set(0.1);
    
        // curr_speed_s = (curr_s - prev_s)/1.0;
        curr_speed_d = (curr_d - prev_d)/1.0;
        prev_s = curr_s;
        prev_d = curr_d;
  
        std::lock_guard<std::mutex> lock(obs_mutex);
//        std::cout << obs.size()<<std::endl;
        trajectory best_traj = traj_set.check_best_trajectory(way_spline, obs);
        if(g_show_all_traj){
          publish_all_trajs_markers(traj_set);
        }
        if(!best_traj.is_initialized){
          ROS_INFO("No candidate trajectories!");
          traj_msg.header.stamp = ros::Time::now();
          traj_msg.header.frame_id = "map";
        }
        else{
          // std::cout << "consistency_value: "<<best_traj.consistency_value
          //           << " cost_s: "<<best_traj.cost_s<<", cost_d: "<<best_traj.cost_d<<std::endl;
          int sz = best_traj.points.size();
          traj_msg.header.stamp = ros::Time::now();
          traj_msg.header.frame_id = "map";
          for(int i = 0; i< sz; ++i)
          {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp.sec = best_traj.points[i].t;
            pose_msg.pose.position.x = best_traj.points[i].x;
            pose_msg.pose.position.y = best_traj.points[i].y;
            pose_msg.pose.position.z = 0;
            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(best_traj.points[i].yaw), pose_msg.pose.orientation);
            traj_msg.poses.push_back(pose_msg);
          } 
        }
      }
    }
    else{
      ROS_INFO("waiting for global path plan...");
      traj_msg.header.stamp = ros::Time::now();
      traj_msg.header.frame_id = "map"; 

    }
    
    if(traj_pub.getNumSubscribers()){
      traj_pub.publish(traj_msg);
    }
//    std::cout << "planning time: "<< ros::Time::now().toSec()*1000 - t1<<std::endl;
    loop_rate.sleep();
  }
}
