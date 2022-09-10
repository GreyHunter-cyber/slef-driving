/**
 * @brief decision_making
 * @author luohai
 * @date 2020-07-24
 */


#include "decision_making.h"

double time1=0.0,time2=0.0;

Decision_making::Decision_making()
{
  traj_sub = nh.subscribe("/best_traj", 5, &Decision_making::traj_callback, this);
  flag_sub = nh.subscribe("/flag_obstacle", 5, &Decision_making::obs_flag_callback, this);
  HmiControl_msg_sub = nh.subscribe("/iau_ros_hmi/HmiControl", 50, &Decision_making::HmiControl_msg_callback, this);
  
  odom_sub = nh.subscribe("/wheel_to_init", 1, &Decision_making::odom_callback,this);

//  globalpath_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 3, true);
  showpath_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 3, true);

  _path_pub = nh.advertise<nav_msgs::Path>("/follow_path", 3, true);
  ctrl_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  nav_pub = nh.advertise<std_msgs::Bool>("/pause_flag", 10);
  follow_pub = nh.advertise<std_msgs::Bool>("/follow_control", 10);
  point_pub = nh.advertise<plan_msgs::PointSYK>("/reset_point", 3);
  _state_pub = nh.advertise<plan_msgs::RobotState>("/RobotState",10);
  nh.param<std::string>("global_path_csv", global_path, "/home/luoluolll/iau_robot_legoloam/maps/trajactory.csv");
  //障碍物标志 
  ob_flag = false;
  restart = false;
  robotstate.mition_arrive_num = 9999;
  robotstate.mition_arrived = false;
  robotstate.Stop = false;
  Repeat = true;//默认不循环
  load_globalpath(global_traj);
 
  for(int i = 0; i< global_traj.size(); i++)
  {
    
    show_path.poses.push_back(global_traj[i]);
  }
    
  local_plan = global_traj;
  HmiControl_msg.e_stop = 0;
  HmiControl_msg.control_flag = 0;

}

Decision_making::~Decision_making(){}

void Decision_making::traj_callback(const nav_msgs::Path::ConstPtr& traj_msg)
{
    plan_traj = traj_msg->poses;
}

void Decision_making::get_near_traj(std::vector<geometry_msgs::PoseStamped>& _local_plan, int& c_indx, double cx, double cy)
{
  double closest_distance = std::numeric_limits<double>::max();
  int closest_waypoint_index = -1;
    //遵循轨迹顺序行走,重线不碍事.
  int front_t = c_indx+100;
  int back_t = c_indx-100;
  front_t = std::min(front_t,int(_local_plan.size()));
  back_t = std::max(back_t,1);

  if(curr_speed >= 0.0)
  {
    for(int i = c_indx; i < front_t; i++)
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
  }else
  {
    for(int i = c_indx; i > back_t; i--)
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
  }
  
  if(closest_waypoint_index == _local_plan.size()-1) 
  { 
    if(Repeat)
      c_indx = 0;
      
    robotstate.mition_arrive_num = 0;//到达终点
  }
  else c_indx = closest_waypoint_index;
}

void Decision_making::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  double dt = odom_msg->header.stamp.toSec() - last_odom_time;
  last_odom_time = odom_msg->header.stamp.toSec();

  
  double closest_distance = std::numeric_limits<double>::max();
  int closest_waypoint_index = -1;

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
    for(int i = 0; i < global_traj.size(); i++)
    { 
      double traj_x = global_traj[i].pose.position.x;
      double traj_y = global_traj[i].pose.position.y;  
      double current_distance = dist(curr_x,curr_y,traj_x,traj_y);  
      if(current_distance < closest_distance)
      {
        closest_distance = current_distance;
        closest_waypoint_index = i;
      } 
    }
  if(closest_waypoint_index == -1)  c_idx = 0;
  else c_idx = closest_waypoint_index;
  printf("idx = %d",c_idx);
  }
  else
  {
    curr_speed = sqrt(pow(curr_x -prev_x,2)+pow(curr_y-prev_y, 2))/dt;
  }
  prev_x = curr_x;
  prev_y = curr_y;
  //  std::cout<<"location success"<<std::endl;
  robotstate.Speed = curr_speed;
  robotstate.Azimuth = curr_yaw;
  std::cout <<"dt = "<< dt<<", last time = "<<last_odom_time<<", curr_speed = "<<curr_speed<<std::endl;


}

//取消global_path_sim节点,在此读取全局轨迹
void Decision_making::load_globalpath(std::vector<geometry_msgs::PoseStamped>& _local_plan)
{
    double s_resolution = 0.5;
    //读取路径文件中的点
    csv::CSVReader<4, io::trim_chars<>, io::no_quote_escape<' '>> hd_map(global_path);
    hd_map.read_header(io::ignore_extra_column, "x", "y","z","yaw");
    // parse map:
    double s=0, x, y, z, yaw;
    points_s_origin.push_back(s);
    int count = 0;
    while(hd_map.read_row(x, y,z,yaw)){
      points_x_origin.push_back(x);
      points_y_origin.push_back(y);
      points_z_origin.push_back(z);
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
      return;
    }
    //抽希(如果不抽希，容易过拟合，导致曲线不够平滑)
    std::vector<double> points_x, points_y, points_s;
    double prev_s = points_s_origin[0];
    points_s.push_back(points_s_origin[0]);
    points_x.push_back(points_x_origin[0]);
    points_y.push_back(points_y_origin[0]);
    bool need_add_lastpoint = true;
    for(int i = 1; i< sz; ++i)
    {
      if(points_s_origin[i]-prev_s > 5.0)//1.5m一个点
      {
        points_s.push_back(points_s_origin[i]);
        points_x.push_back(points_x_origin[i]);
        points_y.push_back(points_y_origin[i]);
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
    }
    std::vector<double> points_ss;
    Spline2D *way_spline = new Spline2D(points_x, points_y, points_ss);
    sz = points_ss.size();
    printf("sz = %d,\n i = %d\n",sz,points_x.size());
    way_path.smooth_path(way_spline, points_ss[0], points_ss[sz-1], s_resolution);
    std::vector<plan_msgs::PointSYK> path_points = way_path.getWayPoints();
    globalpath.header.stamp = ros::Time::now();
    globalpath.header.frame_id = "map";
    for(int i = 0; i< path_points.size(); ++i)
    {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.pose.position.x = path_points[i].x;//points_y[i]
      pose_msg.pose.position.y = path_points[i].y;
      pose_msg.pose.position.z = 0;
      globalpath.poses.push_back(pose_msg);
    }
    std::cout << "sz = "<<sz <<std::endl;
    _local_plan = globalpath.poses;
}


void Decision_making::obs_flag_callback(const std_msgs::Bool::ConstPtr& bool_msg)
{
  ob_flag = bool_msg->data;
}

void Decision_making::HmiControl_msg_callback(const plan_msgs::HmiControl::ConstPtr& msg)
{
  HmiControl_msg.e_stop = msg->e_stop;//停车

  HmiControl_msg.control_flag = msg->control_flag;//远程控制
  HmiControl_msg.speed = msg->speed;
  //HmiControl_msg.ang_velo = msg->ang_velo;
  HmiControl_msg.distance = msg->distance;
  HmiControl_msg.angle = msg->angle;

 // HmiControl_msg.obs_stop_flag = msg->obs_stop_flag;//遇到障碍停车

  //HmiControl_msg.mition_flag = msg->mition_flag;//任务点
  HmiControl_msg.mition_num =msg->mition_num;
  HmiControl_msg.mition_point_x = msg->mition_point_x;
  HmiControl_msg.mition_point_y = msg->mition_point_y;
  HmiControl_msg.mition_point_a = msg->mition_point_a;
  if(msg->mition_finish)
  {
    HmiControl_msg.mition_finish=true;
  }
  HmiControl_msg.action_flag = msg->action_flag;
  if(HmiControl_msg.control_flag != 0)
  {
    stop_nav_flag = true;
  }
  else
  {
    int reset_idx = 0;
    double closest_distance = std::numeric_limits<double>::max();
    double traj_x,traj_y,current_distance;
    plan_msgs::PointSYK reset_point;

    switch (HmiControl_msg.action_flag )//触发式,确保置零后仍可继续导航,现在默认只有非0 controlflag才能重置actionflag;
    {
      case 1:
        mition_point_flag = true;
        break;
      case 2:
        stop_nav_flag = false;
        break;
      case 3:
        restart = true;

        for(int i=0;i<points_x_origin.size()-1;i++)
        {
          traj_x = points_x_origin[i];
          traj_y = points_y_origin[i];  
          current_distance = dist(msg->origin_x,msg->origin_y,traj_x,traj_y);  
          if(current_distance < closest_distance)
          {
            closest_distance = current_distance;
            reset_idx = i;
          }
        }
        reset_point.x = points_x_origin[reset_idx];
        reset_point.y = points_y_origin[reset_idx];
        reset_point.s = points_z_origin[reset_idx];
        reset_point.yaw = points_yaw_origin[reset_idx];
        reset_point.ks = 1.0;
        point_pub.publish(reset_point);
        break;
      default:
        break;
      }
    }
  time2=time1;
}

  //是否到达任务点
bool Decision_making::check_arrive_mission(int i, bool move_to)
{
  if(!move_to)
  {
    int closest_distance = 99999;
    int targ_idx = 0;
     for(int j = 0; j < global_traj.size();++j)
  {
  	double traj_x = global_traj[j].pose.position.x;
  	double traj_y = global_traj[j].pose.position.y;  
  	double current_distance = dist(HmiControl_msg.mition_point_x[i],HmiControl_msg.mition_point_y[i],traj_x,traj_y);  
  	if(current_distance < closest_distance)
  	{
  		closest_distance = current_distance;
  		targ_idx = j;
  	}
  }
    float mition_s = dist(curr_x,curr_y,global_traj[targ_idx].pose.position.x,global_traj[targ_idx].pose.position.y);
    std::cout<<mition_s<<std::endl;
    if(mition_s < 2)
    {
      std::cout<<"**arived**"<<std::endl;
      robotstate.mition_arrive_num = j+1;
      robotstate.mition_arrived = true;
      pause_flag=true;
      return true;
    }
    else
    {
      robotstate.mition_arrived = false;
      return false;
    }
  }else
  {
    /* 行驶到任务点 */
  }
  
}

void Decision_making::decision_make()
{
  //截取一段轨迹
  get_near_traj(global_traj,c_idx,curr_x,curr_y);
  int c_ind = c_idx-60;
  if(c_ind<0)c_ind=0;
  for(int k = c_ind; k<(c_ind+120);k++)
  { 
    if(k>=global_traj.size())
      break;
    local_traj.push_back(global_traj[k]);
  }
  //轨迹发布
  std_msgs::Bool follow_flag;
  follow_flag.data = true;
  float min_L = 999999.0;
  geometry_msgs::Twist cmd_vel;

    global_count++;
    if(global_count==10)
    {
    // 截取的路径对frenet规划不友好,暂时不用,如果出现轨迹有交叉的情况,再酌情考虑.
    //   nav_msgs::Path gl_path;
    // gl_path.header.stamp = ros::Time::now();
    // gl_path.header.frame_id = "map";
    // for(int i = 0; i< local_traj.size(); i++)
    // {
    //   gl_path.poses.push_back(local_traj[i]);
    // }
       global_count = 0;
    //   globalpath_pub_.publish(gl_path);
      //show_path
      show_path.header.stamp = ros::Time::now();
      show_path.header.frame_id = "map";
      showpath_pub_.publish(show_path);
    }

  //决策条件
  if(HmiControl_msg.e_stop==1){
    ROS_INFO("***emergency stop!***");
    pause_flag = true;
  }
  else
  {
    switch(HmiControl_msg.control_flag)
    {
    case 1: 
      ROS_INFO("***pause mode***");
      pause_flag = true;
      break;
    case 2:
      ROS_INFO("***stop mode***");
      pause_flag = true;
      break;
    case 3:
      ROS_INFO("***remote mode***");
      follow_flag.data = false;//遥控单独控制车,不需要follow控制
      cmd_vel.linear.x = HmiControl_msg.speed;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z =  HmiControl_msg.angle/180*PI;//
      break;
    case 0:
      ROS_INFO("***free mode***");
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0,0;//
      if(!stop_nav_flag)
      {
        ROS_INFO("***navigation***");
        pause_flag = false;
        
        if(mition_point_flag)
        {
            ROS_INFO("***mition point***");
            std::cout<<"x size = "<<HmiControl_msg.mition_point_x.size()<<", num = "<<HmiControl_msg.mition_num<<std::endl;
          
          if(HmiControl_msg.mition_point_x.size() != HmiControl_msg.mition_num)
            {
              ROS_INFO("mition points x error!please check points num.");
              return ;
            }
          if(HmiControl_msg.mition_point_y.size() != HmiControl_msg.mition_num)
            {
              ROS_INFO("mition points y error!please check points num.");
              return ;
            }
            if( j < HmiControl_msg.mition_num)
          {

            switch(HmiControl_msg.mition_point_a[j])
            {
            case REGULAR://0
              ROS_INFO("***REGULAR***");
              local_plan = plan_traj;//避障路径
              pause_flag = false;
              if(check_arrive_mission(j,false))//检测是否到达,任务是否完成
              {
                if(HmiControl_msg.mition_finish)
                {
                  ROS_INFO("***this mition finished***");
                  HmiControl_msg.mition_finish = false;
                  robotstate.mition_arrived = false;

                  j++;
                }
              }
              break;
            case FOLLOWING://1
              ROS_INFO("***FOLLOWING***");
              local_plan = global_traj;// 截取的路径对frenet规划不友好,暂时不用,如果出现轨迹有交叉的情况,再酌情考虑.
              if(ob_flag)
                {pause_flag = true;}
              if(check_arrive_mission(j,false))
              {
                if(HmiControl_msg.mition_finish)
                {
                  ROS_INFO("***this mition finished***");

                  HmiControl_msg.mition_finish = false;
                  robotstate.mition_arrived = false;

                  j++;
                }
              }
              break;
            default:
              ROS_INFO("***error situation***");
              break;
            }

            std::cout<<"j = "<<j<<", point = "<<HmiControl_msg.mition_point_x[j]<<" "<<HmiControl_msg.mition_point_y[j]<<std::endl;
          }
          else
          {
            std::cout<<"arived final"<<std::endl;
            mition_point_flag = false;
            j=0;
          }
        }
        else{
        j=0;
        //默认巡航模式,不避障
        local_plan = global_traj;
        std::cout<<"global"<<std::endl;
        if(ob_flag)
          pause_flag = true;
        }
      }
      break;
    default:
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0,0;//
      break;
    }
  }
  //是否由跟随模块控制底盘(机器人有遥控模式,直接在决策里控制)
  follow_pub.publish(follow_flag);
  //发布轨迹话题
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  for(int i = 0; i< local_plan.size(); i++)
  {
    path.poses.push_back(local_plan[i]);
  }
  _path_pub.publish(path);
  std_msgs::Bool flag;
  flag.data = pause_flag;
  nav_pub.publish(flag);
  robotstate.Stop = pause_flag;
  _state_pub.publish(robotstate);
  //非导航时发布控制消息(不想换了)
  if(stop_nav_flag )
    ctrl_pub.publish(cmd_vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "decision_making");
  Decision_making decision;
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    decision.decision_make();
    decision.local_traj.clear();
  //校验通信是否正常
    time1=ros::Time::now().toSec();
    if(time1 - time2 > 1.5 && decision.HmiControl_msg.control_flag == 3)
     decision.HmiControl_msg.e_stop=1;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;   
}











