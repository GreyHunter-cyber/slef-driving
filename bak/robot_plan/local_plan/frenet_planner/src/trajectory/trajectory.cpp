/**
 * @brief trajectory definition
 * @author Renjie Zhu
 * @date 2019-08-10
 */

#include "trajectory.h"

float max_speed = 5.3/3.6;     // 最大速度 [m/s]
float max_accel = 2.0;         // 最大加速度[m/ss]
float max_curvature = 0.5;    // 最大曲率 [1/m]
float max_road_width = 1.0;    // 最大道路宽度 [m] 2.5
float d_road_w = 0.20;         // 道路宽度采样间隔 [m] 0.25
float maxT = 4.0;             // 最大预测时间 [s]
float minT = 3.0;             // 最小预测时间 [s]
float dT = 1.0;                // 预测时间采样间隔 [s]

float d_t_s = 0.10;      // 目标速度采样间隔 [m/s]
int n_s_sample = 0;         // sampling number of target speed
float robot_radius = 0.35;  // robot radius [m]

//损失函数权重
float KJ = 0.1;
float KT = 0.1;
float KD = 1.0;     //目标状态不应偏离道路中心太远,值越小就越容易偏离道路中心
float KLAT = 1.0;    //横向损失权重
float KLON = 0.5;    //纵向损失权重
float KCON = 1.5;     //一致性权重
float KOBS = 1.0;     //障碍物权重



trajectory::trajectory()
{
    T_ = 0.0;
    is_initialized = false;
    is_valid = false;
    is_collision = false;
    collision_length = 999999;
    min_obstacle_dis = 999999;
    Jd = 0.0;
    Js = 0.0;
    cost_d = 0.0;
    cost_s = 0.0;
    cost_total = 0.0;
    avg_ks = 0.0;
    avg_d = 0.0;
    consistency_value = 0.0;
}

trajectory::~trajectory()
{

}

void trajectory::init(double* ss, double T, double di, double tv)
{
    T_ = T;
    /**********************************横向规划**********************************/
    //横向初始状态
    double ss_lat[3] = {ss[3], ss[4], ss[5]};
    //横向目标配置
    double es_lat[3] = {di, 0, 0};
    lat_qp.init(ss_lat, es_lat, T);

    /**********************************纵向规划**********************************/
    //纵向初始状态
    double ss_lon[3] = {ss[0], ss[1], ss[2]};
    //纵向目标配置
    double es_lon[2] = {tv, 0};
    lon_qp.init(ss_lon, es_lon, T);

    is_initialized = true;
    is_valid = true;
}

bool trajectory::generate_trajectory(double dt)
{
    if(!is_initialized) return false;
    for(double t = 0; t<= T_; t+=dt)
    {
        plan_msgs::PointTraj pt;
        pt.t = t;
        pt.d = lat_qp.calc_point(t);
        pt.d_d = lat_qp.calc_first_derivative(t);
        pt.d_dd = lat_qp.calc_second_derivative(t);
        pt.d_ddd = lat_qp.calc_third_derivative(t);
        Jd += pow(pt.d_ddd, 2);

        pt.s = lon_qp.calc_point(t);        
        pt.s_d = lon_qp.calc_first_derivative(t);
        pt.s_dd = lon_qp.calc_second_derivative(t);
        pt.s_ddd = lon_qp.calc_third_derivative(t);
        Js += pow(pt.s_ddd, 2);

        if(pt.s_d > max_speed||pt.s_dd > max_accel){
            is_valid = false;
            //std::cout << pt.s_d << "," <<pt.s_dd <<std::endl;
        }
        avg_d+=pt.d;
        points.push_back(pt);
    }
    avg_d/=points.size();
    return true;
}

trajectory& trajectory::operator =(const trajectory& traj){
    this->points = traj.points;
    this->avg_ks = traj.avg_ks;
    this->avg_d = traj.avg_d;
    this->Jd = traj.Jd;
    this->Js = traj.Js;
    this->cost_s = traj.cost_s;
    this->cost_d = traj.cost_d;
    this->is_initialized = traj.is_initialized;
    return *this;
}

void trajectory::calc_global_xy(Spline2D* sp){
    int sz = points.size();
    for(int i = 0; i< points.size(); ++i){
      std::vector<double> pos = sp->calc_pos(points[i].s);
      if(pos.size() == 0){
          continue;
      }
      double yaw = sp->calc_yaw(points[i].s);
      points[i].x = pos[0] + points[i].d * cos(yaw + PI / 2.0);
      points[i].y = pos[1] + points[i].d * sin(yaw + PI / 2.0);
      if(i>=1){
        double dx = points[i].x-points[i-1].x;
        double dy = points[i].y-points[i-1].y;
        if(sqrt(dx*dx+dy*dy)<1.0e-5){
            points[i].yaw = 0;
            points[i].ks = 0;
        }
        else{
            points[i].yaw = atan2(dy, dx);
            points[i].ks = (points[i].yaw - points[i-1].yaw)/sqrt(dx*dx+dy*dy);
        }
        if(fabs(points[i].ks)>=50) points[i].ks = 0;//异常ks处理
        avg_ks+=points[i].ks;
      }     
    }
    if(sz >= 1){
        points[0].yaw = points[1].yaw;
        points[0].ks = points[1].ks;
        avg_ks+=points[0].ks;
        avg_ks/=sz;
    }
    else{
        points[0].yaw = 0;
        points[0].ks = 0;
        avg_ks = 0;
    }
}

bool trajectory::check_valid()
{

    return is_valid;
}

bool trajectory::check_collision(std::vector<obstacle> obs)
{
    int sz = obs.size();
    if(0 == points.size()||0 == sz) return false;
//#pragma omp for
    for(int i = 0; i< sz; ++i){
        float minDist = 999999;
        int minIdx = 0;
        for(int j = 0; j< points.size(); ++j){
            float tempDist = dist(points[j].x, points[j].y, obs[i].gx, obs[i].gy);
            if(tempDist < minDist){
                minDist = tempDist;
                minIdx = j;
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
        if(minDist <= obs[i].rd + robot_radius  &&
           minIdx != points.size()-1){
            collision_length = points[minIdx].s - points[0].s;
            if(collision_length < 10 ){
                is_collision = true;
                break;
            }
        }
    }
    return is_collision;
}

//为了保证前后规划路线的连续性,防止规划路线突变,引入一致性因素到损失函数中
double trajectory::check_consistency(trajectory pref_traj)
{
    if(pref_traj.is_initialized){
        //std::cout<<avg_ks<<"--"<<pref_traj.avg_ks<<"--"<<avg_d<<"--"<<pref_traj.avg_d<<std::endl;
        consistency_value = fabs(avg_ks-pref_traj.avg_ks)+fabs(avg_d-pref_traj.avg_d);
        if(__isnan(consistency_value)){
            consistency_value = 0.0;
        }
    } else {
        std::cout<<"pre traj not inited!"<<std::endl;
        consistency_value = 0.0;
    }
    return consistency_value;
}

double trajectory::check_cost(double target_velocity)
{
    int sz = points.size();
    //速度误差
    double ds = pow(target_velocity - points[sz-1].s_d, 2);
    //横向误差
    cost_d = KJ * Jd + KT * T_ + KD * pow(points[sz-1].d, 2);
    //纵向误差
    cost_s = KJ * Js + KT * T_ + KD * ds;
    //总体加权误差
    //todo 损失函数未归一化
    cost_total = KLAT * cost_d  + KLON * cost_s + KCON * consistency_value + KOBS*1.0/min_obstacle_dis;
//    std::cout<<"cost_d:"<<KLAT * cost_d
//             <<";cost_s"<<KLON * cost_s
//             <<";consistency_value"<<consistency_value
//             <<";obstacle cost:"<<KOBS/min_obstacle_dis
//             <<";cost_total"<<cost_total<<std::endl;
    return cost_total;
}

/******************************trajectory_set********************************/
trajectory_set::trajectory_set()
{
    is_initialized = false;
    is_generated = false;
    num_trajs = 0;
    target_velocity = 0;
}

trajectory_set::~trajectory_set()
{

}

void trajectory_set::init(double* ss, double tv)
{
    trajs.clear();
    //车道采样
    for(double di = -max_road_width; di <= max_road_width; di+=d_road_w){
        //预测时间采样
        for(double t = minT; t<=maxT; t+=dT){
            //目标速度采样
            for(double v = tv - d_t_s * n_s_sample; v<= tv + d_t_s * n_s_sample; v+=d_t_s){
                trajectory traj;
                traj.init(ss, t, di, v);
                trajs.push_back(traj);
            }
        }
    }
    num_trajs = trajs.size();
    target_velocity = tv;
    is_initialized = true;
}

bool trajectory_set::generate_trajectory_set(double dt)
{
    if(!is_initialized) return false;
    for(int i= 0; i< num_trajs; ++i){
        trajs[i].generate_trajectory(dt);
    }
    is_generated = true;
    return true;
}

trajectory trajectory_set::check_best_trajectory(Spline2D* sp, std::vector<obstacle> obs)
{
    trajectory traj_temp;
    for(int i = 0; i< obs.size(); ++i){
      if(fabs(obs[i].ly)<=0.35 && obs[i].lx<=0.4+obs[i].rd){
        prev_best_traj = traj_temp;
        return prev_best_traj;
      }
    }
    int best_idx = -1;
    double min_cost = 999999;
//    std::cout << "num trajs: "<<num_trajs<<std::endl;
    int coll_num = 0;
    for(int i = 0; i< num_trajs; ++i){
      //if(trajs[i].check_valid())
      {
          try{
              trajs[i].calc_global_xy(sp);
          }catch (std::exception& e){
              std::cout<<"catch######################################################"<<std::endl;
              continue;
          }
        if(trajs[i].check_collision(obs)){
            coll_num++;
            continue;
        }
        trajs[i].check_consistency(prev_best_traj);
        trajs[i].check_cost(target_velocity);
        if(trajs[i].cost_total < min_cost){
            min_cost = trajs[i].cost_total;
            best_idx = i;
        }
      }
    }
//    std::cout << "num coll: "<<coll_num<<std::endl;
    if(best_idx!=-1) prev_best_traj = trajs[best_idx];
    else{
        //prev_best_traj.check_collision(obs);
        prev_best_traj = traj_temp;
    }
    return prev_best_traj;
}