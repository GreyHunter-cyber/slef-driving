/**
 * @brief trajectory definition
 * @author Renjie Zhu
 * @date 2019-08-10
 */
#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include <vector>
#include "plan_msgs/Traj.h"
#include "polynomials.h"

/**
 * @brief 障碍物定义
 */
struct obstacle{
  double gx;   //global x
  double gy;
  double lx;   //local x
  double ly;
  double rd;   //障碍物半径
};

/**
 * @brief 轨迹描述
 */
class trajectory{
public:
  trajectory();
  virtual ~trajectory();
  /**
   * @brief 从初始状态ss， 规划周期T, 目标偏移与速度中生成轨迹描述
   * @param ss包含， 初始横向信息 s0, d_s0, d_s_s0, d0, d_d0, d_d_d0;
   *        周期T为此轨迹规划时长
   *        di为目标偏移， tv为目标速度
   */
  void init(double* ss, double T, double di, double tv);

  /**
   * @brief 生成轨迹点
   * @param dt为轨迹点时间间隔
   */
  bool generate_trajectory(double dt);
  /**
   * @brief calculate global x, y coordinate
   */
  void calc_global_xy(Spline2D* sp);
  /**
   * @brief 检查最大加速度与速度限制
   */
  bool check_valid();
  /**
   * @brief 碰撞检测
   */
  bool check_collision(std::vector<obstacle> obs);
  /**
   * @brief 一致性检测
   */
  double check_consistency(trajectory pref_traj);
  /**
   * @brief 计算损失函数
   */
  double check_cost(double target_velocity);
  /**
   * @brief = operator
   */
  trajectory& operator =(const trajectory& traj);

public:
  bool is_initialized;
  std::vector<plan_msgs::PointTraj> points;    //轨迹点
  double avg_ks;                     //曲率统计平均
  double avg_d;                      //偏移统计平均
  double Jd, Js;                     //Square of Jerk
  double consistency_value;          //一致性评测值
  double cost_s, cost_d, cost_total; //纵向损失， 横向损失， 总损失
  float collision_length;
  float min_obstacle_dis;

private:
  double T_;                //轨迹规划周期
  QuinticPolynomial lat_qp; //横向规划多项式(5次)
  QuarticPolynomial lon_qp; //纵向规划多项式(4次)
  bool is_valid;
  bool is_collision;
};

/**
 * @brief 轨迹集合
 */
class trajectory_set{
public:
  trajectory_set();
  virtual ~trajectory_set();
  /**
   * @brief 初始化轨迹组, 车辆当前状态ss, 目标速度
   */
  void init(double* ss, double tv);
  bool generate_trajectory_set(double dt);

  trajectory  check_best_trajectory(Spline2D* sp, std::vector<obstacle> obs);
  trajectory operator [](int i){return trajs[i];}
  std::vector<trajectory> get_trajs(){return trajs;}
private:
  double target_velocity;
  int num_trajs;
  std::vector<trajectory> trajs;
  trajectory prev_best_traj;
  bool is_initialized;
  bool is_generated;
};

#endif