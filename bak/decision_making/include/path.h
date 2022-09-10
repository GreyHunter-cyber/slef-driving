/**
 * @brief trajectory definition
 * @author Renjie Zhu
 * @date 2019-08-10
 */
#ifndef _GLOBAL_PATH_H_
#define _GLOBAL_PATH_H_

#include <vector>
#include "plan_msgs/Path.h"
#include "polynomials.h"
#include <mutex>

class GlobalPath{
public:
  GlobalPath();
  virtual ~GlobalPath();
  bool smooth_path(Spline2D *way_spline, double s_start, double s_end, double s_step);

public:
  double s_start;
  double s_end;
  bool is_smoothed;

  /**
   * @brief from x, y to frenet s,d
   */
  int get_frenet_pos(double x, double y, double theta, double& frenet_s, double& frenet_d, double& d_theta);
  /**
   * @brief reset global path status
   */
  void reset();

  std::vector<plan_msgs::PointSYK> & getWayPoints(){
    return way_points;
  }

private:
  std::vector<plan_msgs::PointSYK> way_points;
  int next_idx;
  int prev_idx;
  std::mutex              way_mutex;

  /**
   * @brief get closest way point from way_points
   */
  int get_closest_waypoint(double x, double y, int start_idx = 0);

  /**
   * @brief get next way point from way_points
   */
  int get_next_waypoint(double x, double y, double theta);

};

#endif