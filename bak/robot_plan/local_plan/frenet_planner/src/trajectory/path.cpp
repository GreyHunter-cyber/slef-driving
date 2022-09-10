#include "path.h"

GlobalPath::GlobalPath(){
  s_start = 0;
  s_end = 0;
  next_idx = 0;
  prev_idx = -1;
  is_smoothed = false;
}

GlobalPath::~GlobalPath(){

}

bool GlobalPath::smooth_path(Spline2D *way_spline, double s_start, double s_end, double s_step){
  if(!way_spline) return false;
  std::lock_guard<std::mutex> lock(way_mutex);
  way_points.clear();
  this->s_start = s_start;
  this->s_end = s_end;

  plan_msgs::PointSYK ppt;
  std::string ts = "";
  for(double s = s_start; s < s_end; s+=s_step){
    ppt.s = s;
    std::vector<double> pos = way_spline->calc_pos(s);
    ppt.x = pos[0];
    ppt.y = pos[1];
    ppt.yaw = way_spline->calc_yaw(s);
    ppt.ks = way_spline->calc_ks(s);
    way_points.push_back(ppt);
    ts += std::to_string(ppt.x);
    ts += ",";
    ts += std::to_string(ppt.y);
    ts += "##";
  }
//  std::cout<<"spline:"<<ts<<std::endl;
  ppt.s = s_end;
  std::vector<double> pos = way_spline->calc_pos(ppt.s);
  ppt.x = pos[0];
  ppt.y = pos[1];
  ppt.yaw = way_spline->calc_yaw(ppt.s);
  ppt.ks = way_spline->calc_ks(ppt.s);
  way_points.push_back(ppt); 
  is_smoothed = true; 
}

/**
 * @brief get closest way point from way_points
 */
int GlobalPath::get_closest_waypoint(double x, double y, int start_idx){
  double closest_distance = std::numeric_limits<double>::max();
  int closest_waypoint_index = -1;  
  for(int i = start_idx; i < way_points.size(); i++)
  {
  	double waypoint_x = way_points[i].x;
  	double waypoint_y = way_points[i].y;  
  	double current_distance = dist(x,y,waypoint_x,waypoint_y);  
  	if(current_distance < closest_distance)
  	{
  		closest_distance = current_distance;
  		closest_waypoint_index = i;
  	}
  }  
  return closest_waypoint_index;     
}
/**
 * @brief get next way point from way_points
 */
int GlobalPath::get_next_waypoint(double x, double y, double theta){
    if(prev_idx == -1){
        next_idx = get_closest_waypoint(x,y,0);
    }
    else {
        next_idx = get_closest_waypoint(x,y,prev_idx);
    }
    if(next_idx == -1) return next_idx;

    double waypoint_x = way_points[next_idx].x;
    double waypoint_y = way_points[next_idx].y;

    double heading = atan2(
        (waypoint_y-y),
        (waypoint_x-x)
    );

    double angle = fabs(theta - heading);

    angle = std::min(2 * M_PI - angle, angle);

    if(angle > M_PI / 4)
    {
        next_idx++;
        if (next_idx == way_points.size())
        {
            next_idx = 0;
        }
    }
    if (next_idx == 0){
        prev_idx = 0;
        next_idx = (way_points.size() >5?5:way_points.size()-1);
    }else{
        prev_idx = next_idx - 1;
    }
//    prev_idx = (0 == next_idx ?  0 : next_idx - 1);
    return next_idx;
}

/**
 * @brief from x, y to frenet s,d, heading reference to s direction
 */
int GlobalPath::get_frenet_pos(double x, double y, double theta, double& frenet_s, double& frenet_d, double& d_theta){
//    std::cout<<"x:"<<x<<" y:"<<y<<" s:"<<frenet_s<<" yaw:"<<theta<<std::endl;
  std::lock_guard<std::mutex> lock(way_mutex);
  next_idx = get_next_waypoint(x,y, theta);
  if(next_idx == -1) return next_idx;
  // normalized tangential vector:
  double tx = way_points[next_idx].x-way_points[prev_idx].x;
  double ty = way_points[next_idx].y-way_points[prev_idx].y;
  double norm_t = sqrt(tx*tx + ty*ty);
  tx /= norm_t;
  ty /= norm_t;

  // translation relative to previous waypoint:
  double dx = x - way_points[prev_idx].x;
  double dy = y - way_points[prev_idx].y;
  d_theta = theta -(way_points[prev_idx].yaw+way_points[next_idx].yaw)/2.0;

  // find the projection of translation onto t
  double norm_proj = (dx*tx+dy*ty);
  double proj_x = norm_proj * tx;
  double proj_y = norm_proj * ty;

  // frenet s:
  frenet_s = way_points[prev_idx].s + norm_proj; 
  // frenet d:
  frenet_d = dist(dx,dy,proj_x,proj_y);

  // determine the sign of d:
  double perp_x = dx - proj_x;
  double perp_y = dy - proj_y;

  if(tx*perp_y-perp_x*ty<0) frenet_d = -frenet_d;//right is negative
  return next_idx;    
}

void GlobalPath::reset(){
  s_start = 0;
  s_end = 0;
  next_idx = 0;
  prev_idx = -1;
  is_smoothed = false;
}