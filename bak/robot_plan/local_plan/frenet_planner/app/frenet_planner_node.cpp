#include "frenet_planner/frenet_planner.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "frenet_planner");

  ros::NodeHandle nh("~");
  local_planner::FrenetPlanner fplanner(nh);

  ros::AsyncSpinner spinner(6); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  return(0);
}