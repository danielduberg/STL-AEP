#include <stl_frontier_planner/stl_frontier_planner.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stl_frontier_planner");
  ros::NodeHandle nh;

  stl_frontier_planner::STLFrontierPlanner planner(nh);

  ros::spin();
  return 0;
}
