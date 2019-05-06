#include <ros/ros.h>
#include <stl_aeplanner/stl_aeplanner.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stl_aeplanner");
  ros::NodeHandle nh;

  stl_aeplanner::STLAEPlanner stl_aeplanner(nh);

  ros::spin();
  return 0;
}
