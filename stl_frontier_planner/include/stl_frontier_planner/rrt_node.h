#ifndef STL_FRONTIER_PLANNER_RRT_NODE_H
#define STL_FRONTIER_PLANNER_RRT_NODE_H

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Pose.h>

#include <omp.h>

namespace stl_frontier_planner
{
// Rtree
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point;
typedef boost::geometry::model::box<point> box;
typedef boost::geometry::index::rtree<point, boost::geometry::index::rstar<16>> point_rtree;

struct RRTNode
{
  Eigen::Vector3d pos;
  std::shared_ptr<RRTNode> parent;
  std::vector<std::shared_ptr<RRTNode>> children;

  double getDistanceGain(std::shared_ptr<point_rtree> rtree, double ltl_lambda, double min_distance,
                         double max_distance, bool min_distance_active, bool max_distance_active,
                         double max_search_distance, double radius, double step_size);

  double cost(std::shared_ptr<point_rtree> rtree, double ltl_lambda, double min_distance, double max_distance,
              bool min_distance_active, bool max_distance_active, double max_search_distance, double radius,
              double step_size, std::map<int, std::pair<geometry_msgs::Pose, double>> routers, bool routers_active);

  double getMaxRouterDifference(std::map<int, std::pair<geometry_msgs::Pose, double>> routers, double step_size);

  std::pair<double, double> getDistanceToPositionAlongLine(Eigen::Vector3d start, Eigen::Vector3d end,
                                                                  Eigen::Vector3d point, double step_size);

  std::pair<double, double> getDistanceToClosestOccupiedBounded(std::shared_ptr<point_rtree> rtree,
                                                                       Eigen::Vector3d start, Eigen::Vector3d end,
                                                                       double max_search_distance, double radius,
                                                                       double step_size);
};
}  // namespace stl_frontier_planner

#endif  // STL_FRONTIER_PLANNER_RRT_NODE_H