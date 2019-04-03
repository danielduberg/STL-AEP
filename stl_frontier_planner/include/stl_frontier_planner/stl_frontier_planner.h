
#ifndef _RRT_H_
#define _RRT_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <actionlib/server/simple_action_server.h>
#include <aeplanner_msgs/rrtAction.h>

#include <nav_msgs/Path.h>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <omp.h>

#include <rrtplanner/LTLConfig.h>
#include <dynamic_reconfigure/server.h>
#include <dd_gazebo_plugins/Router.h>

namespace stl_frontier_planner
{
// Rtree
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point;
typedef boost::geometry::model::box<point> box;

typedef boost::geometry::index::rtree<point, boost::geometry::index::rstar<16>>
    point_rtree;

struct RRTNode
{
  Eigen::Vector3d pos;
  std::shared_ptr<RrtNode> parent;
  std::vector<std::shared_ptr<RrtNode>> children;

  double getDistanceGain(std::shared_ptr<point_rtree> rtree, double ltl_lambda,
                         double min_distance, double max_distance,
                         bool min_distance_active, bool max_distance_active,
                         double max_search_distance, double radius, double step_size)
  {
    if (!min_distance_active && !max_distance_active)
    {
      return 1;
    }

    Eigen::Vector3d start = (parent) ? parent->pos : pos;
    Eigen::Vector3d end = pos;

    std::pair<double, double> closest_distance = getDistanceToClosestOccupiedBounded(
        rtree, start, end, max_search_distance, radius, step_size);

    double distance_gain = 0;
    if (min_distance_active && max_distance_active)
    {
      distance_gain = std::min(closest_distance.first - min_distance,
                               max_distance - closest_distance.second);
    }
    else if (min_distance_active)
    {
      distance_gain = closest_distance.first - min_distance;
    }
    else
    {
      distance_gain = max_distance - closest_distance.second;
    }

    return distance_gain;
  }

  double cost(std::shared_ptr<point_rtree> rtree, double ltl_lambda, double min_distance,
              double max_distance, bool min_distance_active, bool max_distance_active,
              double max_search_distance, double radius, double step_size,
              std::map<int, std::pair<geometry_msgs::Pose, double>> routers,
              bool routers_active)
  {
    if (!parent)
    {
      return 0;
    }

    double distance_gain = getDistanceGain(rtree, ltl_lambda, min_distance, max_distance,
                                           min_distance_active, max_distance_active,
                                           max_search_distance, radius, step_size);

    double max_router_difference = distance_gain;
    if (routers_active)
    {
      max_router_difference = getMaxRouterDifference(routers, step_size);
    }

    return ((pos - parent->pos).norm() *
            std::fmax(
                std::exp(-ltl_lambda * std::min(distance_gain, max_router_difference)),
                1)) +
           parent->cost(rtree, ltl_lambda, min_distance, max_distance,
                        min_distance_active, max_distance_active, max_search_distance,
                        radius, step_size, routers, routers_active);
  }

  double getMaxRouterDifference(
      std::map<int, std::pair<geometry_msgs::Pose, double>> routers, double step_size)
  {
    Eigen::Vector3d start = (parent) ? parent->pos : pos;
    Eigen::Vector3d end = pos;

    double max_router_difference = -10000000;

    for (auto it = routers.begin(); it != routers.end(); ++it)
    {
      Eigen::Vector3d point(it->second.first.position.x, it->second.first.position.y,
                            it->second.first.position.z);

      std::pair<double, double> distance =
          getDistanceToPositionAlongLine(start, end, point, step_size);

      max_router_difference =
          std::max(max_router_difference, it->second.second - distance.second);
    }

    return max_router_difference;
  }

  static std::pair<double, double> getDistanceToPositionAlongLine(Eigen::Vector3d start,
                                                                  Eigen::Vector3d end,
                                                                  Eigen::Vector3d point,
                                                                  double step_size)
  {
    double current_distance = (point - start).norm();

    std::pair<double, double> closest =
        std::make_pair(current_distance, current_distance);

    for (double i = step_size; i < 1.0; i += step_size)
    {
      Eigen::Vector3d next_point = start + ((i / (end - start).norm()) * (end - start));
      current_distance = (point - next_point).norm();

      closest.first = std::min(closest.first, current_distance);
      closest.second = std::max(closest.second, current_distance);
    }

    current_distance = (point - end).norm();

    closest.first = std::min(closest.first, current_distance);
    closest.second = std::max(closest.second, current_distance);

    return closest;
  }

  static std::pair<double, double> getDistanceToClosestOccupiedBounded(
      std::shared_ptr<point_rtree> rtree, Eigen::Vector3d start, Eigen::Vector3d end,
      double max_search_distance, double radius, double step_size)
  {
    point bbx_min(std::min(start[0] - radius, end[0] - max_search_distance),
                  std::min(start[1] - radius, end[1] - max_search_distance),
                  std::min(start[2], end[2]) - radius);
    point bbx_max(std::max(start[0] + radius, end[0] + max_search_distance),
                  std::max(start[1] + radius, end[1] + max_search_distance),
                  std::max(start[2], end[2]) + radius);

    box query_box(bbx_min, bbx_max);
    std::vector<point> hits;
    rtree->query(boost::geometry::index::intersects(query_box), std::back_inserter(hits));

    std::vector<Eigen::Vector3d> points;
    points.push_back(start);
    if (start != end)
    {
      // Interpolate between start and end with step size
      for (double i = step_size; i < 1.0; i += step_size)
      {
        points.push_back(start + ((i / (end - start).norm()) * (end - start)));
      }
      points.push_back(end);
    }

    std::vector<std::vector<double>> closest(
        omp_get_max_threads(), std::vector<double>(points.size(), 10000000));

    double max_search_distance_squared = std::pow(max_search_distance, 2.0);

#pragma omp parallel for
    for (size_t i = 0; i < hits.size(); ++i)
    {
      Eigen::Vector3d point(hits[i].get<0>(), hits[i].get<1>(), hits[i].get<2>());

      if (point[2] < std::min(start[2], end[2]) ||
          point[2] > std::max(start[2], end[2]) + 0.1)
      {
        continue;
      }

      for (size_t j = 0; j < points.size(); ++j)
      {
        double distance_squared = (point - points[j]).squaredNorm();

        if (distance_squared > max_search_distance_squared)
        {
          continue;
        }

        closest[omp_get_thread_num()][j] =
            std::min(closest[omp_get_thread_num()][j], distance_squared);
      }
    }

    std::vector<double>& final_closest = closest[0];
    for (size_t i = 1; i < closest.size(); ++i)
    {
      for (size_t j = 0; j < closest[i].size(); ++j)
      {
        final_closest[j] = std::min(final_closest[j], closest[i][j]);
      }
    }

    std::pair<std::vector<double>::iterator, std::vector<double>::iterator> minmax =
        std::minmax_element(final_closest.begin(), final_closest.end());
    return std::make_pair(std::sqrt(*minmax.first), std::sqrt(*minmax.second));
  }
};

class Rrt
{
  typedef std::pair<point, std::shared_ptr<RrtNode>> value;
  typedef boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>>
      value_rtree;

public:
  Rrt(const ros::NodeHandle& nh);
  void octomapCallback(const octomap_msgs::Octomap& msg);

  void execute(const aeplanner_msgs::rrtGoalConstPtr& goal);
  void visualizeGoals(std::vector<geometry_msgs::Pose> goals);
  void visualizeNode(geometry_msgs::Point pos, int id = 0);
  void visualizePose(geometry_msgs::Pose pose, int id = 0);
  void visualizeEdge(std::shared_ptr<RrtNode> node, int id = 0);
  void visualizePath(std::shared_ptr<RrtNode> node);

  bool isInsideBoundaries(Eigen::Vector4d point);
  bool isInsideBoundaries(Eigen::Vector3d point);
  bool isInsideBoundaries(octomap::point3d point);

  point_rtree getRtree(std::shared_ptr<octomap::OcTree> ot);

  Eigen::Vector3d sample();
  std::shared_ptr<RrtNode> chooseParent(const std::shared_ptr<point_rtree>& stl_rtree,
                                        const value_rtree& rtree, Eigen::Vector3d,
                                        double l);
  void rewire(std::shared_ptr<octomap::OcTree> ot,
              const std::shared_ptr<point_rtree>& stl_rtree, const value_rtree& rtree,
              std::shared_ptr<RrtNode> new_node, double l, double r, double r_os);
  Eigen::Vector3d getNewPos(Eigen::Vector3d sampled, Eigen::Vector3d parent, double l);
  bool collisionLine(std::shared_ptr<octomap::OcTree> ot, Eigen::Vector3d p1,
                     Eigen::Vector3d p2, double r);
  std::shared_ptr<RrtNode> addNodeToTree(std::shared_ptr<octomap::OcTree> ot,
                                         value_rtree* rtree,
                                         std::shared_ptr<RrtNode> parent,
                                         Eigen::Vector3d new_pos);
  std::shared_ptr<RrtNode> getGoal(std::shared_ptr<octomap::OcTree> ot,
                                   const point_rtree& goal_tree,
                                   std::shared_ptr<RrtNode> new_node, double l, double r,
                                   double r_os);
  nav_msgs::Path getBestPath(const std::shared_ptr<point_rtree>& stl_rtree,
                             std::vector<std::shared_ptr<RrtNode>> goals);
  std::vector<geometry_msgs::Pose> checkIfGoalReached(std::shared_ptr<octomap::OcTree> ot,
                                                      const point_rtree& goal_tree,
                                                      std::shared_ptr<RrtNode> new_node,
                                                      double l, double r, double r_os);

  void configCallback(rrtplanner::LTLConfig& config, uint32_t level);

  void routerCallback(const dd_gazebo_plugins::Router::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  std::shared_ptr<octomap::OcTree> ot_;

  ros::Subscriber octomap_sub_;
  ros::Subscriber router_sub_;
  actionlib::SimpleActionServer<aeplanner_msgs::rrtAction> as_;

  std::string frame_id_;

  ros::Publisher path_pub_;
  double min_nodes_;
  double max_nodes_;
  double bounding_radius_;
  double bounding_overshoot_;
  double extension_range_;

  std::vector<double> boundary_min;
  std::vector<double> boundary_max;

  double ltl_lambda_;
  double ltl_min_distance_;
  double ltl_max_distance_;
  bool ltl_min_distance_active_;
  bool ltl_max_distance_active_;
  bool ltl_routers_active_;
  dynamic_reconfigure::Server<rrtplanner::LTLConfig> ltl_cs_;
  dynamic_reconfigure::Server<rrtplanner::LTLConfig>::CallbackType ltl_f_;
  nav_msgs::Path ltl_path_;
  double ltl_dist_add_path_;
  ros::Publisher ltl_path_pub_;
  ros::Publisher ltl_stats_pub_;
  int ltl_iterations_;
  double ltl_mean_closest_distance_;
  std::vector<double> ltl_closest_distance_;
  double ltl_max_search_distance_;
  std::vector<std::pair<octomap::point3d, double>> ltl_search_distances_;
  double ltl_step_size_;
  std::map<int, std::pair<geometry_msgs::Pose, double>> ltl_routers_;
};

float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                        float lsq, float rsq, const octomap::point3d& pt);
}  // namespace aeplanner_ns

#endif
