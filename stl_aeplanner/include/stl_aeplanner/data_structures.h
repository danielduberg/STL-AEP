#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <octomap/OcTree.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include <omp.h>
#include <queue>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <geometry_msgs/Pose.h>

namespace stl_aeplanner
{
// Rtree
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point;
typedef boost::geometry::model::box<point> box;

typedef boost::geometry::index::rtree<point, boost::geometry::index::rstar<16>> point_rtree;

class RRTNode : public std::enable_shared_from_this<RRTNode>
{
public:
  Eigen::Vector4d state_;
  std::shared_ptr<RRTNode> parent_;
  std::vector<std::shared_ptr<RRTNode>> children_;
  double gain_;
  bool gain_explicitly_calculated_;

  std::shared_ptr<point_rtree> score_rtree_;
  std::shared_ptr<RRTNode> score_parent_;
  double score_;
  std::shared_ptr<point_rtree> cost_rtree_;
  std::shared_ptr<RRTNode> cost_parent_;
  double cost_;

  RRTNode() : parent_(NULL), gain_(0.0), gain_explicitly_calculated_(false)
  {
  }

  std::shared_ptr<RRTNode> getCopyOfParentBranch()
  {
    std::shared_ptr<RRTNode> current_node = shared_from_this();
    std::shared_ptr<RRTNode> new_node;
    std::shared_ptr<RRTNode> new_child_node = NULL;

    while (current_node)
    {
      new_node = std::make_shared<RRTNode>();
      new_node->state_ = current_node->state_;
      new_node->gain_ = current_node->gain_;
      new_node->gain_explicitly_calculated_ = current_node->gain_explicitly_calculated_;
      new_node->parent_ = NULL;

      if (new_child_node)
      {
        new_node->children_.push_back(new_child_node);
        new_child_node->parent_ = new_node;
      }

      current_node = current_node->parent_;
      new_child_node = new_node;
    }

    return new_node;
  }

  double getDistanceGain(std::shared_ptr<point_rtree> rtree, double ltl_lambda, double min_distance,
                         double max_distance, bool min_distance_active, bool max_distance_active,
                         double max_search_distance, double radius, double step_size)
  {
    if (!min_distance_active && !max_distance_active)
    {
      return 1;
    }

    Eigen::Vector3d start;
    if (parent_)
    {
      start[0] = parent_->state_[0];
      start[1] = parent_->state_[1];
      start[2] = parent_->state_[2];
    }
    else
    {
      start[0] = state_[0];
      start[1] = state_[1];
      start[2] = state_[2];
    }
    Eigen::Vector3d end(state_[0], state_[1], state_[2]);

    std::pair<double, double> closest_distance =
        getDistanceToClosestOccupiedBounded(rtree, start, end, max_search_distance, radius, step_size);

    double distance_gain = 0;
    if (min_distance_active && max_distance_active)
    {
      distance_gain = std::min(10 * (closest_distance.first - min_distance), max_distance - closest_distance.second);
    }
    else if (min_distance_active)
    {
      distance_gain = 10 * (closest_distance.first - min_distance);
    }
    else
    {
      distance_gain = max_distance - closest_distance.second;
    }

    return distance_gain;
  }

  double getAltitudeGain(std::shared_ptr<point_rtree> rtree, double min_altitude, double max_altitude,
                         bool min_altitude_active, bool max_altitude_active, double max_search_distance, double radius,
                         double step_size)
  {
    if (!min_altitude_active && !max_altitude_active)
    {
      return 1;
    }

    Eigen::Vector3d start;
    start[0] = parent_ ? parent_->state_[0] : state_[0];
    start[1] = parent_ ? parent_->state_[1] : state_[1];
    start[2] = parent_ ? parent_->state_[2] : state_[2];
    Eigen::Vector3d end(state_[0], state_[1], state_[2]);

    std::pair<double, double> closest_altitude =
        getAltitudeClosestOccupiedBounded(rtree, start, end, max_search_distance, radius, step_size);

    double altitude_gain = 0;
    if (min_altitude_active && max_altitude_active)
    {
      altitude_gain = std::min(10 * (closest_altitude.first - min_altitude), max_altitude - closest_altitude.second);
    }
    else if (min_altitude_active)
    {
      altitude_gain = 10 * (closest_altitude.first - min_altitude);
    }
    else
    {
      altitude_gain = max_altitude - closest_altitude.second;
    }

    return altitude_gain;
  }

  static std::pair<double, double> getAltitudeClosestOccupiedBounded(std::shared_ptr<point_rtree> rtree,
                                                                     Eigen::Vector3d start, Eigen::Vector3d end,
                                                                     double max_search_distance, double radius,
                                                                     double step_size)
  {
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

    std::vector<std::vector<double>> closest(omp_get_max_threads(), std::vector<double>(points.size(), 10000000));

    double max_search_distance_squared = std::pow(max_search_distance, 2.0);

#pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i)
    {
      point bbx_min(points[i][0] - radius, points[i][1] - radius, points[i][2] - max_search_distance - radius);
      point bbx_max(points[i][0] + radius, points[i][1] + radius, points[i][2]);

      box query_box(bbx_min, bbx_max);
      std::vector<point> hits;
      rtree->query(boost::geometry::index::intersects(query_box), std::back_inserter(hits));

      for (size_t j = 0; j < hits.size(); ++j)
      {
        Eigen::Vector3d point(hits[j].get<0>(), hits[j].get<1>(), hits[j].get<2>());

        double distance_squared = (point - points[i]).squaredNorm();

        if (distance_squared > max_search_distance_squared)
        {
          continue;
        }

        closest[omp_get_thread_num()][i] = std::min(closest[omp_get_thread_num()][i], distance_squared);
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

  double score(std::shared_ptr<point_rtree> rtree, double ltl_lambda, double min_distance, double max_distance,
               bool min_distance_active, bool max_distance_active, double max_search_distance, double radius,
               double step_size, std::map<int, std::pair<geometry_msgs::Pose, double>> routers, bool routers_active,
               double lambda, double min_altitude, double max_altitude, bool min_altitude_active,
               bool max_altitude_active)
  {
    if (score_rtree_ && parent_ == score_parent_)
    {
      return score_;
    }

    score_rtree_ = rtree;
    score_parent_ = parent_;

    if (!this->parent_)
    {
      score_ = this->gain_;
      return score_;
    }

    double distance_gain = getDistanceGain(rtree, ltl_lambda, min_distance, max_distance, min_distance_active,
                                           max_distance_active, max_search_distance, radius, step_size);

    double altitude_gain = getAltitudeGain(rtree, min_altitude, max_altitude, min_altitude_active, max_altitude_active,
                                           max_search_distance, radius, step_size);

    double max_router_difference = distance_gain;
    if (routers_active)
    {
      max_router_difference = 10 * getMaxRouterDifference(routers, step_size);
    }

    score_ =
        this->parent_->score(rtree, ltl_lambda, min_distance, max_distance, min_distance_active, max_distance_active,
                             max_search_distance, radius, step_size, routers, routers_active, lambda, min_altitude,
                             max_altitude, min_altitude_active, max_altitude_active) +
        this->gain_ *
            exp(-lambda * (this->distance(this->parent_) *
                           std::fmax(std::exp(-ltl_lambda *
                                              std::min(std::min(distance_gain, max_router_difference), altitude_gain)),
                                     1)));
    return score_;
  }

  double cost(std::shared_ptr<point_rtree> rtree, double ltl_lambda, double min_distance, double max_distance,
              bool min_distance_active, bool max_distance_active, double max_search_distance, double radius,
              double step_size, std::map<int, std::pair<geometry_msgs::Pose, double>> routers, bool routers_active,
              double min_altitude, double max_altitude, bool min_altitude_active, bool max_altitude_active)
  {
    if (cost_rtree_ && parent_ == cost_parent_)
    {
      return cost_;
    }

    cost_rtree_ = rtree;
    cost_parent_ = parent_;

    if (!this->parent_)
    {
      cost_ = 0;
      return cost_;
    }

    double distance_gain = getDistanceGain(rtree, ltl_lambda, min_distance, max_distance, min_distance_active,
                                           max_distance_active, max_search_distance, radius, step_size);

    double altitude_gain = getAltitudeGain(rtree, min_altitude, max_altitude, min_altitude_active, max_altitude_active,
                                           max_search_distance, radius, step_size);

    double max_router_difference = distance_gain;
    if (routers_active)
    {
      max_router_difference = 10 * getMaxRouterDifference(routers, step_size);
    }

    cost_ = (this->distance(this->parent_) *
             std::fmax(std::exp(-ltl_lambda * std::min(std::min(distance_gain, max_router_difference), altitude_gain)),
                       1)) +
            this->parent_->cost(rtree, ltl_lambda, min_distance, max_distance, min_distance_active, max_distance_active,
                                max_search_distance, radius, step_size, routers, routers_active, min_altitude,
                                max_altitude, min_altitude_active, max_altitude_active);
    return cost_;
  }

  double getMaxRouterDifference(std::map<int, std::pair<geometry_msgs::Pose, double>> routers, double step_size)
  {
    Eigen::Vector3d start;
    if (parent_)
    {
      start[0] = parent_->state_[0];
      start[1] = parent_->state_[1];
      start[2] = parent_->state_[2];
    }
    else
    {
      start[0] = state_[0];
      start[1] = state_[1];
      start[2] = state_[2];
    }
    Eigen::Vector3d end(state_[0], state_[1], state_[2]);

    double max_router_difference = -10000000;

    for (auto it = routers.begin(); it != routers.end(); ++it)
    {
      Eigen::Vector3d point(it->second.first.position.x, it->second.first.position.y, it->second.first.position.z);

      std::pair<double, double> distance = getDistanceToPositionAlongLine(start, end, point, step_size);

      max_router_difference = std::max(max_router_difference, it->second.second - distance.second);
    }

    return max_router_difference;
  }

  static double getMaxRouterDifference(Eigen::Vector3d start, Eigen::Vector3d end,
                                       std::map<int, std::pair<geometry_msgs::Pose, double>> routers, double step_size)
  {
    double max_router_difference = 10000000;

    for (auto it = routers.begin(); it != routers.end(); ++it)
    {
      Eigen::Vector3d point(it->second.first.position.x, it->second.first.position.y, it->second.first.position.z);

      std::pair<double, double> distance = getDistanceToPositionAlongLine(start, end, point, step_size);

      max_router_difference = std::min(max_router_difference, distance.second);
    }

    return max_router_difference;
  }

  static std::pair<double, double> getDistanceToPositionAlongLine(Eigen::Vector3d start, Eigen::Vector3d end,
                                                                  Eigen::Vector3d point, double step_size)
  {
    double current_distance = (point - start).norm();

    std::pair<double, double> closest = std::make_pair(current_distance, current_distance);

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

  static std::pair<double, double> getDistanceToClosestOccupiedBounded(std::shared_ptr<point_rtree> rtree,
                                                                       Eigen::Vector3d start, Eigen::Vector3d end,
                                                                       double max_search_distance, double radius,
                                                                       double step_size)
  {
    point bbx_min(std::min(start[0], end[0]) - max_search_distance - radius,
                  std::min(start[1], end[1]) - max_search_distance - radius, std::min(start[2], end[2]));
    point bbx_max(std::max(start[0], end[0]) + max_search_distance + radius,
                  std::max(start[1], end[1]) + max_search_distance + radius, std::max(start[2], end[2]) + radius);

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

    std::vector<std::vector<double>> closest(omp_get_max_threads(), std::vector<double>(points.size(), 10000000));

    double max_search_distance_squared = std::pow(max_search_distance, 2.0);

#pragma omp parallel for
    for (size_t i = 0; i < hits.size(); ++i)
    {
      Eigen::Vector3d point(hits[i].get<0>(), hits[i].get<1>(), hits[i].get<2>());

      if (point[2] < std::min(start[2], end[2]) || point[2] > std::max(start[2], end[2]) + 0.1)
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

        closest[omp_get_thread_num()][j] = std::min(closest[omp_get_thread_num()][j], distance_squared);
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

  double distance(std::shared_ptr<RRTNode> other)
  {
    Eigen::Vector3d p3(this->state_[0], this->state_[1], this->state_[2]);
    Eigen::Vector3d q3(other->state_[0], other->state_[1], other->state_[2]);
    return (p3 - q3).norm();
  }
};  // namespace aeplanner
}  // namespace stl_aeplanner

#endif
