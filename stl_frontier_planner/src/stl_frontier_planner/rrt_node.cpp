#include <stl_frontier_planner/rrt_node.h>

#include <omp.h>

namespace stl_frontier_planner
{
double RRTNode::getDistanceGain(std::shared_ptr<point_rtree> rtree, double ltl_lambda, double min_distance,
                                double max_distance, bool min_distance_active, bool max_distance_active,
                                double max_search_distance, double radius, double step_size)
{
  if (!min_distance_active && !max_distance_active)
  {
    return 1;
  }

  Eigen::Vector3d start = (parent) ? parent->pos : pos;
  Eigen::Vector3d end = pos;

  std::pair<double, double> closest_distance =
      getDistanceToClosestOccupiedBounded(rtree, start, end, max_search_distance, radius, step_size);

  double distance_gain = 0;
  if (min_distance_active && max_distance_active)
  {
    distance_gain = std::min(closest_distance.first - min_distance, max_distance - closest_distance.second);
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

double RRTNode::cost(std::shared_ptr<point_rtree> rtree, double ltl_lambda, double min_distance, double max_distance,
                     bool min_distance_active, bool max_distance_active, double max_search_distance, double radius,
                     double step_size, std::map<int, std::pair<geometry_msgs::Pose, double>> routers,
                     bool routers_active)
{
  if (!parent)
  {
    return 0;
  }

  double distance_gain = getDistanceGain(rtree, ltl_lambda, min_distance, max_distance, min_distance_active,
                                         max_distance_active, max_search_distance, radius, step_size);

  double max_router_difference = distance_gain;
  if (routers_active)
  {
    max_router_difference = getMaxRouterDifference(routers, step_size);
  }

  return ((pos - parent->pos).norm() *
          std::fmax(std::exp(-ltl_lambda * std::min(distance_gain, max_router_difference)), 1)) +
         parent->cost(rtree, ltl_lambda, min_distance, max_distance, min_distance_active, max_distance_active,
                      max_search_distance, radius, step_size, routers, routers_active);
}

double RRTNode::getMaxRouterDifference(std::map<int, std::pair<geometry_msgs::Pose, double>> routers, double step_size)
{
  Eigen::Vector3d start = (parent) ? parent->pos : pos;
  Eigen::Vector3d end = pos;

  double max_router_difference = -10000000;

  for (auto it = routers.begin(); it != routers.end(); ++it)
  {
    Eigen::Vector3d point(it->second.first.position.x, it->second.first.position.y, it->second.first.position.z);

    std::pair<double, double> distance = getDistanceToPositionAlongLine(start, end, point, step_size);

    max_router_difference = std::max(max_router_difference, it->second.second - distance.second);
  }

  return max_router_difference;
}

std::pair<double, double> RRTNode::getDistanceToPositionAlongLine(Eigen::Vector3d start, Eigen::Vector3d end,
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

std::pair<double, double> RRTNode::getDistanceToClosestOccupiedBounded(std::shared_ptr<point_rtree> rtree,
                                                                              Eigen::Vector3d start,
                                                                              Eigen::Vector3d end,
                                                                              double max_search_distance, double radius,
                                                                              double step_size)
{
  point bbx_min(std::min(start[0] - radius, end[0] - max_search_distance),
                std::min(start[1] - radius, end[1] - max_search_distance), std::min(start[2], end[2]) - radius);
  point bbx_max(std::max(start[0] + radius, end[0] + max_search_distance),
                std::max(start[1] + radius, end[1] + max_search_distance), std::max(start[2], end[2]) + radius);

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
}  // namespace stl_frontier_planner