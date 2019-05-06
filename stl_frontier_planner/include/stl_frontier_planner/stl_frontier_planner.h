#ifndef STL_FRONTIER_PLANNER_H
#define STL_FRONTIER_PLANNER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <actionlib/server/simple_action_server.h>
#include <stl_aeplanner_msgs/rrtAction.h>

#include <nav_msgs/Path.h>

#include <dd_gazebo_plugins/Router.h>
#include <dynamic_reconfigure/server.h>
#include <stl_frontier_planner/STLConfig.h>

#include <stl_frontier_planner/rrt_node.h>

namespace stl_frontier_planner
{
class STLFrontierPlanner
{
  typedef std::pair<point, std::shared_ptr<RRTNode>> value;
  typedef boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>> value_rtree;

public:
  STLFrontierPlanner(const ros::NodeHandle& nh);
  void octomapCallback(const octomap_msgs::Octomap& msg);

  void execute(const stl_aeplanner_msgs::rrtGoal::ConstPtr& goal);
  void visualizeGoals(std::vector<geometry_msgs::Pose> goals);
  void visualizeNode(geometry_msgs::Point pos, int id = 0);
  void visualizePose(geometry_msgs::Pose pose, int id = 0);
  void visualizeEdge(std::shared_ptr<RRTNode> node, int id = 0);
  void visualizePath(std::shared_ptr<RRTNode> node);

  bool isInsideBoundaries(Eigen::Vector4d point);
  bool isInsideBoundaries(Eigen::Vector3d point);
  bool isInsideBoundaries(octomap::point3d point);

  point_rtree getRtree(std::shared_ptr<octomap::OcTree> ot);

  Eigen::Vector3d sample();
  std::shared_ptr<RRTNode> chooseParent(const std::shared_ptr<point_rtree>& stl_rtree, const value_rtree& rtree,
                                        Eigen::Vector3d, double l);
  void rewire(std::shared_ptr<octomap::OcTree> ot, const std::shared_ptr<point_rtree>& stl_rtree,
              const value_rtree& rtree, std::shared_ptr<RRTNode> new_node, double l, double r, double r_os);
  Eigen::Vector3d getNewPos(Eigen::Vector3d sampled, Eigen::Vector3d parent, double l);
  bool collisionLine(std::shared_ptr<octomap::OcTree> ot, Eigen::Vector3d p1, Eigen::Vector3d p2, double r);
  std::shared_ptr<RRTNode> addNodeToTree(std::shared_ptr<octomap::OcTree> ot, value_rtree* rtree,
                                         std::shared_ptr<RRTNode> parent, Eigen::Vector3d new_pos);
  std::shared_ptr<RRTNode> getGoal(std::shared_ptr<octomap::OcTree> ot, const point_rtree& goal_tree,
                                   std::shared_ptr<RRTNode> new_node, double l, double r, double r_os);
  nav_msgs::Path getBestPath(const std::shared_ptr<point_rtree>& stl_rtree,
                             std::vector<std::shared_ptr<RRTNode>> goals);
  std::vector<geometry_msgs::Pose> checkIfGoalReached(std::shared_ptr<octomap::OcTree> ot, const point_rtree& goal_tree,
                                                      std::shared_ptr<RRTNode> new_node, double l, double r,
                                                      double r_os);

  void configCallback(stl_frontier_planner::STLConfig& config, uint32_t level);

  void routerCallback(const dd_gazebo_plugins::Router::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  std::shared_ptr<octomap::OcTree> ot_;

  ros::Subscriber octomap_sub_;
  ros::Subscriber router_sub_;
  actionlib::SimpleActionServer<stl_aeplanner_msgs::rrtAction> as_;

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
  dynamic_reconfigure::Server<stl_frontier_planner::STLConfig> ltl_cs_;
  dynamic_reconfigure::Server<stl_frontier_planner::STLConfig>::CallbackType ltl_f_;
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

float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2, float lsq, float rsq,
                        const octomap::point3d& pt);
}  // namespace stl_frontier_planner

#endif  // STL_FRONTIER_PLANNER_H