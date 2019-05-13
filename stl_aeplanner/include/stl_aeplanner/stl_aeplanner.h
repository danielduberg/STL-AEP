#ifndef STL_AEPLANNER_H
#define STL_AEPLANNER_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <stl_aeplanner/data_structures.h>
#include <stl_aeplanner/param.h>
#include <stl_aeplanner_msgs/Reevaluate.h>

#include <stl_aeplanner/stl_aeplanner_viz.h>
#include <visualization_msgs/MarkerArray.h>

#include <actionlib/server/simple_action_server.h>
#include <stl_aeplanner_msgs/aeplannerAction.h>

#include <stl_aeplanner_msgs/BestNode.h>
#include <stl_aeplanner_msgs/Node.h>
#include <stl_aeplanner_msgs/Query.h>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <stl_aeplanner/STLConfig.h>

#include <dd_gazebo_plugins/Router.h>

namespace stl_aeplanner
{
typedef std::pair<point, std::shared_ptr<RRTNode>> value;
typedef boost::geometry::index::rtree<value, boost::geometry::index::rstar<16>> value_rtree;

class STLAEPlanner
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<stl_aeplanner_msgs::aeplannerAction> as_;

  Params params_;

  // Current state of agent (x, y, z, yaw)
  Eigen::Vector4d current_state_;
  bool current_state_initialized_;

  // Keep track of the best node and its score
  std::shared_ptr<RRTNode> best_node_;
  std::shared_ptr<RRTNode> best_branch_root_;

  std::shared_ptr<octomap::OcTree> ot_;

  // Subscribers
  ros::Subscriber octomap_sub_;
  ros::Subscriber agent_pose_sub_;
  ros::Subscriber router_sub_;

  // Publishers
  ros::Publisher rrt_marker_pub_;
  ros::Publisher gain_pub_;

  // Services
  ros::ServiceClient best_node_client_;
  ros::ServiceClient gp_query_client_;
  ros::ServiceServer reevaluate_server_;

  // LTL
  double ltl_lambda_;
  double ltl_min_distance_;
  double ltl_max_distance_;
  double ltl_min_altitude_;
  double ltl_max_altitude_;
  bool ltl_min_distance_active_;
  bool ltl_max_distance_active_;
  bool ltl_min_altitude_active_;
  bool ltl_max_altitude_active_;
  bool ltl_routers_active_;
  dynamic_reconfigure::Server<stl_aeplanner::STLConfig> ltl_cs_;
  dynamic_reconfigure::Server<stl_aeplanner::STLConfig>::CallbackType ltl_f_;
  nav_msgs::Path ltl_path_;
  double ltl_dist_add_path_;
  ros::Publisher ltl_path_pub_;
  ros::Publisher ltl_stats_pub_;
  int ltl_iterations_;
  double ltl_mean_closest_distance_;
  std::vector<double> ltl_closest_distance_;
  double ltl_mean_closest_altitude_;
  std::vector<double> ltl_closest_altitude_;
  double ltl_max_search_distance_;
  std::vector<std::pair<octomap::point3d, double>> ltl_search_distances_;
  double ltl_step_size_;
  std::map<int, std::pair<geometry_msgs::Pose, double>> ltl_routers_;

  double max_sampling_radius_squared_;

  point_rtree getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min, octomap::point3d max);

  // Service server callback
  bool reevaluate(stl_aeplanner_msgs::Reevaluate::Request& req, stl_aeplanner_msgs::Reevaluate::Response& res);

  // ---------------- Initialization ----------------
  std::shared_ptr<RRTNode> initialize(value_rtree* rtree, const Eigen::Vector4d& current_state);
  void initializeKDTreeWithPreviousBestBranch(value_rtree* rtree, std::shared_ptr<RRTNode> root);
  void reevaluatePotentialInformationGainRecursive(std::shared_ptr<RRTNode> node);

  // ---------------- Expand RRT Tree ----------------
  void expandRRT(std::shared_ptr<octomap::OcTree> ot, value_rtree* rtree, std::shared_ptr<point_rtree> stl_rtree,
                 const Eigen::Vector4d& current_state);

  Eigen::Vector4d sampleNewPoint();
  bool isInsideBoundaries(Eigen::Vector4d point);
  bool isInsideBoundaries(Eigen::Vector3d point);
  bool isInsideBoundaries(octomap::point3d point);
  bool collisionLine(std::shared_ptr<point_rtree> stl_rtree, Eigen::Vector4d p1, Eigen::Vector4d p2, double r);
  std::shared_ptr<RRTNode> chooseParent(const value_rtree& rtree, std::shared_ptr<point_rtree> stl_rtree,
                                        std::shared_ptr<RRTNode> node, double l);
  void rewire(const value_rtree& rtree, std::shared_ptr<point_rtree> stl_rtree, std::shared_ptr<RRTNode> new_node,
              double l, double r, double r_os);
  Eigen::Vector4d restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos);

  std::pair<double, double> getGain(std::shared_ptr<RRTNode> node);
  std::pair<double, double> gainCubature(Eigen::Vector4d state);

  // ---------------- Helpers ----------------
  //
  void publishEvaluatedNodesRecursive(std::shared_ptr<RRTNode> node);

  geometry_msgs::Pose vecToPose(Eigen::Vector4d state);

  float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2, float lsq, float rsq,
                          const octomap::point3d& pt);

  // ---------------- Frontier ----------------
  geometry_msgs::PoseArray getFrontiers();

  // LTL
  // void createLTLSearchDistance();
  // double getDistanceToClosestOccupiedBounded(std::shared_ptr<octomap::OcTree> ot,
  //                                            Eigen::Vector4d current_state);
  void configCallback(stl_aeplanner::STLConfig& config, uint32_t level);

  void routerCallback(const dd_gazebo_plugins::Router::ConstPtr& msg);

public:
  STLAEPlanner(const ros::NodeHandle& nh);

  void execute(const stl_aeplanner_msgs::aeplannerGoalConstPtr& goal);

  void octomapCallback(const octomap_msgs::Octomap& msg);
  void agentPoseCallback(const geometry_msgs::PoseStamped& msg);
};

}  // namespace stl_aeplanner

#endif  // STL_AEPLANNER_H
