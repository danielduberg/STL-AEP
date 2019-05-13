#include <stl_aeplanner/stl_aeplanner.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <stl_aeplanner_msgs/LTLStats.h>
#include <numeric>

#include <queue>

#include <omp.h>

namespace stl_aeplanner
{
STLAEPlanner::STLAEPlanner(const ros::NodeHandle& nh)
  : nh_(nh)
  , as_(nh_, "make_plan", boost::bind(&STLAEPlanner::execute, this, _1), false)
  , octomap_sub_(nh_.subscribe("octomap", 1, &STLAEPlanner::octomapCallback, this))
  , agent_pose_sub_(nh_.subscribe("agent_pose", 1, &STLAEPlanner::agentPoseCallback, this))
  , router_sub_(nh_.subscribe("/router", 10, &STLAEPlanner::routerCallback, this))
  , rrt_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("rrtree", 1000))
  , gain_pub_(nh_.advertise<stl_aeplanner_msgs::Node>("gain_node", 1000))
  , gp_query_client_(nh_.serviceClient<stl_aeplanner_msgs::Query>("gp_query_server"))
  , reevaluate_server_(nh_.advertiseService("reevaluate", &STLAEPlanner::reevaluate, this))
  , best_node_client_(nh_.serviceClient<stl_aeplanner_msgs::BestNode>("best_node_server"))
  , current_state_initialized_(false)
  , ot_(NULL)
  , ltl_cs_(nh_)
  , ltl_path_pub_(nh_.advertise<nav_msgs::Path>("ltl_path", 1000))
  , ltl_stats_pub_(nh_.advertise<stl_aeplanner_msgs::LTLStats>("ltl_stats", 1000))
  , ltl_iterations_(0)
{
  // Set up dynamic reconfigure server
  ltl_f_ = boost::bind(&STLAEPlanner::configCallback, this, _1, _2);
  ltl_cs_.setCallback(ltl_f_);

  params_ = readParams();
  max_sampling_radius_squared_ = pow(params_.max_sampling_radius, 2.0);

  as_.start();
}

void STLAEPlanner::execute(const stl_aeplanner_msgs::aeplannerGoalConstPtr& goal)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;
  Eigen::Vector4d current_state = current_state_;

  ROS_ERROR_STREAM("Execute start!");
  stl_aeplanner_msgs::aeplannerResult result;

  // Check if aeplanner has recieved agent's pose yet
  if (!current_state_initialized_)
  {
    ROS_WARN("Agent's pose not yet received");
    ROS_WARN("Make sure it is being published and correctly mapped");
    as_.setSucceeded(result);
    return;
  }
  if (!ot)
  {
    ROS_WARN("No octomap received");
    as_.setSucceeded(result);
    return;
  }

  octomap::point3d min(current_state[0] - params_.max_sampling_radius - ltl_max_search_distance_,
                       current_state[1] - params_.max_sampling_radius - ltl_max_search_distance_,
                       current_state[2] - params_.max_sampling_radius - ltl_max_search_distance_);

  octomap::point3d max(current_state[0] + params_.max_sampling_radius + ltl_max_search_distance_,
                       current_state[1] + params_.max_sampling_radius + ltl_max_search_distance_,
                       current_state[2] + params_.max_sampling_radius + ltl_max_search_distance_);

  std::shared_ptr<point_rtree> stl_rtree = std::make_shared<point_rtree>(getRtree(ot, min, max));

  value_rtree rtree;

  ROS_WARN("Init");
  std::shared_ptr<RRTNode> root = initialize(&rtree, current_state);
  ROS_WARN("expandRRT");
  ROS_WARN_STREAM(root->gain_ << " " << root->children_.size());
  if (root->gain_ > 0.75 or !root->children_.size() or
      root->score(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_,
                  ltl_max_distance_active_, ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_,
                  ltl_routers_, ltl_routers_active_, params_.lambda, ltl_min_altitude_, ltl_max_altitude_,
                  ltl_min_altitude_active_, ltl_max_altitude_active_) < params_.zero_gain)
  {
    expandRRT(ot, &rtree, stl_rtree, current_state);
  }
  else
  {
    best_node_ = root->children_[0];
  }

  ROS_WARN("getCopyOfParent");
  best_branch_root_ = best_node_->getCopyOfParentBranch();

  ROS_WARN("createRRTMarker");
  rrt_marker_pub_.publish(
      createRRTMarkerArray(root, stl_rtree, current_state, ltl_lambda_, ltl_min_distance_, ltl_max_distance_,
                           ltl_min_distance_active_, ltl_max_distance_active_, ltl_max_search_distance_,
                           params_.bounding_radius, ltl_step_size_, ltl_routers_, ltl_routers_active_, params_.lambda,
                           ltl_min_altitude_active_, ltl_max_altitude_active_, ltl_min_altitude_, ltl_max_altitude_));
  ROS_WARN("publishRecursive");
  publishEvaluatedNodesRecursive(root);

  ROS_WARN("extractPose");
  result.pose.pose = vecToPose(best_branch_root_->children_[0]->state_);
  if (best_node_->score(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_,
                        ltl_max_distance_active_, ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_,
                        ltl_routers_, ltl_routers_active_, params_.lambda, ltl_min_altitude_, ltl_max_altitude_,
                        ltl_min_altitude_active_, ltl_max_altitude_active_) > params_.zero_gain)
    result.is_clear = true;
  else
  {
    ROS_WARN("Getting frontiers");
    result.frontiers = getFrontiers();
    result.is_clear = false;
    best_branch_root_ = NULL;
  }
  as_.setSucceeded(result);
}

point_rtree STLAEPlanner::getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min, octomap::point3d max)
{
  point_rtree rtree;
  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max), it_end = ot->end_leafs_bbx();
       it != it_end; ++it)
  {
    if (it->getLogOdds() > 0)
    {
      rtree.insert(point(it.getX(), it.getY(), it.getZ()));
    }
  }

  return rtree;
}

std::shared_ptr<RRTNode> STLAEPlanner::initialize(value_rtree* rtree, const Eigen::Vector4d& current_state)
{
  best_node_ = nullptr;
  std::shared_ptr<RRTNode> root;

  if (best_branch_root_)
  {
    // Initialize with previous best branch

    // Discard root node from tree since we just were there...
    root = best_branch_root_->children_[0];
    root->parent_ = NULL;
    best_branch_root_->children_.clear();

    initializeKDTreeWithPreviousBestBranch(rtree, root);
    reevaluatePotentialInformationGainRecursive(root);
  }
  else
  {
    // Initialize without any previous branch
    root = std::make_shared<RRTNode>();
    root->state_[0] = current_state[0];
    root->state_[1] = current_state[1];
    root->state_[2] = current_state[2];
    rtree->insert(std::make_pair(point(root->state_[0], root->state_[1], root->state_[2]), root));
  }

  return root;
}

void STLAEPlanner::initializeKDTreeWithPreviousBestBranch(value_rtree* rtree, std::shared_ptr<RRTNode> root)
{
  std::shared_ptr<RRTNode> current_node = root;
  do
  {
    rtree->insert(
        std::make_pair(point(current_node->state_[0], current_node->state_[1], current_node->state_[2]), current_node));
    if (current_node->children_.size())
      current_node = current_node->children_[0];
  } while (current_node->children_.size());
}

void STLAEPlanner::reevaluatePotentialInformationGainRecursive(std::shared_ptr<RRTNode> node)
{
  ROS_DEBUG_STREAM("Reevaluating!!");
  std::pair<double, double> ret = gainCubature(node->state_);
  node->state_[3] = ret.second;  // Assign yaw angle that maximizes g
  node->gain_ = ret.first;
  for (typename std::vector<std::shared_ptr<RRTNode>>::iterator node_it = node->children_.begin();
       node_it != node->children_.end(); ++node_it)
    reevaluatePotentialInformationGainRecursive(*node_it);
  ROS_DEBUG_STREAM("Reevaluating done!!");
}

void STLAEPlanner::expandRRT(std::shared_ptr<octomap::OcTree> ot, value_rtree* rtree,
                             std::shared_ptr<point_rtree> stl_rtree, const Eigen::Vector4d& current_state)
{
  // Expand an RRT tree and calculate information gain in every node
  ROS_DEBUG_STREAM("Entering expanding RRT");
  for (int n = 0;
       (n < params_.init_iterations or
        (n < params_.cutoff_iterations and
         best_node_->score(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_,
                           ltl_max_distance_active_, ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_,
                           ltl_routers_, ltl_routers_active_, params_.lambda, ltl_min_altitude_, ltl_max_altitude_,
                           ltl_min_altitude_active_, ltl_max_altitude_active_) < params_.zero_gain)) and
       ros::ok();
       ++n)
  {
    ROS_DEBUG_STREAM("In expand RRT iteration: " << n);
    std::shared_ptr<RRTNode> new_node = std::make_shared<RRTNode>();
    std::shared_ptr<RRTNode> nearest;
    octomap::OcTreeNode* ot_result;

    // Sample new point around agent and check that
    // (1) it is within the boundaries
    // (2) it is in known space
    // (3) the path between the new node and it's parent does not contain any
    // obstacles

    do
    {
      Eigen::Vector4d offset = sampleNewPoint();
      new_node->state_ = current_state + offset;

      nearest = chooseParent(*rtree, stl_rtree, new_node, params_.extension_range);

      new_node->state_ = restrictDistance(nearest->state_, new_node->state_);

      ROS_DEBUG_STREAM("Trying node (" << new_node->state_[0] << ", " << new_node->state_[1] << ", "
                                       << new_node->state_[2] << ")");
      ROS_DEBUG_STREAM("    nearest (" << nearest->state_[0] << ", " << nearest->state_[1] << ", " << nearest->state_[2]
                                       << ")");
      ot_result = ot->search(octomap::point3d(new_node->state_[0], new_node->state_[1], new_node->state_[2]));
      if (ot_result == NULL)
        continue;
      ROS_DEBUG_STREAM("ot check done!");

      // ROS_DEBUG_STREAM("Inside boundaries?  " << isInsideBoundaries(new_node->state_));
      // ROS_DEBUG_STREAM("In known space?     " << ot_result);
      // ROS_DEBUG_STREAM("Collision?          " << collisionLine(
      //                      nearest->state_, new_node->state_,
      //                      params_.bounding_radius));
    } while (!isInsideBoundaries(new_node->state_) or !ot_result or
             collisionLine(stl_rtree, nearest->state_, new_node->state_, params_.bounding_radius));

    ROS_DEBUG_STREAM("New node (" << new_node->state_[0] << ", " << new_node->state_[1] << ", " << new_node->state_[2]
                                  << ")");
    // new_node is now ready to be added to tree
    new_node->parent_ = nearest;
    nearest->children_.push_back(new_node);

    // rewire tree with new node
    rewire(*rtree, stl_rtree, nearest, params_.extension_range, params_.bounding_radius, params_.d_overshoot_);

    // Calculate potential information gain for new_node
    ROS_DEBUG_STREAM("Get gain");
    std::pair<double, double> ret = getGain(new_node);
    new_node->state_[3] = ret.second;  // Assign yaw angle that maximizes g
    new_node->gain_ = ret.first;
    ROS_DEBUG_STREAM("Insert into KDTREE");
    rtree->insert(std::make_pair(point(new_node->state_[0], new_node->state_[1], new_node->state_[2]), new_node));

    // Update best node

    ROS_DEBUG_STREAM("Update best node");
    if (!best_node_ or
        new_node->score(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_,
                        ltl_max_distance_active_, ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_,
                        ltl_routers_, ltl_routers_active_, params_.lambda, ltl_min_altitude_, ltl_max_altitude_,
                        ltl_min_altitude_active_, ltl_max_altitude_active_) >
            best_node_->score(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_,
                              ltl_max_distance_active_, ltl_max_search_distance_, params_.bounding_radius,
                              ltl_step_size_, ltl_routers_, ltl_routers_active_, params_.lambda, ltl_min_altitude_,
                              ltl_max_altitude_, ltl_min_altitude_active_, ltl_max_altitude_active_))
      best_node_ = new_node;

    ROS_DEBUG_STREAM("iteration Done!");
  }

  ROS_DEBUG_STREAM("expandRRT Done!");
}

Eigen::Vector4d STLAEPlanner::sampleNewPoint()
{
  // Samples one point uniformly over a sphere with a radius of
  // param_.max_sampling_radius
  Eigen::Vector4d point(0.0, 0.0, 0.0, 0.0);
  do
  {
    for (int i = 0; i < 3; i++)
      point[i] = params_.max_sampling_radius * 2.0 * (((double)rand()) / ((double)RAND_MAX) - 0.5);
  } while (point.squaredNorm() > max_sampling_radius_squared_);

  return point;
}

std::shared_ptr<RRTNode> STLAEPlanner::chooseParent(const value_rtree& rtree, std::shared_ptr<point_rtree> stl_rtree,
                                                    std::shared_ptr<RRTNode> node, double l)
{
  // Find nearest neighbour
  // TODO: How many neighbours to look for?
  std::vector<value> nearest;
  rtree.query(boost::geometry::index::nearest(point(node->state_[0], node->state_[1], node->state_[2]), 15),
              std::back_inserter(nearest));

  // TODO: Check if correct
  std::shared_ptr<RRTNode> best_node;
  double best_cost;

  for (value item : nearest)
  {
    std::shared_ptr<RRTNode> current_node = item.second;

    double current_cost = current_node->cost(
        stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_,
        ltl_max_distance_active_, ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_, ltl_routers_,
        ltl_routers_active_, ltl_min_altitude_, ltl_max_altitude_, ltl_min_altitude_active_, ltl_max_altitude_active_);

    if (!best_node || current_cost < best_cost)
    {
      best_node = current_node;
      best_cost = current_cost;
    }
  }

  return best_node;
}

void STLAEPlanner::rewire(const value_rtree& rtree, std::shared_ptr<point_rtree> stl_rtree,
                          std::shared_ptr<RRTNode> new_node, double l, double r, double r_os)
{
  // TODO: How many neighbours to look for?
  std::vector<value> nearest;
  rtree.query(boost::geometry::index::nearest(point(new_node->state_[0], new_node->state_[1], new_node->state_[2]), 15),
              std::back_inserter(nearest));

  Eigen::Vector3d p1(new_node->state_[0], new_node->state_[1], new_node->state_[2]);

  double new_cost = new_node->cost(
      stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_, ltl_max_distance_active_,
      ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_, ltl_routers_, ltl_routers_active_,
      ltl_min_altitude_, ltl_max_altitude_, ltl_min_altitude_active_, ltl_max_altitude_active_);

#pragma omp parallel for
  for (size_t i = 0; i < nearest.size(); ++i)
  {
    std::shared_ptr<RRTNode> current_node = nearest[i].second;
    Eigen::Vector3d p2(current_node->state_[0], current_node->state_[1], current_node->state_[2]);

    if (current_node->cost(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_,
                           ltl_max_distance_active_, ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_,
                           ltl_routers_, ltl_routers_active_, ltl_min_altitude_, ltl_max_altitude_,
                           ltl_min_altitude_active_, ltl_max_altitude_active_) > new_cost + (p1 - p2).norm())
    {
      if (!collisionLine(stl_rtree, new_node->state_, current_node->state_, r))
      {
        current_node->parent_ = new_node;
      }
    }
  }
}

Eigen::Vector4d STLAEPlanner::restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos)
{
  // Check for collision
  Eigen::Vector3d origin(nearest[0], nearest[1], nearest[2]);
  Eigen::Vector3d direction(new_pos[0] - origin[0], new_pos[1] - origin[1], new_pos[2] - origin[2]);
  // if (direction.norm() > params_.extension_range)
  if (direction.norm() > params_.extension_range)
    direction = params_.extension_range * direction.normalized();

  new_pos[0] = origin[0] + direction[0];
  new_pos[1] = origin[1] + direction[1];
  new_pos[2] = origin[2] + direction[2];

  return new_pos;
}

std::pair<double, double> STLAEPlanner::getGain(std::shared_ptr<RRTNode> node)
{
  stl_aeplanner_msgs::Query srv;
  srv.request.point.x = node->state_[0];
  srv.request.point.y = node->state_[1];
  srv.request.point.z = node->state_[2];

  if (gp_query_client_.call(srv))
  {
    if (srv.response.sigma < params_.sigma_thresh)
    {
      double gain = srv.response.mu;
      double yaw = srv.response.yaw;
      return std::make_pair(gain, yaw);
    }
  }

  node->gain_explicitly_calculated_ = true;
  return gainCubature(node->state_);
}

bool STLAEPlanner::reevaluate(stl_aeplanner_msgs::Reevaluate::Request& req,
                              stl_aeplanner_msgs::Reevaluate::Response& res)
{
  ROS_DEBUG_STREAM("Reevaluate start!");
  for (std::vector<geometry_msgs::Point>::iterator it = req.point.begin(); it != req.point.end(); ++it)
  {
    Eigen::Vector4d pos(it->x, it->y, it->z, 0);
    std::pair<double, double> gain_response = gainCubature(pos);
    res.gain.push_back(gain_response.first);
    res.yaw.push_back(gain_response.second);
  }
  ROS_DEBUG_STREAM("Reevaluate done!");

  return true;
}

std::vector<octomap::point3d> getChildren(octomap::point3d current, octomap::point3d origin, double resolution)
{
  octomap::point3d transformed = current - origin;

  // Source:
  // https://stackoverflow.com/questions/29557459/round-to-nearest-multiple-of-a-number
  // transformed.x() = int(((transformed.x() + resolution / 2.0) / resolution)) *
  // resolution; transformed.y() = int(((transformed.y() + resolution / 2.0) /
  // resolution)) * resolution; transformed.z() = int(((transformed.z() + resolution
  // / 2.0) / resolution)) * resolution;

  std::vector<octomap::point3d> children;

  std::vector<double> x_values;
  if (transformed.x() == 0)
  {
    x_values.push_back(current.x() - resolution);
    x_values.push_back(current.x());
    x_values.push_back(current.x() + resolution);
  }
  else if (transformed.y() == 0 && transformed.z() == 0)
  {
    double sign = (transformed.x() > 0) ? 1 : -1;

    x_values.push_back(current.x() + (sign * resolution));
  }
  else
  {
    double sign = (transformed.x() > 0) ? 1 : -1;

    x_values.push_back(current.x() + (sign * resolution));
    x_values.push_back(current.x());
  }

  std::vector<double> y_values;
  if (transformed.y() == 0)
  {
    y_values.push_back(current.y() - resolution);
    y_values.push_back(current.y());
    y_values.push_back(current.y() + resolution);
  }
  else if (transformed.y() == 0 && transformed.z() == 0)
  {
    double sign = (transformed.y() > 0) ? 1 : -1;

    y_values.push_back(current.y() + (sign * resolution));
  }
  else
  {
    double sign = (transformed.y() > 0) ? 1 : -1;

    y_values.push_back(current.y() + (sign * resolution));
    y_values.push_back(current.y());
  }

  std::vector<double> z_values;
  if (transformed.z() == 0)
  {
    z_values.push_back(current.z() - resolution);
    z_values.push_back(current.z());
    z_values.push_back(current.z() + resolution);
  }
  else if (transformed.x() == 0 && transformed.y() == 0)
  {
    double sign = (transformed.z() > 0) ? 1 : -1;

    z_values.push_back(current.z() + (sign * resolution));
  }
  else
  {
    double sign = (transformed.z() > 0) ? 1 : -1;

    z_values.push_back(current.z() + (sign * resolution));
    z_values.push_back(current.z());
  }

  for (double x : x_values)
  {
    for (double y : y_values)
    {
      for (double z : z_values)
      {
        if (x == 0 && y == 0 && z == 0)
        {
          // TODO: Maybe some rounding errors?!
          continue;
        }

        children.emplace_back(current.x() + x, current.y() + y, current.z() + z);
      }
    }
  }

  return children;
}

std::pair<double, double> STLAEPlanner::gainCubature(Eigen::Vector4d state)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;

  // octomap::point3d origin(state[0], state[1], state[2]);

  // std::queue<octomap::point3d> open_set;
  // open_set.emplace(int(((origin.x() + ot->getResolution() / 2.0) /
  // ot->getResolution())) *
  //                      ot->getResolution(),
  //                  int(((origin.y() + ot->getResolution() / 2.0) /
  //                  ot->getResolution())) *
  //                      ot->getResolution(),
  //                  int(((origin.z() + ot->getResolution() / 2.0) /
  //                  ot->getResolution())) *
  //                      ot->getResolution());

  // std::set<std::tuple<double, double, double>> closed_set;
  // std::set<std::tuple<double, double, double>> unexplored_set;

  // std::vector<int> gain_per_yaw(360, 0);

  // double half_vfov = params_.vfov / 2.0;

  // octomap::point3d current;
  // while (!open_set.empty())
  // {
  //   current = open_set.front();
  //   open_set.pop();
  //   if (!closed_set.emplace(current.x(), current.y(), current.z()).second)
  //   {
  //     // Have already processed this
  //     continue;
  //   }

  //   octomap::OcTreeNode* result = ot->search(current);
  //   if (result)
  //   {
  //     if (result->getLogOdds() > 0)
  //     {
  //       // Already seen as Occupied space
  //       continue;
  //     }
  //   }
  //   else
  //   {
  //     if (!unexplored_set.emplace(current.x(), current.y(), current.z()).second)
  //     {
  //       // Already processed this
  //       continue;
  //     }
  //     else
  //     {
  //       // Is this correct?
  //       int yaw =
  //           std::round(std::atan2(origin.y() - current.y(), origin.x() - current.x()) *
  //                      180.0 / M_PI);
  //       if (yaw < 0)
  //       {
  //         yaw += 360;
  //       }

  //       gain_per_yaw[yaw]++;
  //     }
  //   }

  //   // Get children
  //   std::vector<octomap::point3d> children =
  //       getChildren(current, origin, ot->getResolution());

  //   for (octomap::point3d child : children)
  //   {
  //     // Check if inside boundaries
  //     if (!isInsideBoundaries(child))
  //     {
  //       // Outside boundaries
  //       continue;
  //     }

  //     double distance = (child - origin).norm();

  //     // Check if in sensor fov
  //     double vertical_angle =
  //         std::fabs(std::asin((child - origin).z() / distance)) * 180.0 / M_PI;
  //     if (vertical_angle > half_vfov || distance > params_.r_max)
  //     {
  //       // Outside sensor fov or range
  //       continue;
  //     }

  //     open_set.push(child);
  //   }
  // }

  // // double gain = pow(ot->getResolution(), 3.0) * free_set.size();

  // int half_hfov = params_.hfov / 2;

  // int best_yaw = 0;
  // int best_yaw_score =
  //     std::accumulate(gain_per_yaw.begin(), gain_per_yaw.begin() + half_hfov, 0) +
  //     std::accumulate(gain_per_yaw.rbegin(), gain_per_yaw.rend() - half_hfov,
  //                     0);  // FIXME: Should the second one be -?
  // int previous_yaw_score = best_yaw_score;

  // for (int yaw = 1; yaw < 360; ++yaw)
  // {
  //   int current_yaw_score =
  //       previous_yaw_score -
  //       gain_per_yaw[(yaw - 1 - half_hfov + gain_per_yaw.size()) % gain_per_yaw.size()]
  //       + gain_per_yaw[(yaw + half_hfov) % gain_per_yaw.size()];

  //   previous_yaw_score = current_yaw_score;
  //   if (current_yaw_score > best_yaw_score)
  //   {
  //     best_yaw_score = current_yaw_score;
  //     best_yaw = yaw;
  //   }
  // }

  // double gain = pow(ot->getResolution(), 3.0) * best_yaw_score;

  // return std::make_pair(gain, best_yaw * M_PI / 180.0);

  double gain = 0.0;

  // This function computes the gain
  double fov_y = params_.hfov, fov_p = params_.vfov;

  double dr = params_.dr, dphi = params_.dphi, dtheta = params_.dtheta;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;

  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec, dir;

  for (theta = -180; theta < 180; theta += dtheta)
  {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi)
    {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      for (r = params_.r_min; r < params_.r_max; r += dr)
      {
        vec[0] = state[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = state[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = state[2] + r * cos(phi_rad);
        dir = vec - origin;

        octomap::point3d query(vec[0], vec[1], vec[2]);
        octomap::OcTreeNode* result = ot->search(query);

        Eigen::Vector4d v(vec[0], vec[1], vec[2], 0);
        if (!isInsideBoundaries(v))
          break;
        if (result)
        {
          // Break if occupied so we don't count any information gain behind a wall.
          if (result->getLogOdds() > 0)
            break;
        }
        else
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
      }

      gain += g;
      gain_per_yaw[theta] += g;
    }
  }

  int best_yaw = 0;
  double best_yaw_score = 0;
  for (int yaw = -180; yaw < 180; yaw++)
  {
    double yaw_score = 0;
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++)
    {
      int theta = yaw + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      yaw_score += gain_per_yaw[theta];
    }

    if (best_yaw_score < yaw_score)
    {
      best_yaw_score = yaw_score;
      best_yaw = yaw;
    }
  }

  double r_max = params_.r_max;
  double h_max = params_.hfov / M_PI * 180;
  double v_max = params_.vfov / M_PI * 180;

  gain = best_yaw_score;  // / ((r_max*r_max*r_max/3) * h_max * (1-cos(v_max))) ;
  // ROS_ERROR_STREAM(gain);

  double yaw = M_PI * best_yaw / 180.f;

  state[3] = yaw;
  return std::make_pair(gain, yaw);
}

geometry_msgs::PoseArray STLAEPlanner::getFrontiers()
{
  geometry_msgs::PoseArray frontiers;

  stl_aeplanner_msgs::BestNode srv;
  srv.request.threshold = 0.75;
  if (best_node_client_.call(srv))
  {
    for (int i = 0; i < srv.response.best_node.size(); ++i)
    {
      geometry_msgs::Pose frontier;
      frontier.position = srv.response.best_node[i];
      frontiers.poses.push_back(frontier);
    }
  }
  else
  {
  }

  return frontiers;
}

bool STLAEPlanner::isInsideBoundaries(Eigen::Vector4d point)
{
  return point[0] > params_.boundary_min[0] and point[0] < params_.boundary_max[0] and
         point[1] > params_.boundary_min[1] and point[1] < params_.boundary_max[1] and
         point[2] > params_.boundary_min[2] and point[2] < params_.boundary_max[2];
}

bool STLAEPlanner::isInsideBoundaries(Eigen::Vector3d point)
{
  return point[0] > params_.boundary_min[0] and point[0] < params_.boundary_max[0] and
         point[1] > params_.boundary_min[1] and point[1] < params_.boundary_max[1] and
         point[2] > params_.boundary_min[2] and point[2] < params_.boundary_max[2];
}

bool STLAEPlanner::isInsideBoundaries(octomap::point3d point)
{
  return point.x() > params_.boundary_min[0] and point.x() < params_.boundary_max[0] and
         point.y() > params_.boundary_min[1] and point.y() < params_.boundary_max[1] and
         point.z() > params_.boundary_min[2] and point.z() < params_.boundary_max[2];
}

bool STLAEPlanner::collisionLine(std::shared_ptr<point_rtree> stl_rtree, Eigen::Vector4d p1, Eigen::Vector4d p2,
                                 double r)
{
  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);

  point bbx_min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r, std::min(p1[2], p2[2]) - r);
  point bbx_max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r, std::max(p1[2], p2[2]) + r);

  box query_box(bbx_min, bbx_max);
  std::vector<point> hits;
  stl_rtree->query(boost::geometry::index::intersects(query_box), std::back_inserter(hits));

  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  for (size_t i = 0; i < hits.size(); ++i)
  {
    octomap::point3d pt(hits[i].get<0>(), hits[i].get<1>(), hits[i].get<2>());

    if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
    {
      return true;
    }
  }

  return false;
}

void STLAEPlanner::octomapCallback(const octomap_msgs::Octomap& msg)
{
  ROS_DEBUG_STREAM("Freeing ot_");
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  delete ot;
  ROS_DEBUG_STREAM("Freeing ot_ done:");
}

void STLAEPlanner::publishEvaluatedNodesRecursive(std::shared_ptr<RRTNode> node)
{
  if (!node)
    return;
  for (typename std::vector<std::shared_ptr<RRTNode>>::iterator node_it = node->children_.begin();
       node_it != node->children_.end(); ++node_it)
  {
    if ((*node_it)->gain_explicitly_calculated_)
    {
      stl_aeplanner_msgs::Node pig_node;
      pig_node.gain = (*node_it)->gain_;
      // ROS_ERROR_STREAM("GAIN: " << pig_node.gain);
      pig_node.pose.pose.position.x = (*node_it)->state_[0];
      pig_node.pose.pose.position.y = (*node_it)->state_[1];
      pig_node.pose.pose.position.z = (*node_it)->state_[2];
      tf2::Quaternion q;
      q.setRPY(0, 0, (*node_it)->state_[3]);
      pig_node.pose.pose.orientation.x = q.x();
      pig_node.pose.pose.orientation.y = q.y();
      pig_node.pose.pose.orientation.z = q.z();
      pig_node.pose.pose.orientation.w = q.w();
      gain_pub_.publish(pig_node);
    }

    publishEvaluatedNodesRecursive(*node_it);
  }
}

void STLAEPlanner::agentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_state_[0] = msg.pose.position.x;
  current_state_[1] = msg.pose.position.y;
  current_state_[2] = msg.pose.position.z;
  current_state_[3] = tf2::getYaw(msg.pose.orientation);

  current_state_initialized_ = true;

  // LTL Path
  bool add_to_ltl_path = true;
  if (ltl_path_.poses.size() != 0)
  {
    Eigen::Vector3d last_state(ltl_path_.poses[ltl_path_.poses.size() - 1].pose.position.x,
                               ltl_path_.poses[ltl_path_.poses.size() - 1].pose.position.y,
                               ltl_path_.poses[ltl_path_.poses.size() - 1].pose.position.z);
    Eigen::Vector3d new_state(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    if ((last_state - new_state).norm() < ltl_dist_add_path_)
    {
      add_to_ltl_path = false;
    }
  }

  if (add_to_ltl_path)
  {
    ltl_path_.poses.push_back(msg);

    ROS_DEBUG("Publish LTL Path");
    ltl_path_.header.frame_id = "map";
    ltl_path_.header.stamp = ros::Time::now();
    ltl_path_pub_.publish(ltl_path_);
  }

  // Stats
  if (ot_)
  {
    ROS_DEBUG("Publish LTL Stats");
    stl_aeplanner_msgs::LTLStats ltl_stats;
    ltl_stats.header.stamp = ros::Time::now();
    ltl_stats.ltl_min_distance = (ltl_min_distance_active_) ? ltl_min_distance_ : -1;
    ltl_stats.ltl_max_distance = (ltl_max_distance_active_) ? ltl_max_distance_ : -1;
    ltl_stats.ltl_min_altitude = (ltl_min_altitude_active_) ? ltl_min_altitude_ : -1;
    ltl_stats.ltl_max_altitude = (ltl_max_altitude_active_) ? ltl_max_altitude_ : -1;

    Eigen::Vector3d position(current_state_[0], current_state_[1], current_state_[2]);

    std::shared_ptr<octomap::OcTree> ot = ot_;

    octomap::point3d min(position[0] - ltl_max_search_distance_, position[1] - ltl_max_search_distance_,
                         position[2] - ltl_max_search_distance_);

    octomap::point3d max(position[0] + ltl_max_search_distance_, position[1] + ltl_max_search_distance_,
                         position[2] + ltl_max_search_distance_);

    std::shared_ptr<point_rtree> rtree = std::make_shared<point_rtree>(getRtree(ot, min, max));

    std::pair<double, double> closest_distance = RRTNode::getDistanceToClosestOccupiedBounded(
        rtree, position, position, ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_);

    ltl_iterations_++;

    if (closest_distance.first >= ltl_max_search_distance_)
    {
      return;
    }

    ltl_stats.current_closest_distance = closest_distance.first;

    if (ltl_iterations_ == 1)
    {
      ltl_mean_closest_distance_ = closest_distance.first;
    }
    else
    {
      ltl_mean_closest_distance_ += (closest_distance.first - ltl_mean_closest_distance_) / ltl_iterations_;
    }

    ltl_stats.mean_closest_distance = ltl_mean_closest_distance_;

    ltl_stats.closest_router_distance =
        (ltl_routers_active_) ? RRTNode::getMaxRouterDifference(position, position, ltl_routers_, ltl_step_size_) : -1;

    // Altitude
    std::pair<double, double> closest_altitude = RRTNode::getAltitudeClosestOccupiedBounded(
        rtree, position, position, ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_);

    if (closest_altitude.first >= ltl_max_search_distance_)
    {
      return;
    }

    ltl_stats.current_closest_altitude = closest_altitude.first;

    if (ltl_iterations_ == 1)
    {
      ltl_mean_closest_altitude_ = closest_altitude.first;
    }
    else
    {
      ltl_mean_closest_altitude_ += (closest_altitude.first - ltl_mean_closest_altitude_) / ltl_iterations_;
    }

    ltl_stats.mean_closest_altitude = ltl_mean_closest_altitude_;

    // Publish
    ltl_stats_pub_.publish(ltl_stats);
  }
}

geometry_msgs::Pose STLAEPlanner::vecToPose(Eigen::Vector4d state)
{
  tf::Vector3 origin(state[0], state[1], state[2]);
  double yaw = state[3];

  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, yaw);
  tf::Pose pose_tf(quat, origin);

  geometry_msgs::Pose pose;
  tf::poseTFToMsg(pose_tf, pose);

  return pose;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James - gjames@NVIDIA.com
// Lisc: Free code - no warranty & no money back.  Use it all you want
// Desc:
//    This function tests if the 3D point 'pt' lies within an arbitrarily
// oriented cylinder.  The cylinder is defined by an axis from 'pt1' to 'pt2',
// the axis having a length squared of 'lsq' (pre-compute for each cylinder
// to avoid repeated work!), and radius squared of 'rsq'.
//    The function tests against the end caps first, which is cheap -> only
// a single dot product to test against the parallel cylinder caps.  If the
// point is within these, more work is done to find the distance of the point
// from the cylinder axis.
//    Fancy Math (TM) makes the whole test possible with only two dot-products
// a subtract, and two multiplies.  For clarity, the 2nd mult is kept as a
// divide.  It might be faster to change this to a mult by also passing in
// 1/lengthsq and using that instead.
//    Elminiate the first 3 subtracts by specifying the cylinder as a base
// point on one end cap and a vector to the other end cap (pass in {dx,dy,dz}
// instead of 'pt2' ).
//
//    The dot product is constant along a plane perpendicular to a vector.
//    The magnitude of the cross product divided by one vector length is
// constant along a cylinder surface defined by the other vector as axis.
//
// Return:  -1.0 if point is outside the cylinder
// Return:  distance squared from cylinder axis if point is inside.
//
//-----------------------------------------------------------------------------
float STLAEPlanner::CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2, float lsq, float rsq,
                                      const octomap::point3d& pt)
{
  float dx, dy, dz;     // vector d  from line segment point 1 to point 2
  float pdx, pdy, pdz;  // vector pd from point 1 to test point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate so pt1 is origin.  Make vector from
  dy = pt2.y() - pt1.y();  // pt1 to pt2.  Need for this is easily eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from pt1 to test point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors to see if point lies behind the
  // cylinder cap at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero the point is behind the pt1 cap.
  // If greater than the cylinder axis line segment length squared
  // then the point is outside the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
    return (-1.0f);
  else
  {
    // Point lies within the parallel caps, so find
    // distance squared from point to line, using the fact that sin^2 + cos^2 = 1
    // the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
    // Carefull: '*' means mult for scalars and dotproduct for vectors
    // In short, where dist is pt distance to cyl axis:
    // dist = sin( pd to d ) * |pd|
    // distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
    // dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
    // dsq = pd * pd - dot * dot / lengthsq
    //  where lengthsq is d*d or |d|^2 that is passed into this function

    // distance squared to the cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
      return (-1.0f);
    else
      return (dsq);  // return distance squared to axis
  }
}

struct
{
  bool operator()(const std::pair<octomap::point3d, double>& lhs, const std::pair<octomap::point3d, double>& rhs) const
  {
    return lhs.second < rhs.second;
  }
} compareByDistance;

// void STLAEPlanner::createLTLSearchDistance()
// {
//   if (!ot_)
//   {
//     return;
//   }

//   ltl_search_distances_.clear();

//   double res = ot_->getResolution();

//   for (double x = -ltl_max_search_distance_; x <= 0; x += res)
//   {
//     for (double y = -ltl_max_search_distance_; y <= 0; y += res)
//     {
//       double distance = std::hypot(x, y);

//       if (distance <= ltl_max_search_distance_)
//       {
//         ltl_search_distances_.emplace_back(octomap::point3d(x, y, 0), distance);
//         ltl_search_distances_.emplace_back(octomap::point3d(x, -y, 0), distance);
//         ltl_search_distances_.emplace_back(octomap::point3d(-x, y, 0), distance);
//         ltl_search_distances_.emplace_back(octomap::point3d(-x, -y, 0), distance);
//       }
//     }
//   }

//   std::sort(ltl_search_distances_.begin(), ltl_search_distances_.end(),
//             compareByDistance);
// }

// double STLAEPlanner::getDistanceToClosestOccupiedBounded(std::shared_ptr<octomap::OcTree>
// ot,
//                                                       Eigen::Vector4d current_state)
// {
//   octomap::point3d state(current_state[0], current_state[1], current_state[2]);

//   for (std::pair<octomap::point3d, double> point : ltl_search_distances_)
//   {
//     octomap::OcTreeNode* node = ot->search(state + point.first);
//     if (node)
//     {
//       if (ot->isNodeOccupied(node))
//       {
//         return point.second;
//       }
//     }
//   }

//   return 10000000;
// }

void STLAEPlanner::configCallback(stl_aeplanner::STLConfig& config, uint32_t level)
{
  ltl_lambda_ = config.lambda;
  ltl_min_distance_ = config.min_distance;
  ltl_max_distance_ = config.max_distance;
  ltl_min_distance_active_ = config.min_distance_active;
  ltl_max_distance_active_ = config.max_distance_active;
  ltl_routers_active_ = config.routers_active;
  ltl_dist_add_path_ = config.distance_add_path;
  ltl_max_search_distance_ = config.max_search_distance;
  ltl_step_size_ = config.step_size;

  ltl_min_altitude_ = config.min_altitude;
  ltl_max_altitude_ = config.max_altitude;
  ltl_min_altitude_active_ = config.min_altitude_active;
  ltl_max_altitude_active_ = config.max_altitude_active;
}

void STLAEPlanner::routerCallback(const dd_gazebo_plugins::Router::ConstPtr& msg)
{
  ltl_routers_[msg->id] = std::make_pair(msg->pose, msg->range);
}

}  // namespace stl_aeplanner
