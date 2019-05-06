#include <stl_frontier_planner/stl_frontier_planner.h>

namespace stl_frontier_planner
{
STLFrontierPlanner::STLFrontierPlanner(const ros::NodeHandle& nh)
  : nh_(nh)
  , nh_priv_("~")
  , frame_id_("map")
  , path_pub_(nh_.advertise<visualization_msgs::Marker>("rrt_path", 1000))
  , octomap_sub_(nh_.subscribe("octomap", 1, &STLFrontierPlanner::octomapCallback, this))
  , as_(nh_, "rrt", boost::bind(&STLFrontierPlanner::execute, this, _1), false)
  , ltl_cs_(nh_priv_)
  , router_sub_(nh_.subscribe("/router", 10, &STLFrontierPlanner::routerCallback, this))
{
  // Set up dynamic reconfigure server
  ltl_f_ = boost::bind(&STLFrontierPlanner::configCallback, this, _1, _2);
  ltl_cs_.setCallback(ltl_f_);

  std::string ns = ros::this_node::getNamespace();
  bounding_radius_ = 0.5;
  bounding_overshoot_ = 0.5;
  extension_range_ = 1.0;
  min_nodes_ = 100;
  max_nodes_ = 100;
  if (!ros::param::get(ns + "/rrt/min_nodes", min_nodes_))
    ROS_WARN("No minimum nodes specified default is 100");
  if (!ros::param::get(ns + "/rrt/max_nodes", max_nodes_))
    ROS_WARN("No minimum nodes specified default is 100");
  if (!ros::param::get(ns + "/system/bbx/r", bounding_radius_))
    ROS_WARN("No bounding radius specified default is 0.5 m");
  if (!ros::param::get(ns + "/system/bbx/overshoot", bounding_overshoot_))
    ROS_WARN("No overshoot paramerer specified, default is 0.5 m");
  if (!ros::param::get(ns + "/aep/tree/extension_range", extension_range_))
    ROS_WARN("No extension range specified, default is 1.0 m");

  if (!ros::param::get(ns + "/boundary/min", boundary_min))
  {
    ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
  }
  if (!ros::param::get(ns + "/boundary/max", boundary_max))
  {
    ROS_WARN("No x-min value specified. Looking for %s", (ns + "/bbx/minX").c_str());
  }

  as_.start();
}

void STLFrontierPlanner::execute(const stl_aeplanner_msgs::rrtGoalConstPtr& goal)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;

  stl_aeplanner_msgs::rrtResult result;
  if (!ot)
  {
    ROS_WARN("No octomap received");
    as_.setSucceeded(result);
    return;
  }
  if (!goal->goal_poses.poses.size())
  {
    ROS_WARN("No goals received");
    as_.setSucceeded(result);
    return;
  }

  int N = min_nodes_;
  int M = max_nodes_;
  double l = extension_range_;
  double r = bounding_radius_;
  double r_os = bounding_overshoot_;
  std::vector<std::shared_ptr<RRTNode>> found_goals;

  value_rtree rtree;       // Initalize tree
  point_rtree goal_rtree;  // kd tree with all goals
  ROS_WARN_STREAM("We have this many goals for RRT: " << goal->goal_poses.poses.size());
  for (int i = 0; i < goal->goal_poses.poses.size(); ++i)
  {
    Eigen::Vector3d* g = new Eigen::Vector3d(goal->goal_poses.poses[i].position.x, goal->goal_poses.poses[i].position.y,
                                             goal->goal_poses.poses[i].position.z);
    goal_rtree.insert(point((*g)[0], (*g)[1], (*g)[2]));
  }

  // Initialize root position
  std::shared_ptr<RRTNode> root = std::make_shared<RRTNode>();
  root->pos[0] = goal->start.pose.position.x;
  root->pos[1] = goal->start.pose.position.y;
  root->pos[2] = goal->start.pose.position.z;
  root->parent = NULL;

  rtree.insert(std::make_pair(point(root->pos[0], root->pos[1], root->pos[2]), root));

  std::shared_ptr<point_rtree> stl_rtree = std::make_shared<point_rtree>(getRtree(ot));

  visualizeNode(goal->start.pose.position, 1000);
  visualizeGoals(goal->goal_poses.poses);

  for (int i = 0; (i < N) || (i < M && found_goals.empty()); ++i)
  {
    // Sample new position
    Eigen::Vector3d z_samp = sample();

    // Get nearest neighbour
    std::shared_ptr<RRTNode> z_parent = chooseParent(stl_rtree, rtree, z_samp, l);
    if (!z_parent)
      continue;

    // Calculate position for new node
    Eigen::Vector3d new_pos = getNewPos(z_samp, z_parent->pos, l);
    Eigen::Vector3d direction = new_pos - z_parent->pos;

    if (isInsideBoundaries(new_pos) && !collisionLine(ot, z_parent->pos, new_pos + direction.normalized() * r_os, r))
    {
      // Add node to tree
      std::shared_ptr<RRTNode> z_new = addNodeToTree(ot, &rtree, z_parent, new_pos);
      rewire(ot, stl_rtree, rtree, z_new, l, r, r_os);

      visualizeEdge(z_new, i);

      // Check if goal has been reached
      std::shared_ptr<RRTNode> tmp_goal = getGoal(ot, goal_rtree, z_new, l, r, r_os);
      if (tmp_goal)
      {
        found_goals.push_back(tmp_goal);
      }
    }
    else
    {
      --i;
    }
  }

  result.path = getBestPath(stl_rtree, found_goals);

  as_.setSucceeded(result);
}

point_rtree STLFrontierPlanner::getRtree(std::shared_ptr<octomap::OcTree> ot)
{
  point_rtree rtree;
  for (octomap::OcTree::leaf_iterator it = ot->begin_leafs(), it_end = ot->end_leafs(); it != it_end; ++it)
  {
    if (it->getLogOdds() > 0)
    {
      rtree.insert(point(it.getX(), it.getY(), it.getZ()));
    }
  }

  return rtree;
}

void STLFrontierPlanner::octomapCallback(const octomap_msgs::Octomap& msg)
{
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  if (ot)
    delete ot;
}

Eigen::Vector3d STLFrontierPlanner::sample()
{
  Eigen::Vector3d x_samp;
  for (int i = 0; i < 3; ++i)
    x_samp[i] = 60 * (((double)rand()) / ((double)RAND_MAX) - 0.5);

  return x_samp;
}

std::shared_ptr<RRTNode> STLFrontierPlanner::chooseParent(const std::shared_ptr<point_rtree>& stl_rtree, const value_rtree& rtree,
                                           Eigen::Vector3d node, double l)
{
  // TODO: How many neighbours to look for?
  std::vector<value> nearest;
  rtree.query(boost::geometry::index::nearest(point(node[0], node[1], node[2]), 15), std::back_inserter(nearest));

  // TODO: Check if correct
  std::shared_ptr<RRTNode> best_node;
  double best_cost;

  for (value item : nearest)
  {
    std::shared_ptr<RRTNode> current_node = item.second;

    double current_cost =
        current_node->cost(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_,
                           ltl_max_distance_active_, ltl_max_search_distance_, bounding_radius_, ltl_step_size_,
                           ltl_routers_, ltl_routers_active_);

    if (!best_node || current_cost < best_cost)
    {
      best_node = current_node;
      best_cost = current_cost;
    }
  }

  return best_node;
}

void STLFrontierPlanner::rewire(std::shared_ptr<octomap::OcTree> ot, const std::shared_ptr<point_rtree>& stl_rtree,
                                const value_rtree& rtree, std::shared_ptr<RRTNode> new_node, double l, double r,
                                double r_os)
{
  // TODO: How many neighbours to look for?
  std::vector<value> nearest;
  rtree.query(boost::geometry::index::nearest(point(new_node->pos[0], new_node->pos[1], new_node->pos[2]), 5),
              std::back_inserter(nearest));

  double new_cost = new_node->cost(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_,
                                   ltl_min_distance_active_, ltl_max_distance_active_, ltl_max_search_distance_,
                                   bounding_radius_, ltl_step_size_, ltl_routers_, ltl_routers_active_);

  for (value item : nearest)
  {
    std::shared_ptr<RRTNode> current_node = item.second;

    if (current_node->cost(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_,
                           ltl_max_distance_active_, ltl_max_search_distance_, bounding_radius_, ltl_step_size_,
                           ltl_routers_, ltl_routers_active_) > new_cost + (current_node->pos - new_node->pos).norm())
    {
      if (!collisionLine(ot, new_node->pos, current_node->pos + (current_node->pos - new_node->pos).normalized() * r_os,
                         r))
      {
        current_node->parent = new_node;
      }
    }
  }
}

Eigen::Vector3d STLFrontierPlanner::getNewPos(Eigen::Vector3d sampled, Eigen::Vector3d parent, double l)
{
  Eigen::Vector3d direction = sampled - parent;
  if (direction.norm() > l)
    direction = l * direction.normalized();

  return parent + direction;
}

std::shared_ptr<RRTNode> STLFrontierPlanner::addNodeToTree(std::shared_ptr<octomap::OcTree> ot, value_rtree* rtree,
                                                           std::shared_ptr<RRTNode> parent, Eigen::Vector3d new_pos)
{
  std::shared_ptr<RRTNode> new_node = std::make_shared<RRTNode>();
  new_node->pos = new_pos;

  new_node->parent = parent;
  parent->children.push_back(new_node);
  rtree->insert(std::make_pair(point(new_node->pos[0], new_node->pos[1], new_node->pos[2]), new_node));

  return new_node;
}

std::shared_ptr<RRTNode> STLFrontierPlanner::getGoal(std::shared_ptr<octomap::OcTree> ot, const point_rtree& goal_tree,
                                                     std::shared_ptr<RRTNode> new_node, double l, double r, double r_os)
{
  // TODO: How many neighbours to look for?
  std::vector<point> nearest;
  goal_tree.query(boost::geometry::index::nearest(point(new_node->pos[0], new_node->pos[1], new_node->pos[2]), 1),
                  std::back_inserter(nearest));

  for (point item : nearest)
  {
    Eigen::Vector3d goal_node(item.get<0>(), item.get<1>(), item.get<2>());

    if ((goal_node - new_node->pos).norm() < 1.5)
    {
      if (!collisionLine(ot, new_node->pos, goal_node + (goal_node - new_node->pos).normalized() * r_os, r))
      {
        return new_node;
      }
    }
  }

  return NULL;
}

nav_msgs::Path STLFrontierPlanner::getBestPath(const std::shared_ptr<point_rtree>& stl_rtree,
                                               std::vector<std::shared_ptr<RRTNode>> goals)
{
  nav_msgs::Path path;
  if (goals.size() == 0)
  {
    return path;
  }

  std::shared_ptr<RRTNode> best_node = goals[0];
  double best_cost = best_node->cost(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_,
                                     ltl_min_distance_active_, ltl_max_distance_active_, ltl_max_search_distance_,
                                     bounding_radius_, ltl_step_size_, ltl_routers_, ltl_routers_active_);

  for (int i = 0; i < goals.size(); ++i)
  {
    double current_cost = goals[i]->cost(stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_,
                                         ltl_min_distance_active_, ltl_max_distance_active_, ltl_max_search_distance_,
                                         bounding_radius_, ltl_step_size_, ltl_routers_, ltl_routers_active_);
    if (best_cost > current_cost)
    {
      best_node = goals[i];
      best_cost = current_cost;
    }
  }

  std::shared_ptr<RRTNode> n = best_node;
  for (int id = 0; n->parent; ++id)
  {
    geometry_msgs::PoseStamped p;
    p.pose.position.x = n->pos[0];
    p.pose.position.y = n->pos[1];
    p.pose.position.z = n->pos[2];
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    // Zero out rotation along
    // x and y axis so only
    // yaw is kept
    Eigen::Vector3d dir(n->pos[0] - n->parent->pos[0], n->pos[1] - n->parent->pos[1], 0);
    q.setFromTwoVectors(init, dir);

    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();

    path.poses.push_back(p);
    visualizePose(p.pose, id);

    n = n->parent;
  }

  visualizePath(best_node);
  return path;
}

std::vector<geometry_msgs::Pose> STLFrontierPlanner::checkIfGoalReached(std::shared_ptr<octomap::OcTree> ot,
                                                                        const point_rtree& goal_tree,
                                                                        std::shared_ptr<RRTNode> new_node, double l,
                                                                        double r, double r_os)
{
  std::vector<geometry_msgs::Pose> path;

  // TODO: How many neighbours to look for?
  std::vector<point> nearest;
  goal_tree.query(boost::geometry::index::nearest(point(new_node->pos[0], new_node->pos[1], new_node->pos[2]), 1),
                  std::back_inserter(nearest));

  for (point item : nearest)
  {
    Eigen::Vector3d goal_node(item.get<0>(), item.get<1>(), item.get<2>());

    if ((goal_node - new_node->pos).norm() < 2 * l)
    {
      if (!collisionLine(ot, new_node->pos, goal_node + (goal_node - new_node->pos).normalized() * r_os, r))
      {
        std::shared_ptr<RRTNode> n = new_node;
        for (int id = 0; n->parent; ++id)
        {
          geometry_msgs::Pose p;
          p.position.x = n->pos[0];
          p.position.y = n->pos[1];
          p.position.z = n->pos[2];
          Eigen::Quaternion<double> q;
          Eigen::Vector3d init(1.0, 0.0, 0.0);
          // Zero out rotation
          // along x and y axis
          // so only yaw is kept
          Eigen::Vector3d dir(n->pos[0] - n->parent->pos[0], n->pos[1] - n->parent->pos[1], 0);
          q.setFromTwoVectors(init, dir);

          p.orientation.x = q.x();
          p.orientation.y = q.y();
          p.orientation.z = q.z();
          p.orientation.w = q.w();

          path.push_back(p);
          visualizePose(p, id);

          n = n->parent;
        }

        visualizePath(new_node);
      }
    }
    return path;  // Should only be one
  }

  return path;
}

bool STLFrontierPlanner::collisionLine(std::shared_ptr<octomap::OcTree> ot, Eigen::Vector3d p1, Eigen::Vector3d p2,
                                       double r)
{
  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);
  octomap::point3d min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r, std::min(p1[2], p2[2]) - r);
  octomap::point3d max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r, std::max(p1[2], p2[2]) + r);
  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  octomap::point3d query(p2[0], p2[1], p2[2]);
  octomap::OcTreeNode* ot_res = ot->search(query);
  if (!ot_res)
    return true;

  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max), it_end = ot->end_leafs_bbx();
       it != it_end; ++it)
  {
    octomap::point3d pt(it.getX(), it.getY(), it.getZ());

    if (it->getLogOdds() > 0)
    {  // Node is occupied
      if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
      {
        return true;
      }
    }
  }

  // Check all unknown nodes
  /*
    octomap::point3d_list
    node_centers;
    ot_->getUnknownLeafCenters(node_centers,
    min, max);
    std::list<octomath::Vector3>::iterator
    it; for (it =
    node_centers.begin(); it
    != node_centers.end();
    ++it){
      if(CylTest_CapsFirst(start,
    end, lsq, rsq, *it) > 0 or
    (end-*it).norm() < r) {
        return true;
      }
    }
    */

  return false;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James -
// gjames@NVIDIA.com Lisc:
// Free code - no warranty &
// no money back.  Use it all
// you want Desc:
//    This function tests if
//    the 3D point 'pt' lies
//    within an arbitrarily
// oriented cylinder.  The
// cylinder is defined by an
// axis from 'pt1' to 'pt2',
// the axis having a length
// squared of 'lsq'
// (pre-compute for each
// cylinder to avoid repeated
// work!), and radius squared
// of 'rsq'.
//    The function tests
//    against the end caps
//    first, which is cheap ->
//    only
// a single dot product to
// test against the parallel
// cylinder caps.  If the
// point is within these, more
// work is done to find the
// distance of the point from
// the cylinder axis.
//    Fancy Math (TM) makes
//    the whole test possible
//    with only two
//    dot-products
// a subtract, and two
// multiplies.  For clarity,
// the 2nd mult is kept as a
// divide.  It might be faster
// to change this to a mult by
// also passing in 1/lengthsq
// and using that instead.
//    Elminiate the first 3
//    subtracts by specifying
//    the cylinder as a base
// point on one end cap and a
// vector to the other end cap
// (pass in {dx,dy,dz} instead
// of 'pt2' ).
//
//    The dot product is
//    constant along a plane
//    perpendicular to a
//    vector. The magnitude of
//    the cross product
//    divided by one vector
//    length is
// constant along a cylinder
// surface defined by the
// other vector as axis.
//
// Return:  -1.0 if point is
// outside the cylinder
// Return:  distance squared
// from cylinder axis if point
// is inside.
//
//-----------------------------------------------------------------------------
float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2, float lsq, float rsq,
                        const octomap::point3d& pt)
{
  float dx, dy,
      dz;  // vector d  from
           // line segment
           // point 1 to point
           // 2
  float pdx, pdy,
      pdz;  // vector pd from
            // point 1 to test
            // point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate
                           // so pt1 is
                           // origin.
                           // Make vector
                           // from
  dy = pt2.y() - pt1.y();  // pt1 to
                           // pt2.  Need
                           // for this
                           // is easily
                           // eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from
                           // pt1 to test
                           // point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors
  // to see if point lies
  // behind the cylinder cap
  // at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero
  // the point is behind the
  // pt1 cap. If greater than
  // the cylinder axis line
  // segment length squared
  // then the point is outside
  // the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
  {
    return (-1.0f);
  }
  else
  {
    // Point lies within the
    // parallel caps, so find
    // distance squared from
    // point to line, using
    // the fact that sin^2 +
    // cos^2 = 1 the dot =
    // cos() * |d||pd|, and
    // cross*cross = sin^2 *
    // |d|^2 * |pd|^2
    // Carefull: '*' means
    // mult for scalars and
    // dotproduct for vectors
    // In short, where dist is
    // pt distance to cyl
    // axis: dist = sin( pd to
    // d ) * |pd| distsq = dsq
    // = (1 - cos^2( pd to d))
    // * |pd|^2 dsq = ( 1 -
    // (pd * d)^2 / (|pd|^2 *
    // |d|^2) ) * |pd|^2 dsq =
    // pd * pd - dot * dot /
    // lengthsq
    //  where lengthsq is d*d
    //  or |d|^2 that is
    //  passed into this
    //  function

    // distance squared to the
    // cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
    {
      return (-1.0f);
    }
    else
    {
      return (dsq);  // return
                     // distance
                     // squared
                     // to
                     // axis
    }
  }
}

void STLFrontierPlanner::visualizeNode(geometry_msgs::Point pos, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "nodes";
  a.type = visualization_msgs::Marker::SPHERE;
  a.action = visualization_msgs::Marker::ADD;
  a.pose.position = pos;

  a.scale.x = 0.2;
  a.scale.y = 0.2;
  a.scale.z = 0.2;
  a.color.r = 0.2;
  a.color.g = 0.7;
  a.color.b = 0.2;
  ;
  a.color.a = 1;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;
  path_pub_.publish(a);
}

void STLFrontierPlanner::visualizePose(geometry_msgs::Pose pose, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "pose";
  a.type = visualization_msgs::Marker::ARROW;
  a.action = visualization_msgs::Marker::ADD;
  a.pose = pose;
  a.scale.x = 0.4;
  a.scale.y = 0.1;
  a.scale.z = 0.1;
  a.color.r = 1.0;
  a.color.g = 0.0;
  a.color.b = 0.0;
  a.color.a = 1.0;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;

  path_pub_.publish(a);
}

void STLFrontierPlanner::visualizeEdge(std::shared_ptr<RRTNode> node, int id)
{
  visualization_msgs::Marker a;
  a.header.stamp = ros::Time::now();
  a.header.seq = id;
  a.header.frame_id = frame_id_;
  a.id = id;
  a.ns = "vp_branches";
  a.type = visualization_msgs::Marker::ARROW;
  a.action = visualization_msgs::Marker::ADD;
  a.pose.position.x = node->parent->pos[0];
  a.pose.position.y = node->parent->pos[1];
  a.pose.position.z = node->parent->pos[2];
  Eigen::Quaternion<double> q;
  Eigen::Vector3d init(1.0, 0.0, 0.0);
  Eigen::Vector3d dir(node->pos[0] - node->parent->pos[0], node->pos[1] - node->parent->pos[1],
                      node->pos[2] - node->parent->pos[2]);
  q.setFromTwoVectors(init, dir);
  q.normalize();
  a.pose.orientation.x = q.x();
  a.pose.orientation.y = q.y();
  a.pose.orientation.z = q.z();
  a.pose.orientation.w = q.w();
  a.scale.x = dir.norm();
  a.scale.y = 0.05;
  a.scale.z = 0.05;
  a.color.r = 1.0;
  a.color.g = 0.3;
  a.color.b = 0.7;
  a.color.a = 1.0;
  a.lifetime = ros::Duration(5.0);
  a.frame_locked = false;

  path_pub_.publish(a);
}

void STLFrontierPlanner::visualizePath(std::shared_ptr<RRTNode> node)
{
  for (int id = 0; node->parent; ++id)
  {
    visualization_msgs::Marker a;
    a.header.stamp = ros::Time::now();
    a.header.seq = id;
    a.header.frame_id = frame_id_;
    a.id = id;
    a.ns = "path";
    a.type = visualization_msgs::Marker::ARROW;
    a.action = visualization_msgs::Marker::ADD;
    a.pose.position.x = node->parent->pos[0];
    a.pose.position.y = node->parent->pos[1];
    a.pose.position.z = node->parent->pos[2];
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->pos[0] - node->parent->pos[0], node->pos[1] - node->parent->pos[1],
                        node->pos[2] - node->parent->pos[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();
    a.pose.orientation.x = q.x();
    a.pose.orientation.y = q.y();
    a.pose.orientation.z = q.z();
    a.pose.orientation.w = q.w();
    a.scale.x = dir.norm();
    a.scale.y = 0.07;
    a.scale.z = 0.07;
    a.color.r = 0.7;
    a.color.g = 0.7;
    a.color.b = 0.3;
    a.color.a = 1.0;
    a.lifetime = ros::Duration(100.0);
    a.frame_locked = false;

    path_pub_.publish(a);

    node = node->parent;
  }
}

void STLFrontierPlanner::visualizeGoals(std::vector<geometry_msgs::Pose> goals)
{
  for (int i = 0; i < goals.size(); ++i)
  {
    visualization_msgs::Marker a;
    a.header.stamp = ros::Time::now();
    a.header.seq = i;
    a.header.frame_id = frame_id_;
    a.id = i;
    a.ns = "goals";
    a.type = visualization_msgs::Marker::ARROW;
    a.action = visualization_msgs::Marker::ADD;
    a.pose = goals[i];

    a.scale.x = 0.2;
    a.scale.y = 0.1;
    a.scale.z = 0.1;
    a.color.r = 1.0;
    a.color.g = 0.3;
    a.color.b = 0.7;
    a.color.a = 1;
    a.lifetime = ros::Duration(100.0);
    a.frame_locked = false;
    path_pub_.publish(a);
  }
}

void STLFrontierPlanner::configCallback(stl_frontier_planner::STLConfig& config, uint32_t level)
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
}

void STLFrontierPlanner::routerCallback(const dd_gazebo_plugins::Router::ConstPtr& msg)
{
  ltl_routers_[msg->id] = std::make_pair(msg->pose, msg->range);
}

bool STLFrontierPlanner::isInsideBoundaries(Eigen::Vector4d point)
{
  return point[0] > boundary_min[0] and point[0] < boundary_max[0] and point[1] > boundary_min[1] and
         point[1] < boundary_max[1] and point[2] > boundary_min[2] and point[2] < boundary_max[2];
}

bool STLFrontierPlanner::isInsideBoundaries(Eigen::Vector3d point)
{
  return point[0] > boundary_min[0] and point[0] < boundary_max[0] and point[1] > boundary_min[1] and
         point[1] < boundary_max[1] and point[2] > boundary_min[2] and point[2] < boundary_max[2];
}

bool STLFrontierPlanner::isInsideBoundaries(octomap::point3d point)
{
  return point.x() > boundary_min[0] and point.x() < boundary_max[0] and point.y() > boundary_min[1] and
         point.y() < boundary_max[1] and point.z() > boundary_min[2] and point.z() < boundary_max[2];
}

}  // namespace stl_frontier_planner
