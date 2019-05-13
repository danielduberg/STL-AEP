#include <geometry_msgs/PoseStamped.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <stl_aeplanner_msgs/LTLStats.h>
#include <stl_aeplanner_msgs/Volume.h>
#include <tf2/utils.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

std::ofstream stats_file;
std::ofstream pose_file;
std::ofstream map_file;

double total_volume;
octomap::point3d bbx_min;
octomap::point3d bbx_max;

ros::Publisher volume_pub;
double volume_scaler;

double volume_complete;
double time_limit;

bool stats_done;
bool pose_done;
bool map_done;

ros::Time start;

void LTLStatsCallback(const stl_aeplanner_msgs::LTLStats::ConstPtr& stats)
{
  if (stats_done || pose_done || map_done)
  {
    exit(0);
  }

  if (stats_done || pose_done || map_done)
  {
    stats_file.close();
    stats_done = true;
    return;
  }

  if (start.is_zero())
  {
    start = ros::Time(stats->header.stamp);
  }

  if ((ros::Time::now() - start).toSec() > time_limit)
  {
    stats_file.close();
    stats_done = true;
    return;
  }

  if (stats_file.is_open())
  {
    stats_file << (stats->header.stamp - start).toSec() << ", ";
    stats_file << stats->ltl_min_distance << ", ";
    stats_file << stats->ltl_max_distance << ", ";
    stats_file << stats->current_closest_distance << ", ";
    stats_file << stats->mean_closest_distance << ", ";
    stats_file << stats->closest_router_distance << ", ";
    stats_file << stats->ltl_min_altitude << ", ";
    stats_file << stats->ltl_max_altitude << ", ";
    stats_file << stats->current_closest_altitude << ", ";
    stats_file << stats->mean_closest_altitude << "\n";
    stats_file.flush();
  }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  if (stats_done || pose_done || map_done)
  {
    exit(0);
  }

  if (stats_done || pose_done || map_done)
  {
    pose_file.close();
    pose_done = true;
    return;
  }

  if (start.is_zero())
  {
    start = ros::Time(pose->header.stamp);
  }

  if ((ros::Time::now() - start).toSec() > time_limit)
  {
    pose_file.close();
    pose_done = true;
    return;
  }

  if (pose_file.is_open())
  {
    pose_file << (pose->header.stamp - start).toSec() << ", ";
    pose_file << pose->pose.position.x << ", ";
    pose_file << pose->pose.position.y << ", ";
    pose_file << pose->pose.position.z << ", ";
    pose_file << tf2::getYaw(pose->pose.orientation) << "\n";
    pose_file.flush();
  }
}

void mapCallback(const octomap_msgs::Octomap::ConstPtr& map)
{
  if (stats_done || pose_done || map_done)
  {
    exit(0);
  }

  if (stats_done || pose_done || map_done)
  {
    map_file.close();
    map_done = true;
    return;
  }

  if (start.is_zero())
  {
    start = ros::Time(map->header.stamp);
  }

  if ((ros::Time::now() - start).toSec() > time_limit)
  {
    map_file.close();
    map_done = true;
    return;
  }

  if (map_file.is_open())
  {
    octomap::OcTree* ot = (octomap::OcTree*)octomap_msgs::msgToMap(*map);

    double res = ot->getResolution();
    double cell_size = std::pow(res, 3.0);

    // ot->expand();

    // double current_volume = 0;
    // for (auto it = ot->begin_leafs_bbx(bbx_min, bbx_max), it_end = ot->end_leafs_bbx();
    // it != it_end; ++it)
    // {
    //   current_volume += cell_size;
    // }

    double volume_unmapped = 0;
    double volume_occupied = 0;
    double volume_free = 0;
    double current_volume = 0;
    for (double x = bbx_min.x(); x < bbx_max.x() - res / 2; x += res)
    {
      for (double y = bbx_min.y(); y < bbx_max.y() - res / 2; y += res)
      {
        for (double z = bbx_min.z(); z < bbx_max.z() - res / 2; z += res)
        {
          octomap::OcTreeNode* result = ot->search(x + res / 2, y + res / 2, z + res / 2);
          if (!result)
          {
            volume_unmapped += cell_size;
          }
          else if (result->getLogOdds() > 0)
          {
            current_volume += cell_size;
          }
          else
          {
            current_volume += cell_size;
          }
        }
      }
    }

    delete ot;

    map_file << (map->header.stamp - start).toSec() << ", ";
    map_file << total_volume << ", ";
    map_file << current_volume << ", ";
    map_file << (current_volume / total_volume) << "\n";
    map_file.flush();

    stl_aeplanner_msgs::Volume msg;
    msg.header.stamp = map->header.stamp;
    msg.current_volume = current_volume * volume_scaler / total_volume;
    volume_pub.publish(msg);

    if (msg.current_volume >= volume_complete)
    {
      map_done = true;
      map_file.close();
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stl_evaluation");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  std::string name = "";
  if (!nh_priv.getParam("name", name))
  {
    ROS_WARN("No name value specified");
  }

  std::string package_path = ros::package::getPath("stl_evaluation");

  int postfix = 0;
  while (boost::filesystem::exists(package_path + "/data/" + name + "_" + std::to_string(postfix)))
  {
    ++postfix;
  }

  boost::filesystem::path p(package_path + "/data/" + name + "_" + std::to_string(postfix));
  boost::filesystem::create_directories(p);

  std::vector<double> boundary_min;
  std::vector<double> boundary_max;
  if (!nh_priv.getParam("boundary/min", boundary_min))
  {
    ROS_WARN("No boundary/min value specified");
  }
  if (!nh_priv.getParam("boundary/max", boundary_max))
  {
    ROS_WARN("No boundary/max value specified");
  }

  stats_done = false;
  pose_done = false;
  map_done = false;
  volume_complete = nh_priv.param<double>("volume_complete", 0.0);
  time_limit = nh_priv.param<double>("time_limit", 0.0);

  std::ofstream info_file;
  info_file.open(package_path + "/data/" + name + "_" + std::to_string(postfix) + "/" + "info.txt");
  info_file << "name: " << nh_priv.param<std::string>("name", "") << "\n";
  info_file << "volume_complete: " << volume_complete << "\n";
  info_file << "time_limit: " << time_limit << "\n";
  info_file << "\n";
  info_file << "camera/horizontal_fov: " << nh_priv.param<double>("camera/horizontal_fov", 0.0) << "\n";
  info_file << "camera/vertical_fov: " << nh_priv.param<double>("camera/vertical_fov", 0.0) << "\n";
  info_file << "\n";
  info_file << "raycast/dr: " << nh_priv.param<double>("raycast/dr", 0.0) << "\n";
  info_file << "raycast/dphi: " << nh_priv.param<double>("raycast/dphi", 0.0) << "\n";
  info_file << "raycast/dtheta: " << nh_priv.param<double>("raycast/dtheta", 0.0) << "\n";
  info_file << "\n";
  info_file << "system/bbx/r: " << nh_priv.param<double>("system/bbx/r", 0.0) << "\n";
  info_file << "system/bbx/overshoot: " << nh_priv.param<double>("system/bbx/overshoot", 0.0) << "\n";
  info_file << "\n";
  info_file << "aep/gain/r_min: " << nh_priv.param<double>("aep/gain/r_min", 0.0) << "\n";
  info_file << "aep/gain/r_max: " << nh_priv.param<double>("aep/gain/r_max", 0.0) << "\n";
  info_file << "aep/gain/zero: " << nh_priv.param<double>("aep/gain/zero", 0.0) << "\n";
  info_file << "aep/gain/lambda: " << nh_priv.param<double>("aep/gain/lambda", 0.0) << "\n";
  info_file << "aep/gain/sigma_thresh: " << nh_priv.param<double>("aep/gain/sigma_thresh", 0.0) << "\n";
  info_file << "aep/tree/extension_range: " << nh_priv.param<double>("aep/tree/extension_range", 0.0) << "\n";
  info_file << "aep/tree/max_sampling_radius: " << nh_priv.param<double>("aep/tree/max_sampling_radius", 0.0) << "\n";
  info_file << "aep/tree/initial_iterations: " << nh_priv.param<double>("aep/tree/initial_iterations", 0.0) << "\n";
  info_file << "aep/tree/cutoff_iterations: " << nh_priv.param<double>("aep/tree/cutoff_iterations", 0.0) << "\n";
  info_file << "\n";
  info_file << "rrt/min_nodes: " << nh_priv.param<double>("rrt/min_nodes", 0.0) << "\n";
  info_file << "rrt/max_nodes: " << nh_priv.param<double>("rrt/max_nodes", 0.0) << "\n";
  info_file << "\n";
  info_file << "visualize_rays: " << nh_priv.param<bool>("visualize_rays", false) << "\n";
  info_file << "visualize_tree: " << nh_priv.param<bool>("visualize_tree", false) << "\n";
  info_file << "visualize_exploration_area: " << nh_priv.param<bool>("visualize_exploration_area", false) << "\n";
  info_file << "\n";
  info_file << "robot_frame: " << nh_priv.param<std::string>("robot_frame", "") << "\n";
  info_file << "world_frame: " << nh_priv.param<std::string>("world_frame", "") << "\n";
  info_file << "\n";
  info_file << "boundary/min: [ " << boundary_min[0] << ", " << boundary_min[1] << ", " << boundary_min[2] << " ]\n";
  info_file << "boundary/max: [ " << boundary_max[0] << ", " << boundary_max[1] << ", " << boundary_max[2] << " ]\n";
  info_file << "\n";
  info_file << "# STL STUFF BELOW\n";
  info_file << "lambda: " << nh_priv.param<double>("lambda", 0.0) << "\n";
  info_file << "min_distance: " << nh_priv.param<double>("min_distance", 0.0) << "\n";
  info_file << "max_distance: " << nh_priv.param<double>("max_distance", 0.0) << "\n";
  info_file << "min_distance_active: " << nh_priv.param<bool>("min_distance_active", false) << "\n";
  info_file << "max_distance_active: " << nh_priv.param<bool>("max_distance_active", false) << "\n";
  info_file << "routers_active: " << nh_priv.param<bool>("routers_active", false) << "\n";
  info_file << "distance_add_path: " << nh_priv.param<double>("distance_add_path", 0.0) << "\n";
  info_file << "max_search_distance: " << nh_priv.param<double>("max_search_distance", 0.0) << "\n";
  info_file << "step_size: " << nh_priv.param<double>("step_size", 0.0) << "\n";
  info_file << "min_altitude: " << nh_priv.param<double>("min_altitude", 0.0) << "\n";
  info_file << "max_altitude: " << nh_priv.param<double>("max_altitude", 0.0) << "\n";
  info_file << "min_altitude_active: " << nh_priv.param<bool>("min_altitude_active", false) << "\n";
  info_file << "max_altitude_active: " << nh_priv.param<bool>("max_altitude_active", false) << "\n";
  info_file.close();

  stats_file.open(package_path + "/data/" + name + "_" + std::to_string(postfix) + "/" + "stats.txt");
  stats_file << "Stamp, Min, Max, Current closest distance, Mean closest distance, "
                "Current closest router distance, Min altitude, Max altitude, Current closest altitude, Mean closest "
                "altitude\n";

  pose_file.open(package_path + "/data/" + name + "_" + std::to_string(postfix) + "/" + "pose.txt");
  pose_file << "Stamp, X, Y, Z, Yaw\n";

  map_file.open(package_path + "/data/" + name + "_" + std::to_string(postfix) + "/" + "map.txt");
  map_file << "Stamp, Max volume, Current volume, Procentage\n";

  bbx_min.x() = boundary_min[0];
  bbx_min.y() = boundary_min[1];
  bbx_min.z() = boundary_min[2];

  bbx_max.x() = boundary_max[0];
  bbx_max.y() = boundary_max[1];
  bbx_max.z() = boundary_max[2];

  volume_scaler = nh_priv.param("volume_scaler", 1.0);

  octomap::point3d temp = bbx_max - bbx_min;
  total_volume = temp.x() * temp.y() * temp.z();

  start = ros::Time(0);

  ros::Subscriber ltl_sub = nh.subscribe("/stl_aeplanner/ltl_stats", 1, &LTLStatsCallback);
  ros::Subscriber pose_sub = nh.subscribe("/mavros/local_position/pose", 1, &poseCallback);
  ros::Subscriber map_sub = nh.subscribe("/stl_aeplanner/octomap_full", 1, &mapCallback);

  volume_pub = nh.advertise<stl_aeplanner_msgs::Volume>("/stl_evaluation/volume", 1);

  ros::spin();

  stats_file.close();
  pose_file.close();
  map_file.close();

  return 0;
}