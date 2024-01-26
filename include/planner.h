#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//services
#include <hybrid_astar/planReqSrv.h>
#include <hybrid_astar/cubeReqSrv.h>
#include <std_srvs/Trigger.h>
#include <vector>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"
#include <cube_block.h>

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     \todo probably removed
  */
  void initializeLookups();

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);

  /*!
     \brief setStart
     \param start the start pose
  */
  void setStart_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start, bool run_plan = true);

  /*!
     \brief setGoal
     \param goal the goal pose
  */
  void setGoal_cb(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal, bool run_plan = true);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
  size_t plan();

  /// The plan request handler
  bool plan_req_handler(hybrid_astar::planReqSrvRequest &req, hybrid_astar::planReqSrvResponse &res);
  /// The cube request handler
  bool cube_req_handler(hybrid_astar::cubeReqSrvRequest &req, hybrid_astar::cubeReqSrvResponse &res);
  /// The cube cost request handler
  bool push_cost_req_handler(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

 private:
  /// The node handle
  ros::NodeHandle n;
  /// A publisher publishing the start position for RViz
  ros::Publisher pubStart;
  /// A subscriber for receiving map updates
  ros::Subscriber subMap;
  /// A subscriber for receiving goal updates
  ros::Subscriber subGoal;
  /// A subscriber for receiving start updates
  ros::Subscriber subStart;
  /// A listener that awaits transforms
  tf::TransformListener listener;
  /// A transform for moving start positions
  tf::StampedTransform transform;

  /// A plan request service
  ros::ServiceServer srvPlanReqService;
  /// A service for adding a cube
  ros::ServiceServer srvCubeReqService;
  /// The cube push cost est. req
  ros::ServiceServer srvPushCostReqService;

  /// The path produced by the hybrid A* algorithm
  Path path;
  /// The smoother used for optimizing the path
  Smoother smoother;
  /// The path smoothed and ready for the controller
  Path smoothedPath = Path(true);
  /// The visualization used for search visualization
  Visualize visualization;
  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;
  /// The voronoi diagram
  DynamicVoronoi voronoiDiagram;
  /// A pointer to the grid the planner runs on
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The start pose set through RViz
  geometry_msgs::PoseWithCovarianceStamped start;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;
  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  Constants::config collisionLookup[Constants::headings * Constants::positions];
  /// A lookup of analytical solutions (Dubin's paths)
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];

  /// A list of cube blocks
  std::vector<jeeho::cube> cube_list = std::vector<jeeho::cube>();

};
}
#endif // PLANNER_H
