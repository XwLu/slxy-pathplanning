//
// Created by luyifan on 19-11-23.
//

#ifndef RRT_GLOBAL_PLANNER_RRT_GLOBAL_PLANNER_H
#define RRT_GLOBAL_PLANNER_RRT_GLOBAL_PLANNER_H

#include <string>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include "tf/transform_datatypes.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

#include "rrt_config.h"

using std::string;

namespace global_planner {

  class RRTGlobalPlanner : public nav_core::BaseGlobalPlanner {
  public:

    RRTGlobalPlanner();
    RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);
    void pubMap(costmap_2d::Costmap2D* costmap);

  private:
    bool initialized_ = false;
    costmap_2d::Costmap2D* costmap_;
    ros::Publisher pub_cost_map_;
    ros::Publisher pub_rrt_path_;
  };
};

#endif //RRT_GLOBAL_PLANNER_RRT_GLOBAL_PLANNER_H
