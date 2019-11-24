//
// Created by luyifan on 19-11-23.
//

#include <pluginlib/class_list_macros.h>
#include "rrt_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::RRTGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

  RRTGlobalPlanner::RRTGlobalPlanner (){

  }

  RRTGlobalPlanner::RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
  }


  void RRTGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ = costmap_ros->getCostmap(); //get the costmap_ from costmap_ros_
      // initialize other planner parameters
      ros::NodeHandle private_nh("~/" + name);
      pub_cost_map_ = private_nh.advertise<nav_msgs::OccupancyGrid>("cost_map", 1);
      pub_rrt_path_ = private_nh.advertise<nav_msgs::Path>("rrt_path", 1);
//      private_nh.param("step_size", step_size_, costmap_->getResolution());
//      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
//      world_model_ = new base_local_planner::CostmapModel(*costmap_);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool RRTGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& plan ){
    plan.clear();
    // Construct the robot state space in which we're planning.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));
    // Set the bounds of space to be in [0,1].
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, -15);
    bounds.setLow(1, -15);
    bounds.setLow(2, 0);

    bounds.setHigh(0, 15);
    bounds.setHigh(1, 15);
    bounds.setHigh(2, 1);

    unsigned int mx, my;
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
    pubMap(costmap_);
    ROS_INFO("start x: %f, y: %f, goal x: %f, y: %f.",
        start.pose.position.x, start.pose.position.y,
        goal.pose.position.x, goal.pose.position.y);
    ROS_INFO("map low x: %f, y: %f, high x: %f, y: %f",
        costmap_->getOriginX(), costmap_->getOriginY(),
        costmap_->getSizeInMetersX() + costmap_->getOriginX(), costmap_->getSizeInMetersY() + costmap_->getOriginY());

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si, costmap_)));
    si->setup();
    // Set our robot's starting state
    ob::ScopedState<> start_pose(space);
    start_pose[0] = start.pose.position.x;
    start_pose[1] = start.pose.position.y;
    start_pose[2] = 0;
    ob::ScopedState<> goal_pose(space);
    goal_pose[0] = goal.pose.position.x;
    goal_pose[1] = goal.pose.position.y;
    goal_pose[2] = 0;
    // Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    // Set the start and goal states
    pdef->setStartAndGoalStates(start_pose, goal_pose);
    // Set the optimization objective
    pdef->setOptimizationObjective(getPathLengthObjective(si));
    // Construct our optimizing planner using the RRTstar algorithm
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));
    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();
    // attempt to solve the planning problem within one second of

    ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

    if(solved){
      // get the goal representation from the problem definition (not the same as the goal state)
      // and inquire about the found path
      og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();

      double theta = 0.0;
      for(size_t id = 0; id < path->getStateCount(); id++){
        const ob::RealVectorStateSpace::StateType *state = path->getState(id)->as<ob::RealVectorStateSpace::StateType>();
        geometry_msgs::PoseStamped tmp;
        tmp.header.frame_id = "map";
        tmp.pose.position.x = state->values[0];
        tmp.pose.position.y = state->values[1];
        tmp.pose.position.z = 0.0;

        if(id < path->getStateCount() - 1){
          const ob::RealVectorStateSpace::StateType *next_state = path->getState(id+1)->as<ob::RealVectorStateSpace::StateType>();
          theta = atan2(next_state->values[1] - state->values[1], next_state->values[0] - state->values[0]);
        }
        tmp.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        plan.emplace_back(tmp);
      }

      nav_msgs::Path rrt_path;
      rrt_path.header.frame_id = "map";
      for(size_t id = 0; id < path->getStateCount(); id++){
        const ob::RealVectorStateSpace::StateType *state = path->getState(id)->as<ob::RealVectorStateSpace::StateType>();
        rrt_path.poses.emplace_back();
        rrt_path.poses.back().pose.position.x = state->values[0];
        rrt_path.poses.back().pose.position.y = state->values[1];
      }
      pub_rrt_path_.publish(rrt_path);

      ROS_INFO("Find a solution!!");
      return true;
    }
    ROS_INFO("Failed!!");
    return false;
  }

  void RRTGlobalPlanner::pubMap(costmap_2d::Costmap2D *costmap) {
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.origin.position.x = costmap->getOriginX();
    map.info.origin.position.y = costmap->getOriginY();
    map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
    map.info.width = costmap->getSizeInCellsX();
    map.info.height = costmap->getSizeInCellsY();
    map.info.resolution = costmap->getResolution();
    for(int x = 0; x < costmap->getSizeInCellsX(); x++){
      for(int y = 0; y < costmap->getSizeInCellsY(); y++) {
        map.data.emplace_back(costmap->getCost(y, x));
      }
    }
    pub_cost_map_.publish(map);
  }

};