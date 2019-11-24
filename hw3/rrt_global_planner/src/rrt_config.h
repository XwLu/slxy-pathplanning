//
// Created by luyifan on 19-11-23.
//

#ifndef RRT_GLOBAL_PLANNER_VALIDITY_CHECK_H
#define RRT_GLOBAL_PLANNER_VALIDITY_CHECK_H

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Our collision checker. For this demo, our robot's state space
class ValidityChecker : public ob::StateValidityChecker
{
public:
  ValidityChecker(const ob::SpaceInformationPtr& si, costmap_2d::Costmap2D* costmap) :
      ob::StateValidityChecker(si), costmap_(costmap) {}
  // Returns whether the given state's position overlaps the
  // circular obstacle
  bool isValid(const ob::State* state) const
  {
    // We know we're working with a RealVectorStateSpace in this
    // example, so we downcast state into the specific type.
    const ob::RealVectorStateSpace::StateType* state3D =
        state->as<ob::RealVectorStateSpace::StateType>();
    //Extract the robot's (x,y,z) position from its state
    double wx = state3D->values[0];
    double wy = state3D->values[1];

    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    unsigned int mx = (unsigned int)((wx - origin_x) / resolution);
    unsigned int my = (unsigned int)((wy - origin_y) / resolution);

    if(mx < 0 || my <0 || mx >= costmap_->getSizeInCellsX() || my >= costmap_->getSizeInCellsY())
      return false;

    unsigned char cost = costmap_->getCost(mx, my);
    if(cost != costmap_2d::FREE_SPACE && cost != costmap_2d::NO_INFORMATION){
      //ROS_INFO("x: %f, y: %f, cost: %d", wx, wy, (int)cost);
      return false;
    }
    return true;
  }

private:
  costmap_2d::Costmap2D* costmap_;
};

inline ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
  return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

inline ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(1.51));
  return obj;
}

#endif //RRT_GLOBAL_PLANNER_VALIDITY_CHECK_H
