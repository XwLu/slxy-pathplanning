//
// Created by luyifan on 19-12-6.
//

#ifndef GRID_PATH_SEARCHER_OPTIMAL_TIME_SOLVER_H
#define GRID_PATH_SEARCHER_OPTIMAL_TIME_SOLVER_H

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include <chrono>

double GetOptimalTime(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d target_pt);

#endif //GRID_PATH_SEARCHER_OPTIMAL_TIME_SOLVER_H
