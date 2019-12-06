//
// Created by luyifan on 19-12-6.
//
#include "optimal_time_solver.h"
#include "ros/ros.h"
using namespace std;
// 代价函数的计算模型
struct MINI_JERK_COST {
  MINI_JERK_COST(Eigen::MatrixXd start, Eigen::MatrixXd goal) : _start(start), _goal(goal) {}

  // 残差的计算
  template<typename T>
  bool operator()(
      const T *const t, // 模型参数，有1维
      T *residual) const {

    const T t2 = t[0]*t[0];
    const T t3 = t2*t[0];

    const T delta_px = T(_goal(0) - _start(3)*t[0] - _start(0));
    const T delta_py = T(_goal(1) - _start(4)*t[0] - _start(1));
    const T delta_pz = T(_goal(2) - _start(5)*t[0] - _start(2));
    const T delta_vx = T(_goal(3) - _start(3));
    const T delta_vy = T(_goal(4) - _start(4));
    const T delta_vz = T(_goal(5) - _start(5));

    const T a1 = T(-12.0) / t3 * delta_px + T(6.0) / t2 * delta_vx;
    const T a2 = T(-12.0) / t3 * delta_py + T(6.0) / t2 * delta_vy;
    const T a3 = T(-12.0) / t3 * delta_pz + T(6.0) / t2 * delta_vz;
    const T b1 = T(6.0) / t2 * delta_px + T(-2.0) / t[0] * delta_vx;
    const T b2 = T(6.0) / t2 * delta_py + T(-2.0) / t[0] * delta_vy;
    const T b3 = T(6.0) / t2 * delta_pz + T(-2.0) / t[0] * delta_vz;

    const T J = t[0] + T(1.0/3)*a1*a1*t3 + a1*b1*t2 + b1*b1*t[0] +
                       T(1.0/3)*a2*a2*t3 + a2*b2*t2 + b2*b2*t[0] +
                       T(1.0/3)*a3*a3*t3 + a3*b3*t2 + b3*b3*t[0];

    residual[0] = T(0.0) - J; //
    return true;
  }

  const Eigen::MatrixXd _start, _goal;    //lattice起点终点
};


double GetOptimalTime(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d target_pt){
  Eigen::MatrixXd start(6, 1);
  start << start_pt(0), start_pt(1), start_pt(2), start_vel(0), start_vel(1), start_vel(2);
  Eigen::MatrixXd goal(6, 1);
  goal << target_pt(0), target_pt(1), target_pt(2), 0, 0, 0;
  //构建优化问题
  double optimal_time[1] = {1000.0};
  ceres::Problem problem;
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<MINI_JERK_COST, 1, 1>(
          new MINI_JERK_COST(start, goal)
          ),
          nullptr,
          optimal_time
      );
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  //cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
  ROS_INFO("Optimal time = %f s. Solve time cost = %f seconds.", optimal_time[0], time_used.count());
  return optimal_time[0];
}