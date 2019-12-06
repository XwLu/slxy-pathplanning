#include <hw_tool.h>
#include "utlis.h"
#include "optimal_time_solver.h"
using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y, const double coord_z)
{
  if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
    return;

  int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);

  data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
  Vector3d pt;
  Vector3i idx;

  pt(0) = coord_x;
  pt(1) = coord_y;
  pt(2) = coord_z;
  idx = coord2gridIndex(pt);

  int idx_x = idx(0);
  int idx_y = idx(1);
  int idx_z = idx(2);

  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i & index)
{
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d & pt)
{
  Vector3i idx;
  idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d & coord)
{
  return gridIndex2coord(coord2gridIndex(coord));
}


double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position, Eigen::Vector3d _start_velocity,
    Eigen::Vector3d _target_position,
    std::vector<Eigen::Vector3d>& Position,
    bool& collision)
{
  double optimal_cost = 100000; // this just to initial the optimal_cost, you can delete it
  /*
  STEP 2: go to the hw_tool.cpp and finish the function Homeworktool::OptimalBVP
  the solving process has been given in the document
  because the final point of trajectory is the start point of OBVP, so we input the pos,vel to the OBVP
  after finish Homeworktool::OptimalBVP, the Trajctory_Cost will record the optimal cost of this trajectory
  */
  double T = GetOptimalTime(_start_position, _start_velocity, _target_position);//先给一个固定值
  if (T < 0){
    T = 10;
    collision = true;
  }
  double T2 = T * T;
  double T3 = T2 * T;
  Eigen::MatrixXd delta_state(6, 1);
  delta_state << _target_position[0] - _start_velocity(0)*T - _start_position(0),
      _target_position[1] - _start_velocity(1)*T - _start_position(1),
      _target_position[2] - _start_velocity(2)*T - _start_position(2),
      0 - _start_velocity(0),
      0 - _start_velocity(1),
      0 - _start_velocity(2);
  Eigen::MatrixXd transform(6, 6);
  transform << -12/T3, 0, 0, 6/T2, 0, 0,
               0, -12/T3, 0, 0, 6/T2, 0,
               0, 0, -12/T3, 0, 0, 6/T2,
               6/T2, 0, 0, -2/T, 0, 0,
               0, 6/T2, 0, 0, -2/T, 0,
               0, 0, 6/T2, 0, 0, -2/T;
  Eigen::MatrixXd coeffs(6, 1);
  coeffs = transform * delta_state;
  double t = 0;
  Vector3d pos, vel;
  while(t <= T){
    double t2 = t * t;
    double t3 = t2 * t;
    pos(0) = 1.0/6 * coeffs(0,0) * t3 + 1.0/2 * coeffs(3,0) * t2 + _start_velocity(0) * t + _start_position(0);
    pos(1) = 1.0/6 * coeffs(1,0) * t3 + 1.0/2 * coeffs(4,0) * t2 + _start_velocity(1) * t + _start_position(1);
    pos(2) = 1.0/6 * coeffs(2,0) * t3 + 1.0/2 * coeffs(5,0) * t2 + _start_velocity(2) * t + _start_position(2);
    vel(0) = 1.0/2 * coeffs(0,0) * t2 + coeffs(3,0) * t + _start_velocity(0);
    vel(1) = 1.0/2 * coeffs(1,0) * t2 + coeffs(4,0) * t + _start_velocity(1);
    vel(2) = 1.0/2 * coeffs(2,0) * t2 + coeffs(5,0) * t + _start_velocity(2);
    t += 0.1;
    Position.push_back(pos);
    double coord_x = pos(0);
    double coord_y = pos(1);
    double coord_z = pos(2);
    //check if if the trajectory face the obstacle
    if(isObsFree(coord_x,coord_y,coord_z) != 1){
      collision = true;
    }
  }

  optimal_cost = 1.0/3*coeffs(0)*coeffs(0)*T3 + coeffs(0)*coeffs(3)*T2 + coeffs(3)*coeffs(3)*T +
                 1.0/3*coeffs(1)*coeffs(1)*T3 + coeffs(1)*coeffs(4)*T2 + coeffs(4)*coeffs(4)*T +
                 1.0/3*coeffs(2)*coeffs(2)*T3 + coeffs(2)*coeffs(5)*T2 + coeffs(5)*coeffs(5)*T + T;

  return optimal_cost;
}