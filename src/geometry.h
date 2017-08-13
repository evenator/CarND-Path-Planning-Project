#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <cmath>
#include "Eigen-3.3/Eigen/Dense"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

Eigen::Vector2d transform_to_local(Eigen::Vector3d const& local_origin,
                                   Eigen::Vector2d const& point) {
  Eigen::Vector2d translation = point - local_origin.head(2);
  double c_theta = cos(local_origin[2]);
  double s_theta = sin(local_origin[2]);
  Eigen::Matrix2d h;
  h << c_theta, s_theta, -s_theta, c_theta;
  return h * translation;
}

Eigen::Vector2d transform_to_global(Eigen::Vector3d const& local_origin,
                                   Eigen::Vector2d const& point) {
  double c_theta = cos(local_origin[2]);
  double s_theta = sin(local_origin[2]);
  Eigen::Matrix2d h;
  h << c_theta, -s_theta, s_theta, c_theta;
  return local_origin.head(2) + h * point;
}
#endif  // GEOMETRY_H_
