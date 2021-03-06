#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include "Eigen-3.3/Eigen/Dense"
#include <cmath>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double bearing(double x_from, double y_from, double x_to, double y_to) {
  return atan2(y_to - y_from, x_to - x_from);
}

Eigen::Vector2d transform_to_local(Eigen::Vector3d const &local_origin,
                                   Eigen::Vector2d const &point) {
  Eigen::Vector2d translation = point - local_origin.head(2);
  double c_theta = cos(local_origin[2]);
  double s_theta = sin(local_origin[2]);
  Eigen::Matrix2d h;
  h << c_theta, s_theta, -s_theta, c_theta;
  return h * translation;
}

Eigen::Vector2d transform_to_global(Eigen::Vector3d const &local_origin,
                                    Eigen::Vector2d const &point) {
  double c_theta = cos(local_origin[2]);
  double s_theta = sin(local_origin[2]);
  Eigen::Matrix2d h;
  h << c_theta, -s_theta, s_theta, c_theta;
  return local_origin.head(2) + h * point;
}

Eigen::Vector3d transform_pose_to_global(Eigen::Vector3d const &local_origin,
                                    Eigen::Vector3d const &pose) {
  double c_theta = cos(local_origin[2]);
  double s_theta = sin(local_origin[2]);
  Eigen::Matrix3d h;
  h << c_theta, -s_theta,  0,
       s_theta,  c_theta,  0,
             0,        0, -1;
  Eigen::Vector3d pose_out = local_origin + h * pose;
  pose_out[2] = atan2(sin(pose_out[2]), cos(pose_out[2]));
  return pose_out;
}


/**
 * Return the d value of the center of a lane
 */
inline double lane_center(int lane) { return lane * 4 + 2; }

/**
 * Return true if an object is fully or partially in a lane
 *
 * Assumes that the object is 3 m wide (wider than a car would be)
 */
bool in_lane(int lane, double d) {
  double center = lane_center(lane);
  static const double margin = 2.0;
  return (d > center - margin) && (d < center + margin);
}

#endif // GEOMETRY_H_
