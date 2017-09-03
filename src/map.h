#ifndef MAP_H_
#define MAP_H_
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "geometry.h"

class Map {
public:
  static Map from_file(std::string const &map_file) {
    Map map;

    std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

    std::string line;
    while (getline(in_map_, line)) {
      std::istringstream iss(line);
      double x;
      double y;
      float s;
      float d_x;
      float d_y;
      iss >> x;
      iss >> y;
      iss >> s;
      iss >> d_x;
      iss >> d_y;
      map.x_.push_back(x);
      map.y_.push_back(y);
      map.s_.push_back(s);
      map.dx_.push_back(d_x);
      map.dy_.push_back(d_y);
    }
    return map;
  }

  /**
   * Get the index of the waypoint closest to the given x-y coordinates
   */
  size_t closest_waypoint(double x, double y) const {
    double closestLen = 100000; // large number
    size_t closestWaypoint = 0;

    for (size_t i = 0; i < size(); i++) {
      double map_x = x_[i];
      double map_y = y_[i];
      double dist = distance(x, y, map_x, map_y);
      if (dist < closestLen) {
        closestLen = dist;
        closestWaypoint = i;
      }
    }

    return closestWaypoint;
  }

  /**
   * Get the index of the next waypoint in front of the x-y-theta coordinates
   */
  size_t next_waypoint(double x, double y, double theta) const {
    size_t closestWaypoint = closest_waypoint(x, y);

    double map_x = x_[closestWaypoint];
    double map_y = y_[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = abs(theta - heading);

    if (angle > pi() / 4) {
      closestWaypoint++;
    }

    return closestWaypoint;
  }

  /**
   * Transform from Cartesian x,y coordinates to Frenet s,d coordinates
   */
  Eigen::Vector2d get_frenet(Eigen::Vector3d xy_theta) {
    return get_frenet(xy_theta[0], xy_theta[1], xy_theta[2]);
  }

  Eigen::Vector2d get_frenet(double x, double y, double theta) const {
    int next_wp = next_waypoint(x, y, theta);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
      prev_wp = size() - 1;
    }

    double n_x = x_[next_wp] - x_[prev_wp];
    double n_y = y_[next_wp] - y_[prev_wp];
    double x_x = x - x_[prev_wp];
    double x_y = y - y_[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    // see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - x_[prev_wp];
    double center_y = 2000 - y_[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
      frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
      frenet_s += distance(x_[i], y_[i], x_[i + 1], y_[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return Eigen::Vector2d(frenet_s, frenet_d);
  }

  /**
   * Transform from Frenet s,d coordinates to Cartesian x,y
   */
  Eigen::Vector2d get_xy(Eigen::Vector2d const &frenet) const {
    return get_xy(frenet[0], frenet[1]);
  }

  Eigen::Vector2d get_xy(double s, double d) const {
    int prev_wp = -1;

    s = fmod(s, max_s_);

    while (s > s_[prev_wp + 1] && (prev_wp < (int)(size() - 1))) {
      prev_wp++;
    }

    int wp2 = (prev_wp + 1) % size();

    double heading = atan2((y_[wp2] - y_[prev_wp]), (x_[wp2] - x_[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - s_[prev_wp]);

    double seg_x = x_[prev_wp] + seg_s * cos(heading);
    double seg_y = y_[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return Eigen::Vector2d(x, y);
  }

  size_t size() const { return x_.size(); }

  /**
   * Subtract s2 from s1, with wraparound
   */
  static double s_dist(double s1, double s2) {
    double ds = s1 - s2;
    if (ds >= max_s_) {
      ds = fmod(ds, max_s_);
    }
    if (ds <= -max_s_) {
      ds = fmod(ds, max_s_) + max_s_;
    }
    return ds;
  }

  static double get_max_s() {
    return max_s_;
  }

  // The max s value before wrapping around the track back to 0
  static constexpr double max_s_ = 6945.554;

private:
  Map() {}

  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> s_;
  std::vector<double> dx_;
  std::vector<double> dy_;
};
#endif // MAP_H_
