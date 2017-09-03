#ifndef OBSTACLE_H_
#define OBSTACLE_H_
#include "Eigen-3.3/Eigen/Dense"
#include "geometry.h"
#include "json.hpp"
#include <cmath>
#include <limits>
#include <vector>
#include "map.h"

class Obstacle {
public:
  Obstacle(const nlohmann::basic_json<> &json_params)
      : id_(json_params[0]), xy_(json_params[1], json_params[2]),
        v_xy_(json_params[3], json_params[4]),
        frenet_(json_params[5], json_params[6]) {}

  Obstacle(uint32_t id, double x, double y, double vx, double vy, double s, double d)
      : id_(id), xy_(x, y),
        v_xy_(vx, vy),
        frenet_(s, d) {}

  Eigen::Vector2d sd(double time=0) const {
    double v = v_xy_.norm();
    return frenet_ + Eigen::Vector2d(time * v, 0);
  }

  Eigen::Vector2d velocity() const { return v_xy_; }

  Eigen::Vector2d xy(double time=0) const {
    return xy_ + time * v_xy_;
  }

  double distance(Eigen::Vector2d const& other_xy, double time=0) const {
    return (xy(time) - other_xy).norm();
  }

  double frenet_distance(Eigen::Vector2d const& other_sd, double time=0) const {
    return (sd(time) - other_sd).norm();
  }

  uint32_t get_id() const { return id_; }

  bool in_collision(Eigen::Vector2d const& other_sd,
                    Eigen::Vector2d const& bounding_box_sd,
                    double time=0) const {
    Eigen::Vector2d dist_sd = sd(time) - other_sd;
    double s = abs(dist_sd[0]);
    double d = abs(dist_sd[1]);
    return s < bounding_box_sd[0] && d < bounding_box_sd[1];
  }

private:
  uint32_t id_;
  Eigen::Vector2d xy_;
  Eigen::Vector2d v_xy_;
  Eigen::Vector2d frenet_;
};

static std::vector<Obstacle>::const_iterator
find_immediate_leader(Eigen::Vector2d const &vehicle_frenet,
                      std::vector<Obstacle> const &obstacles) {
  int vehicle_lane = round((vehicle_frenet[1] - 2) / 4);
  double best_s = std::numeric_limits<double>::max();
  auto leader = obstacles.cend();
  for (auto obstacle = obstacles.cbegin(); obstacle != obstacles.cend();
       ++obstacle) {
    auto obstacle_frenet = obstacle->sd();
    // Filter obstacles to only include those in the vehicle's lane
    if (!in_lane(vehicle_lane, obstacle_frenet[1])) {
      continue;
    }
    // Find the first obstacle with s greater than vehicle
    double s_dist = Map::s_dist(obstacle_frenet[0], vehicle_frenet[0]);
    if (s_dist > 0 && s_dist < best_s) {
      leader = obstacle;
      best_s = s_dist;
    }
  }
  std::cout << "Leader is " << leader->get_id() << " @ " << best_s << std::endl;
  return leader;
}


bool check_collisions(Eigen::Vector2d sd,
                      std::vector<Obstacle> const &obstacles,
                      Eigen::Vector2d const& bounding_box_sd,
                      double time) {
  bool in_collision = false;
  for (auto const& obstacle : obstacles) {
    //std::cout << "(" << xy[0] << ", " << xy[1] << ")@ " << time << " Obstacle " << obstacle.get_id() << "@(" << obstacle.xy(time)[0] << ", " << obstacle.xy(time)[1] << ") range=" << obstacle.distance(xy, time) << std::endl;
    if (obstacle.in_collision(sd, bounding_box_sd, time)) {
      std::cout << "In collision with obstacle " << obstacle.get_id() << " at (" << sd[0] << ", " << sd[1] << ")" << std::endl;
      in_collision = true;
    }
  }
  return in_collision;  // No collisions
}

bool check_for_obstacles_frenet(double s_min, double s_max,
                                double d_min, double d_max,
                                std::vector<Obstacle> const &obstacles) {
  std::cout << "Checking for obstacles in range s(" << s_min <<", "<<s_max<<") d("<<d_min<<", "<<d_max<<")"<<std::endl;
  // For wraparound
  bool invert_s = (s_min > s_max);
  for (auto const& obstacle : obstacles) {
    Eigen::Vector2d sd = obstacle.sd();
    if (invert_s) {
      if ((sd[0] < s_min || sd[0] > s_max) && sd[1] > d_min && sd[1] < d_max) {
        std::cout << "Obstacle  " << obstacle.get_id() << " is in range. frenet=(" << obstacle.sd()[0] << ", " << obstacle.sd()[1] << ")" << std::endl;
        return true;
      }
    }
    else {
      if (sd[0] > s_min && sd[0] < s_max && sd[1] > d_min && sd[1] < d_max) {
        std::cout << "Obstacle  " << obstacle.get_id() << " is in range. frenet=(" << obstacle.sd()[0] << ", " << obstacle.sd()[1] << ")" << std::endl;
        return true;
      }
    }
  }
  std::cout << "Checked " << obstacles.size() << " obstacles. Found none in range" << std::endl;
  return false;  // No obstacles in region
}
#endif // MAP_H_
