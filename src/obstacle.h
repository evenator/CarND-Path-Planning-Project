#ifndef OBSTACLE_H_
#define OBSTACLE_H_
#include "Eigen-3.3/Eigen/Dense"
#include "geometry.h"
#include "json.hpp"
#include <cmath>
#include <limits>
#include <vector>

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

  uint32_t get_id() const { return id_; }

  bool in_collision(Eigen::Vector2d const& other_xy,
                    double radius,
                    double time=0) const {
    return distance(other_xy, time) < radius;
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
    if (obstacle_frenet[0] > vehicle_frenet[0] && obstacle_frenet[0] < best_s) {
      leader = obstacle;
      best_s = obstacle_frenet[0];
    }
  }
  return leader;
}


bool check_collisions(Eigen::Vector2d xy,
                      std::vector<Obstacle> const &obstacles,
                      double collision_radius,
                      double time) {
  bool in_collision = false;
  for (auto const& obstacle : obstacles) {
    std::cout << "(" << xy[0] << ", " << xy[1] << ")@ " << time << " Obstacle " << obstacle.get_id() << "@(" << obstacle.xy(time)[0] << ", " << obstacle.xy(time)[1] << ") range=" << obstacle.distance(xy, time) << std::endl;
    if (obstacle.in_collision(xy, collision_radius, time)) {
      std::cout << "In collision with obstacle " << obstacle.get_id() << " at (" << xy[0] << ", " << xy[1] << ")" << std::endl;
      in_collision = true;
    }
  }
  return in_collision;  // No collisions
}
#endif // MAP_H_
