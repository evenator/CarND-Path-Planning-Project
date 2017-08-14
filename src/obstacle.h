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

  Eigen::Vector2d sd() const { return frenet_; }

  Eigen::Vector2d velocity() const { return v_xy_; }

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
#endif // MAP_H_
