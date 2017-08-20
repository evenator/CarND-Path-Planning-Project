#ifndef PLANNER_H_
#define PLANNER_H_
#include "Eigen-3.3/Eigen/Dense"
#include "geometry.h"
#include "map.h"
#include "obstacle.h"
#include "path.h"
#include "spline.h"
#include <vector>

class LaneKeepPlanner {
public:
  LaneKeepPlanner(Map *map, int lane)
      : v_des_(21.0), lane_des_(lane), map_(map), follow_dist_(20.0) {}

  /**
   * Plan a path from a start state, which is a 4-vector
   * of x, y, theta, speed
   */
  Path plan(Eigen::Vector4d const &start_state,
            std::vector<Obstacle> const &obstacle_list, Path path = Path()) {
    // Make spline
    // Note that all spline points are in the ego frame of the start_state
    std::vector<double> spline_x(1, -1);
    std::vector<double> spline_y(1, 0);
    spline_x.push_back(0);
    spline_y.push_back(0);
    Eigen::Vector2d start_sd = map_->get_frenet(start_state.head(3));
    for (double offset = 20; offset <= 80; offset += 20) {
      Eigen::Vector2d sd(start_sd[0] + offset, lane_center(lane_des_));
      Eigen::Vector2d global_xy = map_->get_xy(sd);
      Eigen::Vector2d xy = transform_to_local(start_state.head(3), global_xy);
      spline_x.push_back(xy[0]);
      spline_y.push_back(xy[1]);
    }
    std::cout << "Spline points: ";
    for (size_t i = 0; i < spline_x.size(); ++i) {
      std::cout << "(" << spline_x[i] << ", " << spline_y[i] << "), ";
    }
    std::cout << std::endl;
    tk::spline spline;
    spline.set_points(spline_x, spline_y);

    auto next_obstacle = find_immediate_leader(start_sd, obstacle_list);
    double obstacle_dist = map_->max_s_;
    double v_obs = v_des_;
    if (next_obstacle != obstacle_list.cend()) {
      obstacle_dist = fmod(next_obstacle->sd()[0] - start_sd[0], map_->max_s_);
      v_obs = next_obstacle->velocity().norm();
      cout << "Distance to next obstacle is " << obstacle_dist << "m (" << v_obs
           << " m/s)" << std::endl;
    }

    // Pull points from the spline at given intervals
    // We don't need theta anymore, and this state is in the ego frame
    Eigen::Vector3d state(0, 0, start_state[3]); // x, y, v
    size_t added = 0;
    while (path.size() < PATH_SIZE_) {
      // d is  the total distance to travel in this step, based on current
      // velocity
      double d = state[2] * T_STEP_;
      obstacle_dist -= d;
      // Calculate dx using current v and current tangent slope m
      double m = (spline(state[0] + 0.01) - state[1]) / (.01);
      double dx = d / sqrt(1 + m * m);
      // match the vehicle in front's speed when it is 30 m in front of us
      double v_des = (obstacle_dist - follow_dist_) * v_obs / 10.0 + v_obs;
      v_des = std::min(v_des_, v_des);
      // a is the desired acceleration
      double a = 0;
      // TODO: Don't use bang-bang acceleration
      if (state[2] < v_des) {
        a = MAX_A_;
      }
      if (state[2] > v_des) {
        a = -MAX_A_;
      }
      // State update equation:
      state[0] += dx;
      state[1] = spline(state[0]);
      state[2] += a * T_STEP_;
      // Convert point from ego frame to global frame and push onto the path
      Eigen::Vector2d xy =
          transform_to_global(start_state.head(3), state.head(2));
      path.push_back(xy[0], xy[1]);
      added++;
    }
    std::cout << "Added " << added << " points" << std::endl;
    return path;
  }

  /**
   * Plan a path that begins with a previous path
   */
  Path plan(Path const &path, std::vector<Obstacle> const &obstacle_list) {
    Eigen::Vector2d end_xy = path[path.size() - 1];
    Eigen::Vector2d penultimate_xy = path[path.size() - 2];
    Eigen::Vector2d d_xy = end_xy - penultimate_xy;
    double theta = atan2(d_xy[1], d_xy[0]);
    double v = d_xy.norm() / T_STEP_;
    auto end_sd = map_->get_frenet(end_xy[0], end_xy[1], theta);
    Eigen::Vector4d start_state(end_xy[0], end_xy[1], theta, v);
    return plan(start_state, obstacle_list, path);
  }

private:
  const size_t PATH_SIZE_ = 50; // Plan size, points
  const double T_STEP_ = 0.02;  // Time step, seconds
  double v_des_;                // Desired speed m/s
  int lane_des_;                // Desired lane (integer)
  double follow_dist_;          // Follow distance (m)
  const double MAX_A_ = 5.0;    // Max acceleration/decel, m/s^2
  Map *map_;
};

class FSMPlanner {
public:
  FSMPlanner(Map *map) : map_(map), lane_(1) {}

  Path plan(Eigen::Vector4d const &car_state_xy,
            std::vector<Obstacle> const &obstacles,
            Path const &previous_path = Path()) {
    auto planner = LaneKeepPlanner(map_, lane_);
    if (previous_path.size() > 0) {
      return planner.plan(previous_path, obstacles);
    } else {
      return planner.plan(car_state_xy, obstacles);
    }
  }

private:
  Map *map_;
  int lane_;
};

#endif // PLANNER_H_
