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
    for (double offset = 30; offset <= 100; offset += 20) {
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
      cout << "Distance to obstacle " << next_obstacle->get_id() << " is " << obstacle_dist << "m (" << v_obs
           << " m/s) frenet=(" << next_obstacle->sd()[0] << ", " << next_obstacle->sd()[1] << ")" << std::endl;
    }

    // Pull points from the spline at given intervals
    // This state is in the ego frame
    Eigen::Vector4d state(0, 0, 0, start_state[3]);
    size_t added = 0;
    while (path.size() < PATH_SIZE_ && obstacle_dist > 1.0) {
      // d is  the total distance to travel in this step, based on current
      // velocity
      double d = state[3] * T_STEP_;
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
      if (state[3] < v_des) {
        a = MAX_A_;
      }
      if (state[3] > v_des) {
        a = -MAX_A_;
      }
      // State update equation:
      Eigen::Vector2d previous_xy = state.head(2);
      state[0] += dx;
      state[1] = spline(state[0]);
      state[2] = bearing(previous_xy[0], previous_xy[1], state[0], state[1]);
      state[3] += a * T_STEP_;
      // Convert point from ego frame to global frame and push onto the path
      Eigen::Vector3d xy_theta = transform_pose_to_global(start_state.head(3), state.head(3));
      Eigen::Vector2d sd = map_->get_frenet(xy_theta);
      double time = path.size() * T_STEP_;
      const double collision_radius = 2.0;
      // TODO: This is wildly inefficient
      const Eigen::Vector2d bounding_box_sd(3, 2);
      if (false) {// (check_collisions(sd, obstacle_list, bounding_box_sd, time)) {
        // Path is in collision. Plan failed
        path.set_valid(false);
        return path;
      }
      path.push_back(xy_theta[0], xy_theta[1]);
      added++;
    }
    std::cout << "Added " << added << " points" << std::endl;
    path.set_valid(true);
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
  const double MAX_A_ = 7.5;    // Max acceleration/decel, m/s^2
  Map *map_;
};

class FSMPlanner {
public:
  FSMPlanner(Map *map) : map_(map), lane_(1), changing_lane_(false) {}

  Path plan(Eigen::Vector4d const &car_state_xy,
            std::vector<Obstacle> const &obstacles,
            Path const &previous_path = Path()) {
    double t_predict = 0.0;
    // Update the state of changinge_lane_
    auto car_sd = map_->get_frenet(car_state_xy.head(3));
    if (changing_lane_) {
      // If currently changing lanes, find out how far the car is from the destination
      // lane. If it's within 0.5 meters, the lane change is complete.
      double d = car_sd[1];
      if (abs(d - lane_center(lane_)) < 0.5) {
        changing_lane_ = false;
      }
    }

    // Truncate previous plan to 10 points to allow more flexibility
    const size_t trunc_size = 10;
    Path truncated_previous;
    for (size_t i = 0; i < trunc_size && i < previous_path.size(); ++i) {
      truncated_previous.push_back(previous_path[i]);
    }

    // Find distance to leader vehicle in each lane
    // TODO: There's some efficiency to be gained here
    std::vector<double> leader_distances;
    std::vector<int> leader_ids;
    auto lane_sd = car_sd;
    for (size_t lane = 0; lane < 3; lane++) {
      lane_sd[1] = lane_center(lane);
      auto leader = find_immediate_leader(lane_sd, obstacles);
      if (leader == obstacles.cend()) {
        leader_distances.push_back(map_->get_max_s());
        leader_ids.push_back(-1);
      }
      else {
        double distance_to_leader = Map::s_dist(leader->sd(t_predict)[0], car_sd[0]);
        leader_distances.push_back(distance_to_leader);
        leader_ids.push_back(leader->get_id());
      }
    }

    // Choose the best lane
    // This naive approach just chooses the lane with the most space until the next car
    size_t best_lane = 0;
    double best_dist = 0;
    for (size_t lane = 0; lane < 3; lane++) {
      if (leader_distances[lane] > best_dist) {
        best_dist = leader_distances[lane];
        best_lane = lane;
      }
    }

    // Print lane info
    for (size_t i = 0; i < 3; ++i) {
      std::cout << "| ";
      if (best_lane == i) {
        std::cout << "**";
      }
      else {
        std::cout << "  ";
      }
      if (lane_ == i) {
        std::cout << "(" << i << ")";
      }
      else {
        std::cout << " " << i << " ";
      }
      if (best_lane == i) {
        std::cout << "** ";
      }
      else {
        std::cout << "   ";
      }
    }
    std::cout << "|" << std::endl
              << "|---------|---------|---------|" << std::endl;
    for (size_t i = 0; i < 3; ++i) {
      std::cout << "|" << leader_ids[i] << "@" << leader_distances[i];
    }
    std::cout << "|" << std::endl;

    // Only bother changing lanes if there's less than 50 m of space and the best lane has at least
    // 10 m more space than this lane
    // TODO: Reenable lane changing
    bool should_change_lane = (leader_distances[lane_] < 50.0)
                              && (best_dist - leader_distances[lane_] > 10.0);

    if (!changing_lane_ && should_change_lane) {
      size_t next_lane = lane_;
      if (best_lane < lane_) {
        // Try to change lanes to the left
        next_lane--;
      }
      else if (best_lane > lane_) {
        // Try to change lanes to the right
        next_lane++;
      }
      double next_lane_center = lane_center(next_lane);
      bool can_change_lane = !check_for_obstacles_frenet(car_sd[0]-50.0, car_sd[0]+25.0,
                                                         next_lane_center - 2.0, next_lane_center + 2.0,
                                                         obstacles,
                                                         t_predict);
      if (can_change_lane) {
        Path plan;
        auto planner = LaneKeepPlanner(map_, next_lane);
        if (previous_path.size() > 0) {
          // TODO: Check for collisions
          // TODO: Limit lateral acceleration during lane changes
          plan = planner.plan(truncated_previous, obstacles);
        } else {
          plan = planner.plan(car_state_xy, obstacles);
        }

        if (plan.is_valid()) {
            changing_lane_ = true;
            lane_ = next_lane;
            std::cout << "Changing lanes" << std::endl;
            return plan;
        }
        else {
          std::cout << "Lane change plan is invalid" << std::endl;
        }
      }
      else {
        std::cout << "Can't change lane due to obstacles" << std::endl;
      }
    }

    // Stay in the current lane
    auto planner = LaneKeepPlanner(map_, lane_);
    Path plan;
    if (previous_path.size() > 0) {
      plan = planner.plan(truncated_previous, obstacles);
    }
    else {
      plan = planner.plan(car_state_xy, obstacles);
    }
    if (plan.is_valid()) {
          return plan;
    }
    else {
        std::cout << "!!!!!!!!!!!!!  Lane keep plan is invalid" << std::endl;
        return plan;
    }
  }

private:
  Map *map_;
  int lane_;
  bool changing_lane_;
};

#endif // PLANNER_H_
