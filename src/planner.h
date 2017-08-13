#ifndef PLANNER_H_
#define PLANNER_H_
#include "geometry.h"
#include "map.h"
#include "path.h"
#include "spline.h"
#include "Eigen-3.3/Eigen/Dense"

double lane_to_d(int lane) {
  return 0;
}

class Planner
{
public:
  Planner(Map* map) : v_des_(21.0), lane_des_(1), map_(map) { }

  /**
   * Plan a path from a start state, which is a 4-vector
   * of x, y, theta, speed
   */
  Path plan(Eigen::Vector4d const& start_state, Path path = Path()) {
    // Make spline
    // Note that all spline points are in the ego frame of the start_state
    std::vector<double> spline_x(1, -1);
    std::vector<double> spline_y(1, 0);
    spline_x.push_back(0);
    spline_y.push_back(0);
    Eigen::Vector2d start_sd = map_->get_frenet(start_state.head(3));
    for (double offset=30; offset <= 90; offset+=30) {
      Eigen::Vector2d sd(start_sd[0] + offset, map_->lane_center(lane_des_));
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
    
    // Pull points from the spline at given intervals
    // We don't need theta anymore, and this state is in the ego frame
    Eigen::Vector3d state(0, 0, start_state[3]);  // x, y, v
    size_t added = 0;
    while (path.size() < PATH_SIZE_) {
      // d is  the total distance to travel in this step, based on current velocity
      double d = state[2] * T_STEP_;
      // Calculate dx using current v and current tangent slope m
      double m = (spline(state[0]+0.01) - state[1]) / (.01);
      double dx = d/sqrt(1+m*m);
      // a is the desired acceleration
      double a = 0;
      if (state[2] < v_des_) {
        a = MAX_A_;
      }
      if (state[2] > v_des_) {
        a = -MAX_A_;
      }
      // State update equation:
      state[0] += dx;
      state[1] = spline(state[0]);
      state[2] += a * T_STEP_;
      // Convert point from ego frame to global frame and push onto the path
      Eigen::Vector2d xy = transform_to_global(start_state.head(3), state.head(2));
      path.push_back(xy[0], xy[1]);
      added++;
    }
    std::cout << "Added " << added << " points" << std::endl;
    return path;
  }

  /**
   * Plan a path that begins with a previous path
   */
  Path plan(Path const& path) {
    Eigen::Vector2d end_xy = path[path.size()-1];
    Eigen::Vector2d penultimate_xy = path[path.size()-2];
    Eigen::Vector2d d_xy = end_xy - penultimate_xy;
    double theta = atan2(d_xy[1], d_xy[0]);
    double v = d_xy.norm() / T_STEP_;
    auto end_sd = map_->get_frenet(end_xy[0], end_xy[1], theta);
    Eigen::Vector4d start_state(end_xy[0], end_xy[1], theta, v);
    return plan(start_state, path);    
  }
  
  private:
    const size_t PATH_SIZE_ = 50;  // Plan size, points
    const double T_STEP_ = 0.02;  // Time step, seconds
    double v_des_;  // Desired speed m/s
    int lane_des_;  // Desired lane (integer)
    const double MAX_A_ = 5.0;  // Max acceleration/decel, m/s^2
    Map* map_;
};
#endif  // PLANNER_H_
