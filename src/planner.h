#ifndef PLANNER_H_
#define PLANNER_H_
#include "map.h"
#include "path.h"
#include "Eigen-3.3/Eigen/Dense"

double lane_to_d(int lane) {
  return 0;
}

class Planner
{
public:
  Planner(Map* map) : v_des_(21.0), lane_des_(1), map_(map) { }

  /**
   * Plan a path from a start state, which is a 3-vector
   * of s, d, speed
   */
  Path plan(Eigen::VectorXd const& start_state, Path path = Path()) {
    double dist = 0;
    Eigen::VectorXd state = start_state;
    Eigen::MatrixXd A(3, 3);
    A << 1, 0, T_STEP_,
         0, 1, 0,
         0, 0, 1;
    Eigen::VectorXd B(3);
    B << 0, 0, T_STEP_;
    while (path.length() < HORIZON_) {
      double a = 0;
      if (state[2] < v_des_) {
        a = MAX_A_;
      }
      if (state[2] > v_des_) {
        a = -MAX_A_;
      }
      state = A * state + B * a;
      auto xy = map_->get_xy(state[0], state[1]);
      path.push_back(xy[0], xy[1]);
    }
    std::cout << std::endl;
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
    Eigen::VectorXd start_state(3);
    start_state << end_sd[0], end_sd[1], v;
    return plan(start_state, path);    
  }
  
  private:
    const double HORIZON_ = 30.0;  // Planning distance, meters
    const double T_STEP_ = 0.02;  // Time step, seconds
    double v_des_;  // Desired speed m/s
    int lane_des_;  // Desired lane (integer)
    const double MAX_A_ = 5.0;  // Max acceleration/decel, m/s^2
    Map* map_;
};
#endif  // PLANNER_H_
