#include <iostream>
#include "obstacle.h"

int main(int argc, char** argv) {
  Eigen::Vector2d car(0, 0);
  std::vector<Obstacle> obstacles;
  obstacles.emplace_back(obstacles.size(), 5.0, 0, -0.5, 0, 0, 0);
  double collision_radius = 3.0;
  for (double time = 0; time < 10; time += 1.0) {
    std::cout << "Time=" << time<< "; obstacle is at (" << obstacles[0].xy(time)[0] << ", " << obstacles[0].xy(time)[1] << ")" << std::endl;
    check_collisions(car, obstacles, collision_radius, time);
  }
  return 0;
}
