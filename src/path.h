#ifndef PATH_H_
#define PATH_H_
#include <cmath>
#include <vector>

class Path {
public:
  Path() : length_(0), valid_(false) {}

  Path(std::vector<double> const &x, std::vector<double> const &y)
      : x_(x), y_(y), length_(0) {
    if (size() > 1) {
      Eigen::Vector2d last_point(x_[0], y_[0]);
      for (size_t i = 1; i < size(); ++i) {
        Eigen::Vector2d point(x_[i], y_[i]);
        double ds = (point - last_point).norm();
        length_ += ds;
        last_point = point;
      }
    }
  }

  Path(Path const &p) : x_(p.x_), y_(p.y_), length_(p.length_), valid_(p.valid_) {}

  std::vector<double> get_x() const { return x_; }

  std::vector<double> get_y() const { return y_; }

  size_t size() const { return x_.size(); }

  void push_back(Eigen::Vector2d const& point) {
    push_back(point[0], point[1]);
  }

  void push_back(double x, double y) {
    x_.push_back(x);
    y_.push_back(y);
    size_t size = this->size();
    if (size > 1) {
      Eigen::Vector2d prev_point(x_[size - 2], y_[size - 2]);
      Eigen::Vector2d this_point(x_[size - 1], y_[size - 1]);
      double ds = (this_point - prev_point).norm();
      length_ += ds;
    }
  }

  double length() const { return length_; }

  double delta_s(Map* map) const {
    size_t n = size();
    if (n < 2) {
      return 0;
    }
    Eigen::Vector3d start_xy(x_[0], y_[0], bearing(x_[0], y_[0], x_[1], y_[1]));
    Eigen::Vector3d end_xy(x_[n-1], y_[n-1], bearing(x_[n-2], y_[n-2], x_[n-1], y_[n-1]));
    Eigen::Vector2d start_sd = map->get_frenet(start_xy);
    Eigen::Vector2d end_sd = map->get_frenet(end_xy);
    return fmod(end_sd[0] - start_sd[0], map->max_s_);

  }

  Eigen::Vector2d operator[](size_t pos) const {
    return Eigen::Vector2d(x_[pos], y_[pos]);
  }

  bool is_valid() const { return valid_; }

  void set_valid(bool valid) { valid_ = valid; };

private:
  std::vector<double> x_;
  std::vector<double> y_;
  double length_;
  bool valid_;
};
#endif // PATH_H_
