#ifndef PATH_H_
#define PATH_H_
#include <vector>

class Path {
public:
  Path() : length_(0) {}

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

  Path(Path const &p) : x_(p.x_), y_(p.y_), length_(p.length_) {}

  std::vector<double> get_x() const { return x_; }

  std::vector<double> get_y() const { return y_; }

  size_t size() const { return x_.size(); }

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

  Eigen::Vector2d operator[](size_t pos) const {
    return Eigen::Vector2d(x_[pos], y_[pos]);
  }

private:
  std::vector<double> x_;
  std::vector<double> y_;
  double length_;
};
#endif // PATH_H_
