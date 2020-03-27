#ifndef BASIC_DEFINE_VEC2I__H
#define BASIC_DEFINE_VEC2I__H

#include <cmath>


class Vec2i {
 public:
  //! Constructor which takes x- and y-coordinates.
  Vec2i(const int x, const int y) : x_(x), y_(y) {}

  //! Constructor returning the zero vector.
  Vec2i() : Vec2i(0, 0) {}

  //! Getter for x component
  int x() const { return x_; }

  //! Getter for y component
  int y() const { return y_; }

  //! Setter for x component
  void set_x(const int x) { x_ = x; }

  //! Setter for y component
  void set_y(const int y) { y_ = y; }

  //! Returns the distance to the given vector
  double DistanceTo(const Vec2i &other) const {
    return hypot(x_ - other.x_, y_ - other.y_);
  }

  //! Returns the squared distance to the given vector
  int DistanceSquareTo(const Vec2i &other) const {
    const int dx = x_ - other.x_;
    const int dy = y_ - other.y_;
    return dx * dx + dy * dy;
  }

  //! Compares two Vec2i
  bool operator==(const Vec2i &other) const {
    return x_ == other.x() && y_ == other.y();
  }

 protected:
  int x_ = 0;
  int y_ = 0;
};

#endif //BASIC_DEFINE_VEC2I__H