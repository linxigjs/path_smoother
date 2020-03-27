#ifndef BASIC_DEFINE_POSE2D_H
#define BASIC_DEFINE_POSE2D_H

#include <cmath>


class Pose2d {
public:
    //! Constructor which takes x- and y-coordinates.
    Pose2d(const double x, const double y, const double t) : x_(x), y_(y), theta_(t) {}

    //! Constructor returning the zero vector.
    Pose2d() : Pose2d(0, 0, 0) {}

    //! Getter for x component
    double x() const { return x_; }

    //! Getter for y component
    double y() const { return y_; }

    double theta() const { return theta_; }

    //! Setter for x component
    void set_x(const double x) { x_ = x; }

    //! Setter for y component
    void set_y(const double y) { y_ = y; }

    void set_theta(const double t) { theta_ = t; }

    //! Gets the length of the vector
    double Length() const { return hypot(x_, y_); }

    //! Gets the squared length of the vector
    double LengthSquare() const { return x_ * x_ + y_ * y_; }

    //! Returns the unit vector that is co-linear with this vector
    void Normalize()  {
        const double l = Length();
        const double kMathEpsilon = 1e-10;
        if (l > kMathEpsilon) {
            x_ /= l;
            y_ /= l;
        }
    }

    //! Returns the distance to the given vector
    double DistanceTo(const Pose2d &other) const  {
        return hypot(x_ - other.x_, y_ - other.y_);
    }

    //! Returns the squared distance to the given vector
    double DistanceSquareTo(const Pose2d &other) const  {
        const double dx = x_ - other.x_;
        const double dy = y_ - other.y_;
        return dx * dx + dy * dy;
    }

protected:
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
};


#endif //BASIC_DEFINE_POSE2D_H
