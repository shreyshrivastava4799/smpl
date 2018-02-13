#ifndef SMPL_UNICYCLE_H
#define SMPL_UNICYCLE_H

#include <cmath>
#include <cstdlib>
#include <limits>

namespace sbpl {

struct Pose2D
{
    double data_[3];

    Pose2D() = default;

    Pose2D(double x, double y, double theta)
    {
        data_[0] = x;
        data_[1] = y;
        data_[2] = theta;
    }

    double& x() { return data_[0]; }
    double& y() { return data_[1]; }
    double& theta() { return data_[2]; }

    double x() const { return data_[0]; }
    double y() const { return data_[1]; }
    double theta() const { return data_[2]; }
};

struct UnicycleMotion
{
    Pose2D start = Pose2D{ 0.0, 0.0, 0.0 };
    Pose2D goal = Pose2D{ 0.0, 0.0, 0.0 };

    // determined by geometry alone
    double l = 0.0; // length of straight line segment
    double r = 0.0; // turning radius

    // derived from l and r, normalized so that 0 <= t <= 1
    double w = 0.0; // angular velocity
    double v = 0.0; // linear velocity

    // time to end of the straight line segment
    double tl = 0.0;

    // whether the solver could find a valid solution
    //
    // invalid solutions occur when either:
    // (1) the start and goal heading are the same but the goal position is not
    //     inline with the start heading
    // (2) the start and goal heading are different but the goal position is
    //     inline with the start heading so that a turn of radius 0
    //     (turn-in-place) is required to meet the final orientation
    bool valid = false;

    auto operator()(double t) const -> Pose2D
    {
        return at(t);
    }

    double length() const {
        const double arc_len = r * std::fabs(goal.theta() - start.theta()); //w * (1.0 - tl);
        return std::fabs(l) + std::fabs(arc_len);
    }

    bool is_valid() const { return valid; }

    auto at(double t) const -> Pose2D
    {
        if (t <= tl) {
            double x = start.x() + v * t * std::cos(start.theta());
            double y = start.y() + v * t * std::sin(start.theta());
            double theta = start.theta();
            return Pose2D{ x, y, theta };
        } else {
            double x =
                    start.x()
                    + l * std::cos(start.theta())
                    + r * std::sin(w * (t - tl) + start.theta())
                    - r * std::sin(start.theta());
            double y =
                    start.y()
                    + l * std::sin(start.theta())
                    - r * std::cos(w * (t - tl) + start.theta())
                    + r * std::cos(start.theta());
            double theta = start.theta() + w * (t - tl);
            return Pose2D{ x, y, theta };
        }
    }
};

auto MakeUnicycleMotion(
    double start_x, double start_y, double start_theta,
    double goal_x, double goal_y, double goal_theta,
    double eps = std::numeric_limits<double>::epsilon())
    -> UnicycleMotion;

auto MakeUnicycleMotion(
    const Pose2D& start,
    const Pose2D& goal,
    double eps = std::numeric_limits<double>::epsilon())
    -> UnicycleMotion
{
    return MakeUnicycleMotion(
            start.x(), start.y(), start.theta(),
            goal.x(), goal.y(), goal.theta(),
            eps);
}

} // namespace sbpl

#endif
