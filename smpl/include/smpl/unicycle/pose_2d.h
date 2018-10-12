#ifndef SMPL_POSE_2D_H
#define SMPL_POSE_2D_H

// system includes
#include <Eigen/Dense>

namespace smpl {

struct Pose2D
{
    union {
        double data[3];

        struct {
            double x;
            double y;
            double theta;
        };
    };

    Pose2D() = default;

    Pose2D(double x, double y, double theta)
    {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    Pose2D(const Eigen::Vector2d& v, double theta)
    {
        this->x = v.x();
        this->y = v.y();
        this->theta = theta;
    }
};

inline auto pos(Pose2D pose) -> Eigen::Vector2d
{
    return Eigen::Vector2d(pose.x, pose.y);
}


} // namespace smpl

#endif

