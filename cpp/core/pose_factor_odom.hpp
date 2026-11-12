#pragma once

#include "config/types.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace openlidarmap {
// Relative pose between two graph nodes measured by external odometry: T_i^-1 * T_j = delta.
struct OdomPoseError {
    OdomPoseError(const Vector7d &delta, double translation_stddev, double rotation_stddev)
        : delta_(delta),
          translation_stddev_(translation_stddev),
          rotation_stddev_(rotation_stddev) {}

    template <typename T>
    bool operator()(const T *const pose_i, const T *const pose_j, T *residual) const {
        const Eigen::Matrix<T, 3, 1> t_i(pose_i[0], pose_i[1], pose_i[2]);
        const Eigen::Quaternion<T> q_i =
            Eigen::Quaternion<T>(pose_i[6], pose_i[3], pose_i[4], pose_i[5]).normalized();
        const Eigen::Matrix<T, 3, 1> t_j(pose_j[0], pose_j[1], pose_j[2]);
        const Eigen::Quaternion<T> q_j =
            Eigen::Quaternion<T>(pose_j[6], pose_j[3], pose_j[4], pose_j[5]).normalized();

        // Estimated relative pose
        const Eigen::Quaternion<T> q_ij = q_i.conjugate() * q_j;
        const Eigen::Matrix<T, 3, 1> t_ij = q_i.conjugate() * (t_j - t_i);

        const Eigen::Quaternion<T> q_delta{T(delta_[6]), T(delta_[3]), T(delta_[4]),
                                           T(delta_[5])};
        const Eigen::Matrix<T, 3, 1> t_delta{T(delta_[0]), T(delta_[1]), T(delta_[2])};

        // Error = delta^-1 * T_ij
        const Eigen::Quaternion<T> q_error = q_delta.normalized().conjugate() * q_ij;
        const Eigen::Matrix<T, 3, 1> t_error = q_delta.normalized().conjugate() * (t_ij - t_delta);

        for (int k = 0; k < 3; ++k) {
            residual[k] = t_error[k] / T(translation_stddev_);
        }
        residual[3] = T(2.0) * q_error.x() / T(rotation_stddev_);
        residual[4] = T(2.0) * q_error.y() / T(rotation_stddev_);
        residual[5] = T(2.0) * q_error.z() / T(rotation_stddev_);

        return true;
    }

private:
    const Vector7d delta_;
    const double translation_stddev_;
    const double rotation_stddev_;
};
}  // namespace openlidarmap
