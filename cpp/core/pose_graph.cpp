#include "core/pose_graph.hpp"

#include "core/pose_factor_abs.hpp"
#include "core/pose_factor_odom.hpp"
#include "core/pose_factor_rel.hpp"

namespace openlidarmap {

PoseGraph::PoseGraph(const config::Config &config, std::deque<Vector7d> &poses)
    : config_(config), poses_(poses) {
    solver_options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    solver_options_.max_num_iterations = config_.ceres_.num_iterations;
    solver_options_.minimizer_progress_to_stdout = config_.ceres_.print;
    solver_options_.num_threads = config_.ceres_.num_threads;

    pose_manifold_ =
        new ceres::ProductManifold<ceres::EuclideanManifold<3>, ceres::EigenQuaternionManifold>{};
}

void PoseGraph::addConstraint(size_t idx_from,
                              size_t idx_to,
                              const Vector7d &measurement,
                              bool absolute = false) {
    if (poses_.size() > constraints_.size()) {
        constraints_.emplace_back(std::make_pair(nullptr, nullptr));
    }

    if (absolute) {
        ceres::ResidualBlockId abs_block_id = problem_.AddResidualBlock(
            new ceres::AutoDiffCostFunction<AbsPoseError, 6, 7>(new AbsPoseError(measurement)),
            new ceres::TukeyLoss(1.0), poses_[idx_to].data());
        constraints_[idx_to].first = abs_block_id;
        problem_.SetManifold(poses_[idx_to].data(), pose_manifold_);
    } else {
        ceres::ResidualBlockId rel_block_id = problem_.AddResidualBlock(
            new ceres::AutoDiffCostFunction<RelPoseError, 6, 7, 7>(new RelPoseError(measurement)),
            new ceres::CauchyLoss(1.0), poses_[idx_from].data(), poses_[idx_to].data());
        problem_.SetManifold(poses_[idx_from].data(), pose_manifold_);
        problem_.SetManifold(poses_[idx_to].data(), pose_manifold_);
        constraints_[idx_to].second = rel_block_id;
    }

    manageSlidingWindow();
}

void PoseGraph::addOdometryConstraint(size_t idx_from, size_t idx_to, const Vector7d &delta) {
    if (poses_.size() > constraints_.size()) {
        constraints_.emplace_back(std::make_pair(nullptr, nullptr));
    }

    ceres::ResidualBlockId odom_block_id = problem_.AddResidualBlock(
        new ceres::AutoDiffCostFunction<OdomPoseError, 6, 7, 7>(
            new OdomPoseError(delta, config_.odometry_.translation_stddev,
                              config_.odometry_.rotation_stddev)),
        new ceres::HuberLoss(1.0), poses_[idx_from].data(), poses_[idx_to].data());
    problem_.SetManifold(poses_[idx_from].data(), pose_manifold_);
    problem_.SetManifold(poses_[idx_to].data(), pose_manifold_);
    constraints_[idx_to].second = odom_block_id;

    manageSlidingWindow();
}

bool PoseGraph::optimize() {
    ceres::Solve(solver_options_, &problem_, &summary_);
    return summary_.IsSolutionUsable();
}

void PoseGraph::manageSlidingWindow() {
    if (current_fixed_pose_ == -1) {
        problem_.SetParameterBlockConstant(poses_[0].data());
        current_fixed_pose_ = 0;
    }

    if (!config_.ceres_.use_sliding_window) {
        return;
    }

    size_t oldest_allowed = (poses_.size() > config_.ceres_.sliding_window_size)
                                ? poses_.size() - config_.ceres_.sliding_window_size
                                : 0;

    if (oldest_allowed > 0 && current_fixed_pose_ != static_cast<int>(oldest_allowed)) {
        auto &constraint = constraints_[oldest_allowed - 1];
        if (constraint.first != nullptr) {
            problem_.RemoveResidualBlock(constraint.first);
            constraint.first = nullptr;
        }
        if (constraint.second != nullptr) {
            problem_.RemoveResidualBlock(constraint.second);
            constraint.second = nullptr;
        }
        problem_.SetParameterBlockConstant(poses_[oldest_allowed].data());
        current_fixed_pose_ = oldest_allowed;
    }
}

}  // namespace openlidarmap
