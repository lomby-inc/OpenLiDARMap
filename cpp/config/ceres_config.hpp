#pragma once

#include <ceres/ceres.h>

namespace openlidarmap::config {

struct CeresConfig {
    ceres::LinearSolverType solver_type{};
    int num_iterations{};
    bool print{};
    int num_threads{};
    size_t sliding_window_size{};
    bool use_sliding_window{};

    CeresConfig()
        : solver_type(ceres::SPARSE_NORMAL_CHOLESKY),
          num_iterations(100),
          print(false),
          num_threads(6),
          sliding_window_size(25),
          use_sliding_window(true) {}
};

}  // namespace openlidarmap::config
