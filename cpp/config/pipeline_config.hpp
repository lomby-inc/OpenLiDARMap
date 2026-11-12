#pragma once

#include <cstddef>

namespace openlidarmap::config {

struct PipelineConfig {
    double translation_threshold{};
    double rotation_threshold{};
    bool visualize{};
    bool save_submaps{};
    double map_save_interval{};

    PipelineConfig()
        : translation_threshold(0.05),
          rotation_threshold(0.05),
          visualize(true),
          save_submaps(true),
          map_save_interval(50.0) {}
};

}  // namespace openlidarmap::config
