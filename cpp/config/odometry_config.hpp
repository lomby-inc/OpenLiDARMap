#pragma once

namespace openlidarmap::config {

// External (e.g. LiDAR-inertial) odometry, used in place of scan-to-scan registration when an
// odometry file is given to Pipeline::initialize.
struct OdometryConfig {
    double translation_stddev{};  // [m] per pose graph edge
    double rotation_stddev{};     // [rad] per pose graph edge

    OdometryConfig() : translation_stddev(0.02), rotation_stddev(0.005) {}
};

}  // namespace openlidarmap::config
