#pragma once

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "config/config.hpp"
#include "core/pose_graph.hpp"
#include "core/prediction.hpp"
#include "core/preprocess.hpp"
#include "core/registration.hpp"
#include "utils/file_utils.hpp"
#include "utils/helpers.hpp"
#include "utils/pose_utils.hpp"

namespace guik {
class LightViewer;
}

namespace openlidarmap::pipeline {

class Pipeline {
public:
    Pipeline(const config::Config &config);
    ~Pipeline();

    bool initialize(const std::string &map_path,
                    const std::string &scans_dir,
                    const std::string &output_path,
                    const Vector7d &initial_pose = Vector7d(0, 0, 0, 0, 0, 0, 1),
                    const std::string &odometry_path = "");

    bool run();
    void writeResults() const;
    const std::deque<Vector7d> &getPoses() const { return poses_; }
    void addPose(const Vector7d &pose);
    bool processFrame(small_gicp::PointCloud::Ptr &frame);

private:
    bool initializeFirstPoses(const Vector7d &initial_pose);
    bool loadOdometry(const std::string &odometry_path);
    Eigen::Isometry3d odometryDelta(size_t from_scan, size_t to_scan) const;
    bool processFrameWithOdometry(small_gicp::PointCloud::Ptr &frame);
    void saveSubmapFrame(const small_gicp::PointCloud::Ptr &frame);
    void updatePoseGraph(const small_gicp::RegistrationResult &scan2map_result,
                         const small_gicp::RegistrationResult &scan2scan_result);
    Vector7d predictNextPose();

    void updateVisualization(const small_gicp::PointCloud::Ptr &cloud);
    void handleVisualizationControls(guik::LightViewer *viewer);
    void waitIfPaused();
    void processingLoop();
    void saveSubmap();
    double calculateTravelledDistance(const Vector7d &pose1, const Vector7d &pose2);

    config::Config config_{};
    config::Config scan2scan_config_{};
    config::Config scan2map_config_{};
    std::unique_ptr<Preprocess> preprocess_{};
    std::unique_ptr<Registration> scan2map_registration_{};
    std::unique_ptr<Registration> scan2scan_registration_{};
    std::unique_ptr<PoseGraph> pose_graph_{};

    std::vector<std::string> scan_files_{};
    std::deque<Vector7d> poses_{};
    std::vector<Vector7d> kitti_poses_{};
    std::string output_path_{};

    size_t frame_count_{1};
    utils::Stopwatch timer_{};
    size_t pose_index_{1};

    // External odometry (one pose per scan file, in the scans' frame); replaces scan-to-scan
    // registration for prediction and the relative pose graph constraints when non-empty
    std::vector<Eigen::Isometry3d> odometry_{};
    size_t current_scan_{0};
    size_t last_node_scan_{0};

    std::mutex pause_mutex_;
    std::mutex visualization_mutex_;
    std::mutex viewer_mutex_;
    std::condition_variable pause_cv_;
    std::atomic<bool> paused_{false};
    std::atomic<bool> stop_requested_{false};
    // Set once processingLoop has written its results, so run() returns without the viewer
    // having to be closed
    std::atomic<bool> processing_done_{false};
    std::thread processing_thread_;
    double current_processing_time_{0.0};
    float progress_{0.0f};

    double last_save_distance_{0.0};
    int submap_counter_{0};
    small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerPoints>::Ptr current_submap_;
};

}  // namespace openlidarmap::pipeline
