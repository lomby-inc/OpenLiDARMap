#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "config/config.hpp"
#include "config/types.hpp"
#include "pipeline/openlidarmap.hpp"

namespace py = pybind11;

PYBIND11_MODULE(openlidarmap_pybind, m) {
    m.doc() = "Python bindings for OpenLiDARMap";

    py::class_<openlidarmap::config::Config>(m, "Config")
        .def(py::init<>(), "Initialize default configuration");

    py::class_<openlidarmap::pipeline::Pipeline>(m, "Pipeline")
        .def(py::init<const openlidarmap::config::Config &>())
        .def("initialize", &openlidarmap::pipeline::Pipeline::initialize)
        .def("run", &openlidarmap::pipeline::Pipeline::run);

    m.def(
        "process_map",
        [](const std::string &map_path, const std::string &scans_dir,
           const std::string &output_path, const py::array_t<double> &initial_pose_np,
           const std::string &odometry_path, bool visualize) {
            try {
                openlidarmap::config::Config config{};
                config.pipeline_.visualize = visualize;
                openlidarmap::pipeline::Pipeline pipeline(config);

                openlidarmap::Vector7d initial_pose;
                if (initial_pose_np.size() != 7) {
                    throw std::invalid_argument("initial_pose must be a NumPy array of size 7");
                }

                py::buffer_info buf = initial_pose_np.request();
                double *ptr = (double *)buf.ptr;
                for (size_t i = 0; i < 7; ++i) {
                    initial_pose[i] = ptr[i];
                }

                if (!pipeline.initialize(map_path, scans_dir, output_path, initial_pose,
                                         odometry_path)) {
                    throw std::runtime_error("Failed to initialize pipeline");
                }

                if (!pipeline.run()) {
                    throw std::runtime_error("Pipeline execution failed");
                }
            } catch (const std::exception &e) {
                throw std::runtime_error(std::string("Pipeline error: ") + e.what());
            }
        },
        "Process map with given parameters", py::arg("map_path"), py::arg("scans_dir"),
        py::arg("output_path"), py::arg("initial_pose"), py::arg("odometry_path") = "",
        py::arg("visualize") = true);
}
