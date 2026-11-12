cmake_minimum_required(VERSION 3.20)
project(openlidarmap_pybind VERSION 0.0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

set(PYBIND11_NEWPYTHON ON)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Resolve bundled libraries next to the module and non-system deps (e.g. Iridescence) via link paths
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Build core library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp ${CMAKE_BINARY_DIR}/../cpp)

# Python bindings
pybind11_add_module(openlidarmap_pybind MODULE pybind/openlidarmap_pybind.cpp)
target_link_libraries(openlidarmap_pybind PRIVATE openlidarmap_lib)

# Install into Python package structure
set(PYTHON_PACKAGE_DIR "openlidarmap_pybind")
install(TARGETS openlidarmap_lib small_gicp DESTINATION ${PYTHON_PACKAGE_DIR})
install(TARGETS openlidarmap_pybind DESTINATION ${PYTHON_PACKAGE_DIR})
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/${PYTHON_PACKAGE_DIR}/__init__.py" "from .openlidarmap_pybind import *")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PYTHON_PACKAGE_DIR}/__init__.py" DESTINATION ${PYTHON_PACKAGE_DIR})
