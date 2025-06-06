cmake_minimum_required(VERSION 3.5)
project(pez_interfaces)

# ─── Explicitly find Python 3 (interpreter + headers + libs) ───────────────────
# This guarantees that CMake knows where Python3.h and libpython3.so live.
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Python3 include dirs: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3 library:   ${Python3_LIBRARIES}")

# ─── Find ament + rosidl generator ─────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # for std_msgs/Header

# ─── Generate Python (and C++) service code from our .srv ───────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SnapSensors.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
