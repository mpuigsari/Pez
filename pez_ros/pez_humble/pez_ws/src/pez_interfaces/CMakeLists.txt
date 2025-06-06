cmake_minimum_required(VERSION 3.5)
project(pez_interfaces)

# ─── Find what we need ───────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)  # for std_msgs/Header

# ─── Generate Python (and C++) service code from our .srv ────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SnapSensors.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ─── Install nothing else (no libs, no nodes) ────────────────────────────────────
ament_package()
