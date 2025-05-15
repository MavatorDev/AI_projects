cmake_minimum_required(VERSION 3.8)
project(ar_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#Build defined custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CubicTrajCoeffs.msg"
  "msg/CubicTrajParams.msg"
  "srv/ComputeCubicTraj.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
