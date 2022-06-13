cmake_minimum_required(VERSION 3.5)
project(mpi_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(EXT_LIBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/qpOASES")
include_directories("${EXT_LIBS_DIR}/include")

include_directories(${EIGEN3_INCLUDE_DIR})

add_library(mpi_controller
  SHARED
  src/mpi_controller.cpp
  src/mpic.cpp
)

# qpOASES
add_library(libqpOASES STATIC IMPORTED qpOASES.hpp)
set_property(TARGET libqpOASES PROPERTY IMPORTED_LOCATION "${EXT_LIBS_DIR}/libqpOASES.so")

target_include_directories(mpi_controller PRIVATE include ${EXT_LIBS_DIR}/include)

ament_target_dependencies(mpi_controller
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  trajectory_msgs
  Eigen3
)

target_link_libraries(mpi_controller libqpOASES)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mpi_controller PRIVATE "MPI_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(mpi_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  mpi_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_mpi_controller
    test/test_load_mpi_controller.cpp
  )

  target_include_directories(
    test_load_mpi_controller 
    PRIVATE
    include
  )

  ament_target_dependencies(
    test_load_mpi_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  # ament_add_gmock(
  #   test_mpi_controller
  #   test/test_mpi_controller.cpp
  # )

  # target_include_directories(
  #   test_mpi_controller
  #   PRIVATE 
  #   include
  #   ${EXT_LIBS_DIR}/include
  # )

  # target_link_libraries(
  #   test_mpi_controller
  #   mpi_controller
  #   libqpOASES
  # )
  # ament_target_dependencies(
  #   test_mpi_controller
  #   controller_interface
  #   hardware_interface
  #   rclcpp
  #   rclcpp_lifecycle
  #   realtime_tools
  #   std_msgs
  # )
endif()

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  mpi_controller
)
ament_package()