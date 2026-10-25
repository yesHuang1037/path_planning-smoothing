cmake_minimum_required(VERSION 3.8)
project(path_searching)

# ---------- 编译选项 ----------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Workaround: CMake 4.x + OpenMPI wrapper may leave "-I..." in MPI include cache,
# which becomes a non-existent relative path (e.g. "<src>/-I/usr/...") during PCL/VTK MPI checks.
find_program(_MPICC_EXECUTABLE NAMES mpicc)
if(_MPICC_EXECUTABLE)
  execute_process(
    COMMAND ${_MPICC_EXECUTABLE} --showme:compile
    OUTPUT_VARIABLE _MPI_SHOWME_COMPILE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  string(REGEX MATCHALL "(^| )-I[^ ]+" _MPI_INCLUDE_FLAGS "${_MPI_SHOWME_COMPILE}")
  set(_MPI_INCLUDE_DIRS "")
  foreach(_flag IN LISTS _MPI_INCLUDE_FLAGS)
    string(REGEX REPLACE "(^| )-I" "" _inc "${_flag}")
    if(IS_ABSOLUTE "${_inc}" AND EXISTS "${_inc}")
      list(APPEND _MPI_INCLUDE_DIRS "${_inc}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _MPI_INCLUDE_DIRS)
  if(_MPI_INCLUDE_DIRS)
    set(MPI_C_COMPILER_INCLUDE_DIRS "${_MPI_INCLUDE_DIRS}" CACHE STRING "Sanitized MPI include dirs for FindMPI" FORCE)
    set(MPI_C_ADDITIONAL_INCLUDE_DIRS "" CACHE STRING "Additional MPI include dirs" FORCE)
  endif()
endif()

# ---------- 找包 ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ---------- 算法库 ----------
add_library(${PROJECT_NAME}_lib STATIC
  src/Astar.cpp
  src/bucketedqueue.cpp
  src/dynamic_obstacle_manager.cpp
  src/dynamicvoronoi.cpp
  src/esdf_map.cpp
  src/path_safety_checker.cpp
  src/path_simplifier.cpp
  src/replan_coordinator.cpp
  src/smoother.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp nav_msgs map_msgs geometry_msgs sensor_msgs tf2_ros tf2_eigen tf2_geometry_msgs
  Eigen3 pcl_conversions visualization_msgs std_msgs OpenCV
)
target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})

# ---------- 可执行节点 ----------
add_executable(plan_manager src/plan_manager.cpp)
ament_target_dependencies(plan_manager
  rclcpp nav_msgs map_msgs geometry_msgs sensor_msgs tf2_ros tf2_eigen tf2_geometry_msgs
  Eigen3 pcl_conversions visualization_msgs std_msgs OpenCV
)
target_link_libraries(plan_manager ${PROJECT_NAME}_lib)

# Benchmark executable for ESDF performance testing
add_executable(esdf_benchmark src/benchmark_esdf.cpp)
target_link_libraries(esdf_benchmark ${PROJECT_NAME}_lib)
ament_target_dependencies(esdf_benchmark Eigen3 OpenCV)

# ---------- 包含目录 ----------
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/searching>
  ${PCL_INCLUDE_DIRS}
  $<INSTALL_INTERFACE:include>
)
# ---------- 安装 ----------
install(TARGETS plan_manager esdf_benchmark ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(path_simplifier_test test/path_simplifier_test.cpp)
  target_link_libraries(path_simplifier_test ${PROJECT_NAME}_lib)
  add_test(NAME path_simplifier_test COMMAND path_simplifier_test)

  add_executable(smoother_simplification_test test/smoother_simplification_test.cpp)
  target_link_libraries(smoother_simplification_test ${PROJECT_NAME}_lib)
  add_test(NAME smoother_simplification_test COMMAND smoother_simplification_test)
  add_executable(replan_coordinator_periodic_test
    test/replan_coordinator_periodic_test.cpp
  )
  target_link_libraries(replan_coordinator_periodic_test ${PROJECT_NAME}_lib)
  ament_target_dependencies(replan_coordinator_periodic_test Eigen3)
  add_test(
    NAME replan_coordinator_periodic_test
    COMMAND replan_coordinator_periodic_test
  )

  add_executable(path_safety_checker_truncation_test
    test/path_safety_checker_truncation_test.cpp
  )
  target_link_libraries(path_safety_checker_truncation_test ${PROJECT_NAME}_lib)
  ament_target_dependencies(path_safety_checker_truncation_test Eigen3 OpenCV)
  add_test(
    NAME path_safety_checker_truncation_test
    COMMAND path_safety_checker_truncation_test
  )

  add_executable(astar_start_escape_test
    test/astar_start_escape_test.cpp
  )
  target_link_libraries(astar_start_escape_test ${PROJECT_NAME}_lib)
  ament_target_dependencies(astar_start_escape_test Eigen3 OpenCV)
  add_test(
    NAME astar_start_escape_test
    COMMAND astar_start_escape_test
  )
endif()

# ---------- 导出 ----------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp nav_msgs map_msgs geometry_msgs sensor_msgs tf2_ros tf2_eigen tf2_geometry_msgs
  Eigen3 pcl_conversions visualization_msgs std_msgs OpenCV
)

ament_package()
