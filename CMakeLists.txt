cmake_minimum_required(VERSION 3.0.2)
project(imu_gnss_eskf)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  eigen_conversions
)
catkin_package()
find_package(Eigen3  REQUIRED)
find_package (GeographicLib REQUIRED)

include_directories(
 include/
 ${EIGEN3_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/imu_gnss_eskf_node.cpp src/eskf.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   Eigen3::Eigen
   ${GeographicLib_LIBRARIES})
