cmake_minimum_required(VERSION 3.0.2)
project(imu_gnss_eskf)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)
catkin_package()
include_directories(
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/imu_gps_eskf.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node src/imu_gnss_eskf_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} )
