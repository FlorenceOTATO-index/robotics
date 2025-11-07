cmake_minimum_required(VERSION 3.14.4)
project(flightgoggles_uav_dynamics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS})


add_library(inertialMeasurementSim libs/multicopterDynamicsSim/inertialMeasurementSim.cpp)
add_library(multicopterDynamicsSim libs/multicopterDynamicsSim/multicopterDynamicsSim.cpp)

add_executable(${PROJECT_NAME}_node src/flightgoggles_uav_dynamics_node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

target_link_libraries(inertialMeasurementSim Eigen3::Eigen)
target_link_libraries(multicopterDynamicsSim Eigen3::Eigen)
target_link_libraries(multicopterDynamicsSim inertialMeasurementSim)

## Specify libraries to link a library or executable target against
target_link_libraries(flightgoggles_uav_dynamics_node PUBLIC
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  multicopterDynamicsSim
)

install(TARGETS flightgoggles_uav_dynamics_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()