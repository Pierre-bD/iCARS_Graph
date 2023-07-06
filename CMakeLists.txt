cmake_minimum_required(VERSION 3.20.0)
project(ic_graph VERSION 0.2.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)


#include(CTest)
enable_testing()
#
find_package(GTSAM REQUIRED)
#find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
#Add Ros2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(PCL REQUIRED)
#find_package(rosidl_default_generator REQUIRED)

include_directories(
 include
 ${EIGEN3_INCLUDE_DIR}
 ${PCL_INCLUDE_DIRS}
 ${GTSAM_INCLUDE_DIRS}
 ${GeographicLib_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS} ${GeographicLib_INCLUDE_DIRS})
set(library_name graph_lib)

add_library(${library_name} SHARED
  src/lib/imuIntegration.cpp 
  src/lib/gnss.cpp
  #src/lib/lidar.cpp
)
target_link_libraries(${library_name} gtsam gtsam_unstable GeographicLib Eigen3::Eigen)
ament_target_dependencies(${library_name} rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(${PROJECT_NAME}_node
  src/optimization.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${library_name} gtsam gtsam_unstable Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros)



#ament_export_include_directories(include)
#ament_export_dependencies(PCL gtsam Eigen)
#ament_export_libraries(${PROJECT_NAME})

#ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(TARGETS 
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
#install(
 #DIRECTORY include/${PROJECT_NAME}
 #DESTINATION include
 #)

#install(TARGETS ${PROJECT_NAME} 
 #EXPORT ${PROJECT_NAME}
 #ARCHIVE DESTINATION lib
 #LIBRARY DESTINATION lib
 #RUNTIME DESTINATION bin
 #INCLUDES DESTINATION include
 #)

ament_package()
