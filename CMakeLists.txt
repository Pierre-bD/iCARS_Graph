cmake_minimum_required(VERSION 3.20.0)
project(ic_graph VERSION 0.2.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)


#include(CTest)
enable_testing()
#
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
#Add Ros2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
#find_package(rosidl_default_generator REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/lib/imuIntegration.cpp
  src/lib/gnss.cpp
  src/lib/lidar.cpp
  src/imuFilter.cpp
)

add_executable(${PROJECT_NAME}_test src/test.cpp)
add_executable(${PROJECT_NAME}_optimization src/optimizarion.cpp)
add_executable(${PROJECT_NAME}_carlaLidar src/carla_lidar.cpp)
#add_executable(${PROJECT_NAME}_map src/map.cpp)
#add_executable(${PROJECT_NAME}_lidarOdom src/lidarOdom.cpp)

#rosidl_generate_interfaces(${PROJECT_NAME}
  #  "msg/preint.msg"
 #   "srv/TrigIntegration.srv"
    #DEPENDENCIES()
#)

target_link_libraries(${PROJECT_NAME} gtsam)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_optimization DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_imuIntegration DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY "include/" DESTINATION include)


#set(CPACK_PROJECT_NAME ${PROJECT_NAME})
#set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
#include(CPack)
ament_export_include_directories(include)

ament_package()
