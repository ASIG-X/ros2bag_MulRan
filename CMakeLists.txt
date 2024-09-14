cmake_minimum_required(VERSION 3.5)
project(ros2bag_mulran)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -Wall -g")

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE release)
endif ()
add_compile_options(-g)
find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
set(CMAKE_AUTOMOC ON)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(${PROJECT_NAME}
    src/BagWriter.cpp
    src/main.cpp
)
ament_lint_auto_find_test_dependencies()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(bagWriter src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
	rclcpp
	std_msgs
	geometry_msgs
	nav_msgs
	image_transport
	sensor_msgs
	pcl_conversions
	cv_bridge
	tf2_ros
	rosbag2_cpp)

target_link_libraries(bagWriter
  ${PROJECT_NAME}
  ${BOOST_CUSTOM_LIBS}
  ${Eigen3_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS bagWriter DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclcpp message_runtime)
ament_export_libraries(${PROJECT_NAME})
ament_package()
