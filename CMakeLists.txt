cmake_minimum_required(VERSION 3.8)
project(statistical_outlier_removal_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Defining used ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# sudo apt install pcl* pcl-tools libpcl-dev libpcap-dev
# Installing Point Cloud Library
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Creating C++ executables and add them their implemented ROS2 packages
add_executable (sor_filter src/sor_filter.cpp)
target_link_libraries (sor_filter ${PCL_LIBRARIES})
ament_target_dependencies(sor_filter rclcpp)

add_executable (sor_filter_server src/sor_filter_server.cpp)
target_link_libraries (sor_filter_server ${PCL_LIBRARIES})
ament_target_dependencies(sor_filter_server rclcpp std_srvs)

add_executable (sor_filter_client src/sor_filter_client.cpp)
target_link_libraries (sor_filter_client ${PCL_LIBRARIES})
ament_target_dependencies(sor_filter_client rclcpp std_srvs)

# Installing custom C++ executables in ROS2 ecosystem
install(TARGETS
  sor_filter 
  sor_filter_server
  sor_filter_client
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
