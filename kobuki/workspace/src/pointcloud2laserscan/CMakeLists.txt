cmake_minimum_required(VERSION 3.8)
project(hsl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)

# find_package(Boost COMPONENTS system REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (pointcloud2laserscan src/pointcloud2laserscan.cpp)
target_link_libraries (pointcloud2laserscan ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud2laserscan rclcpp sensor_msgs pcl_conversions PCL) 

install(TARGETS
pointcloud2laserscan
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

