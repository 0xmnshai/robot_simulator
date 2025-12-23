cmake_minimum_required(VERSION 3.20)
project(robot_simulator_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
set(SDL2_HINT_PATHS "/opt/homebrew")
set(SDL2_IMAGE_HINT_PATHS "/opt/homebrew/Cellar/sdl2_image/2.8.8")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(SDL2 REQUIRED HINTS ${SDL2_HINT_PATHS})
find_package(SDL2_image REQUIRED HINTS ${SDL2_IMAGE_HINT_PATHS})

include_directories(
  include
  ${SDL2_INCLUDE_DIRS}
  ${SDL2_IMAGE_INCLUDE_DIRS}
  /opt/homebrew/Cellar/sdl2_image/2.8.8/include
)

set(SOURCES
  src/main.cpp
  src/robot.cpp
  src/ultrasonic.cpp
  src/graphics.cpp
  src/ros_interface.cpp
)

add_executable(robot_simulator ${SOURCES})

target_link_libraries(robot_simulator PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  ${SDL2_LIBRARIES}
  ${SDL2_IMAGE_LIBRARIES}
  /opt/homebrew/Cellar/sdl2_image/2.8.8/lib/libSDL2_image.dylib
)

install(TARGETS robot_simulator 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED) 
  set(ament_cmake_copyright_FOUND TRUE) 
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(ament_cmake_cppcheck_FOUND FALSE)
set(AMENT_CMAKE_CPPCHECK_DISABLED TRUE)
set(ament_cmake_cppcheck_ENABLED FALSE)

ament_package()
