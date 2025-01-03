cmake_minimum_required(VERSION 3.8)
project(rover_esp32)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED)

# Include the wiringPi library 
find_library(WIRINGPI_LIB NAMES wiringPi PATHS /usr/lib)

add_executable(home_rover src/home_rover.cpp)
target_include_directories(home_rover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(home_rover PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(home_rover rclcpp std_msgs)
target_link_libraries(home_rover ${WIRINGPI_LIB})

install(TARGETS home_rover
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
