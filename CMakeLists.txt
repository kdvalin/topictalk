cmake_minimum_required(VERSION 3.8)
project(topictalk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(topictalk_publisher src/topictalk_publisher.cpp)
add_executable(topictalk_listener src/topictalk_listener.cpp)
set(EXECUTABLES "topictalk_listener;topictalk_publisher")

foreach(EXECUTABLE IN LISTS EXECUTABLES)
  target_include_directories(${EXECUTABLE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_compile_features(${EXECUTABLE} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

  ament_target_dependencies(
    ${EXECUTABLE}
    "rclcpp"
    "std_msgs"
  )

  install(TARGETS ${EXECUTABLE}
    DESTINATION lib/${PROJECT_NAME})
endforeach(EXECUTABLE IN LISTS EXECUTABLES)

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
