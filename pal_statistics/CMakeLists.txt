cmake_minimum_required(VERSION 3.5.0)
project(pal_statistics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(Boost REQUIRED)
find_package(ament_cmake REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)

find_package(pal_statistics_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)

# catkin_python_setup() System dependencies are found with CMake's conventions

include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})
include_directories(include)

# Declare a C++ library
add_library(
  ${PROJECT_NAME} src/pal_statistics_utils.cpp src/pal_statistics.cpp
                  src/pal_statistics_macros.cpp src/registration_list.cpp)

ament_target_dependencies(${PROJECT_NAME} pal_statistics_msgs rclcpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBARIES})

# ##############################################################################
# Install ##
# ##############################################################################

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

# ##############################################################################
# Testing ##
# ##############################################################################

if(BUILD_TESTING)
  find_package(ament_cmake_gtest)
  find_package(ament_cmake_gmock)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(buffer-test test/gtest_buffer.cpp)
  target_link_libraries(buffer-test ${PROJECT_NAME})

  ament_add_gmock(pal_statistics-test test/gtest_pal_statistics.cpp)
  target_include_directories(
    pal_statistics-test PRIVATE src) # dirty hack for a forward declaration
  target_link_libraries(pal_statistics-test ${PROJECT_NAME})

  # TODO v-lopez python migration add_rostest(test/pal_statistics_py.test)
endif()

ament_export_dependencies(pal_statistics_msgs rclcpp)
ament_package()
