cmake_minimum_required(VERSION 3.5.0)
project(pal_statistics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED 
	src/pal_statistics_utils.cpp src/pal_statistics.cpp
	src/pal_statistics_macros.cpp src/registration_list.cpp)

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

  #  ament_add_gmock(pal_statistics-test test/gtest_pal_statistics.cpp)
  #  target_include_directories(
  #    pal_statistics-test PRIVATE src) # dirty hack for a forward declaration
  #  target_link_libraries(pal_statistics-test ${PROJECT_NAME})

  # TODO v-lopez python migration add_rostest(test/pal_statistics_py.test)
endif()

ament_auto_package()
