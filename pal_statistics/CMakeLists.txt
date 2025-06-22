cmake_minimum_required(VERSION 3.5.0)
project(pal_statistics)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/pal_statistics_utils.cpp src/pal_statistics.cpp
        src/pal_statistics_macros.cpp src/registration_list.cpp)

# ##############################################################################
# Testing ##
# ##############################################################################

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  ament_auto_add_gtest(buffer-test test/gtest_buffer.cpp)

  ament_add_gmock(pal_statistics-test test/gtest_pal_statistics.cpp
    TIMEOUT 180)

  target_include_directories(
    pal_statistics-test PRIVATE src) # dirty hack for a forward declaration
  target_link_libraries(pal_statistics-test ${PROJECT_NAME})

  # Add pal_statistics_py.test
  ament_add_pytest_test(pal_statistics_pytest test/test_pal_statistics.py)
endif()


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_auto_package()
