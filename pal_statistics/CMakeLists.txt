cmake_minimum_required(VERSION 3.10)
project(pal_statistics)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(DEPENDENCIES
  pal_statistics_msgs
  rclcpp
  rclcpp_lifecycle
)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

set(TARGET_DEPENDENCIES
  ${pal_statistics_msgs_TARGETS}
  rclcpp::rclcpp
  ${rclcpp_lifecycle_TARGETS}
)

if(WIN32)
  # set the same behavior for windows as it is on linux
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

  find_package(Boost QUIET COMPONENTS headers)
  if(NOT Boost_headers_FOUND)
    find_package(Boost REQUIRED)
  endif()
  list(APPEND DEPENDENCIES
    Boost
  )
  list(APPEND TARGET_DEPENDENCIES
    Boost::boost
  )
endif()

add_library(${PROJECT_NAME} SHARED
  src/pal_statistics_utils.cpp src/pal_statistics.cpp
  src/pal_statistics_macros.cpp src/registration_list.cpp)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${TARGET_DEPENDENCIES}
)


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

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
