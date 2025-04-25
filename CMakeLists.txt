cmake_minimum_required(VERSION 3.8)
project(soar_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  std_srvs
  ament_index_cpp
  SQLite3
)

find_package(example_interfaces REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


include(FetchContent)

FetchContent_Declare(
    soar
    GIT_REPOSITORY https://github.com/moschmdt/Soar.git
    GIT_TAG cmake
)

FetchContent_MakeAvailable(soar)

add_library(soar_ros src/soar_ros.cpp src/SoarRunner.cpp)
target_include_directories(soar_ros
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(soar_ros rclcpp std_msgs std_srvs ament_index_cpp)
target_link_libraries(soar_ros soar_lib)
set_target_properties(soar_ros
  PROPERTIES CMAKE_CXX_STANDARD 20)

add_executable(test_example test/test_soar_ros.cpp)
ament_target_dependencies(test_example ${THIS_PACKAGE_INCLUDE_DEPENDS} example_interfaces)
target_link_libraries(test_example soar_ros)
set_target_properties(test_example
  PROPERTIES CMAKE_CXX_STANDARD 20)


if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/test_launch.py)
  # add_launch_test(test/test_launch_client.py)
endif()

install(TARGETS test_example
    DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS soar_ros soar_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  Soar/
  DESTINATION share/${PROJECT_NAME}/Soar
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
