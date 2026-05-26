cmake_minimum_required(VERSION 3.28)
project(soar_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  std_srvs
  ament_index_cpp
)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(SQLite3 REQUIRED)

set(THIS_PACKAGE_LINK_DEPENDS
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ament_index_cpp::ament_index_cpp
)

include(FetchContent)

FetchContent_Declare(
    soar
    GIT_REPOSITORY https://github.com/SoarGroup/Soar.git
    GIT_TAG releases/9.6.5
)

FetchContent_MakeAvailable(soar)

add_library(soar_ros src/soar_ros.cpp src/SoarRunner.cpp)
target_include_directories(soar_ros
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(soar_ros
    soar_lib
    ${THIS_PACKAGE_LINK_DEPENDS})
set_target_properties(soar_ros
  PROPERTIES CMAKE_CXX_STANDARD 20)

add_executable(start src/main.cpp)
target_link_libraries(start soar_ros ${THIS_PACKAGE_LINK_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(example_interfaces REQUIRED)
  find_package(launch_testing_ament_cmake)

  add_executable(test_input_output test/pubsub/test_input_output.cpp)
  target_link_libraries(test_input_output soar_ros ${THIS_PACKAGE_LINK_DEPENDS})

  add_executable(test_service test/service/test_service.cpp)
  target_link_libraries(test_service soar_ros ${THIS_PACKAGE_LINK_DEPENDS} ${example_interfaces_TARGETS})

  add_executable(test_client test/client/test_client.cpp)
  target_link_libraries(test_client soar_ros ${THIS_PACKAGE_LINK_DEPENDS} ${example_interfaces_TARGETS})

  add_executable(test_action_client test/action_client/test_action_client.cpp)
  target_link_libraries(test_action_client soar_ros ${THIS_PACKAGE_LINK_DEPENDS} ${example_interfaces_TARGETS})

  add_executable(test_multi_agent test/multi_agent/test_multi_agent.cpp)
  target_link_libraries(test_multi_agent soar_ros ${THIS_PACKAGE_LINK_DEPENDS})

  add_executable(test_shared_input test/multi_agent/test_shared_input.cpp)
  target_link_libraries(test_shared_input soar_ros ${THIS_PACKAGE_LINK_DEPENDS})

  ament_add_gtest(test_std_msgs_converters
    test/msg_converters/test_std_msgs_converters.cpp)
  target_include_directories(test_std_msgs_converters PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_std_msgs_converters
    soar_lib
    ${std_msgs_TARGETS})

  ament_add_gtest(test_geometry_msgs_converters
    test/msg_converters/test_geometry_msgs_converters.cpp)
  target_include_directories(test_geometry_msgs_converters PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_geometry_msgs_converters
    soar_lib
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS})

  find_package(builtin_interfaces REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(tf2_msgs REQUIRED)
  find_package(action_msgs REQUIRED)
  find_package(unique_identifier_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)

  ament_add_gtest(test_builtin_interfaces_converters
    test/msg_converters/test_builtin_interfaces_converters.cpp)
  target_include_directories(test_builtin_interfaces_converters PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_builtin_interfaces_converters
    soar_lib
    ${builtin_interfaces_TARGETS})

  ament_add_gtest(test_sensor_msgs_converters
    test/msg_converters/test_sensor_msgs_converters.cpp)
  target_include_directories(test_sensor_msgs_converters PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_sensor_msgs_converters
    soar_lib
    ${builtin_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS})

  ament_add_gtest(test_nav_msgs_converters
    test/msg_converters/test_nav_msgs_converters.cpp)
  target_include_directories(test_nav_msgs_converters PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_nav_msgs_converters
    soar_lib
    ${builtin_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS})

  ament_add_gtest(test_misc_msgs_converters
    test/msg_converters/test_misc_msgs_converters.cpp)
  target_include_directories(test_misc_msgs_converters PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_misc_msgs_converters
    soar_lib
    ${builtin_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
    ${unique_identifier_msgs_TARGETS}
    ${action_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${trajectory_msgs_TARGETS})

  ament_add_gtest(test_action_msgs_converters
    test/msg_converters/test_action_msgs_converters.cpp)
  target_include_directories(test_action_msgs_converters PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_action_msgs_converters
    soar_lib
    ${builtin_interfaces_TARGETS}
    ${unique_identifier_msgs_TARGETS}
    ${action_msgs_TARGETS})

  ament_add_gtest(test_soar_roundtrip
    test/msg_converters/test_soar_roundtrip.cpp)
  target_include_directories(test_soar_roundtrip PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_soar_roundtrip
    soar_lib
    ${builtin_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${nav_msgs_TARGETS})

  add_launch_test(test/pubsub/test_launch_input_output.py)
  add_launch_test(test/service/test_launch_service.py)
  add_launch_test(test/client/test_launch_client.py)
  add_launch_test(test/action_client/test_launch_action_client.py)
  add_launch_test(test/multi_agent/test_launch_multi_agent.py)
  add_launch_test(test/multi_agent/test_launch_shared_input.py)

  install(TARGETS test_input_output test_service test_client test_action_client test_multi_agent test_shared_input
      DESTINATION lib/${PROJECT_NAME})
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS start
    DESTINATION lib/${PROJECT_NAME})

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
