cmake_minimum_required(VERSION 3.8)
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
  std_srvs
  ament_index_cpp
  SQLite3
)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


include(FetchContent)

FetchContent_Declare(
    soar
    GIT_REPOSITORY https://github.com/SoarGroup/Soar.git
    GIT_TAG releases/9.6.4
)

FetchContent_MakeAvailable(soar)

add_library(soar_ros src/soar_ros.cpp src/SoarRunner.cpp)
target_include_directories(soar_ros
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(soar_ros rclcpp rclcpp_action std_msgs std_srvs ament_index_cpp)
target_link_libraries(soar_ros soar_lib)
set_target_properties(soar_ros
  PROPERTIES CMAKE_CXX_STANDARD 20)

add_executable(start src/main.cpp)
ament_target_dependencies(start ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(start soar_ros)

if(BUILD_TESTING)
  find_package(example_interfaces REQUIRED)
  find_package(launch_testing_ament_cmake)

  add_executable(test_input_output test/pubsub/test_input_output.cpp)
  ament_target_dependencies(test_input_output ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(test_input_output soar_ros)

  add_executable(test_service test/service/test_service.cpp)
  ament_target_dependencies(test_service ${THIS_PACKAGE_INCLUDE_DEPENDS} example_interfaces)
  target_link_libraries(test_service soar_ros)

  add_executable(test_client test/client/test_client.cpp)
  ament_target_dependencies(test_client ${THIS_PACKAGE_INCLUDE_DEPENDS} example_interfaces)
  target_link_libraries(test_client soar_ros)

  add_executable(test_action_client test/action_client/test_action_client.cpp)
  ament_target_dependencies(test_action_client ${THIS_PACKAGE_INCLUDE_DEPENDS} example_interfaces)
  target_link_libraries(test_action_client soar_ros)

  add_launch_test(test/pubsub/test_launch_input_output.py)
  add_launch_test(test/service/test_launch_service.py)
  add_launch_test(test/client/test_launch_client.py)
  add_launch_test(test/action_client/test_launch_action_client.py)

  install(TARGETS test_input_output test_service test_client test_action_client
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
