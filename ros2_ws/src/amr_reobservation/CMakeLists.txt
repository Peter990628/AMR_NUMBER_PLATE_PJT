cmake_minimum_required(VERSION 3.16)
project(amr_reobservation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(scan_preprocessor src/scan_preprocessor.cpp)
target_compile_features(scan_preprocessor PUBLIC cxx_std_17)
target_include_directories(scan_preprocessor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(scan_preprocessor_node src/scan_preprocessor_node.cpp)
target_link_libraries(scan_preprocessor_node scan_preprocessor)
ament_target_dependencies(scan_preprocessor_node
  rclcpp rcl_interfaces sensor_msgs geometry_msgs visualization_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(scan_preprocessor PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(scan_preprocessor_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS scan_preprocessor EXPORT scan_preprocessorTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS scan_preprocessor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_preprocessor test/test_scan_preprocessor.cpp)
  target_link_libraries(test_scan_preprocessor scan_preprocessor)
endif()

ament_export_targets(scan_preprocessorTargets HAS_LIBRARY_TARGET)
ament_package()
