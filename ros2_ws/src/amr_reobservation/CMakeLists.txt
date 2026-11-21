cmake_minimum_required(VERSION 3.16)
project(amr_reobservation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)

set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/onnxruntime-linux-x64-1.23.2"
  CACHE PATH "ONNX Runtime C++ distribution root")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h HINTS "${ONNXRUNTIME_ROOT}/include" REQUIRED)
find_library(ONNXRUNTIME_LIBRARY onnxruntime HINTS "${ONNXRUNTIME_ROOT}/lib" REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleSideEstimate.msg"
  "msg/ClusterFitDiagnostics.msg"
  "msg/VehicleSideFitDiagnostics.msg"
  "msg/PlateObservation.msg"
  "msg/ObservationCandidates.msg"
  "msg/RecoveryRequest.msg"
  "msg/RecoveryPreview.msg"
  "msg/PlateWindowSample.msg"
  "srv/InspectPlateFrame.srv"
  "srv/ReadPlateCrop.srv"
  "srv/AssessPlateWindow.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(scan_preprocessor src/scan_preprocessor.cpp)
target_compile_features(scan_preprocessor PUBLIC cxx_std_17)
target_include_directories(scan_preprocessor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(scan_preprocessor_node src/scan_preprocessor_node.cpp)
target_link_libraries(scan_preprocessor_node scan_preprocessor)
ament_target_dependencies(scan_preprocessor_node
  rclcpp rcl_interfaces sensor_msgs geometry_msgs visualization_msgs)

add_library(vehicle_side_estimator src/vehicle_side_estimator.cpp)
target_link_libraries(vehicle_side_estimator PUBLIC scan_preprocessor Eigen3::Eigen)

add_executable(vehicle_side_estimator_node src/vehicle_side_estimator_node.cpp)
target_link_libraries(vehicle_side_estimator_node vehicle_side_estimator "${cpp_typesupport_target}")
ament_target_dependencies(vehicle_side_estimator_node
  rclcpp rcl_interfaces sensor_msgs std_msgs geometry_msgs visualization_msgs)

add_library(plate_observation src/plate_observation.cpp)
target_compile_features(plate_observation PUBLIC cxx_std_17)
target_include_directories(plate_observation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
add_library(plate_detector SHARED src/plate_detector.cpp)
target_link_libraries(plate_detector PUBLIC plate_observation ${OpenCV_LIBS} PRIVATE ${ONNXRUNTIME_LIBRARY})
set_target_properties(plate_observation PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(plate_detector PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(plate_detector SYSTEM PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
set_target_properties(plate_detector PROPERTIES INSTALL_RPATH "$ORIGIN")
add_executable(plate_observation_node src/plate_observation_node.cpp)
target_link_libraries(plate_observation_node plate_detector "${cpp_typesupport_target}")
ament_target_dependencies(plate_observation_node rclcpp rcl_interfaces sensor_msgs std_msgs cv_bridge)
set_target_properties(plate_observation_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")
add_executable(plate_window_decision_node src/plate_window_decision_node.cpp)
target_link_libraries(plate_window_decision_node "${cpp_typesupport_target}")
ament_target_dependencies(plate_window_decision_node rclcpp)
add_executable(plate_image_probe src/plate_image_probe.cpp)
target_link_libraries(plate_image_probe plate_detector)
set_target_properties(plate_image_probe PROPERTIES INSTALL_RPATH "$ORIGIN/..")

add_library(observation_candidates src/observation_candidates.cpp)
target_compile_features(observation_candidates PUBLIC cxx_std_17)
target_include_directories(observation_candidates PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
add_executable(observation_candidates_node src/observation_candidates_node.cpp)
target_link_libraries(observation_candidates_node observation_candidates "${cpp_typesupport_target}")
ament_target_dependencies(observation_candidates_node rclcpp rcl_interfaces geometry_msgs std_msgs visualization_msgs)

add_library(candidate_stability src/candidate_stability.cpp)
target_compile_features(candidate_stability PUBLIC cxx_std_17)
target_include_directories(candidate_stability PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
add_executable(observation_candidate_stabilizer_node src/observation_candidate_stabilizer_node.cpp)
target_link_libraries(observation_candidate_stabilizer_node candidate_stability "${cpp_typesupport_target}")
ament_target_dependencies(observation_candidate_stabilizer_node
  rclcpp rcl_interfaces geometry_msgs std_msgs visualization_msgs tf2 tf2_ros)

add_library(recovery_preview src/recovery_preview.cpp)
target_compile_features(recovery_preview PUBLIC cxx_std_17)
target_include_directories(recovery_preview PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
add_library(recovery_contour src/recovery_contour.cpp)
target_compile_features(recovery_contour PUBLIC cxx_std_17)
target_include_directories(recovery_contour PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
add_executable(recovery_preview_node src/recovery_preview_node.cpp)
target_link_libraries(recovery_preview_node recovery_preview recovery_contour "${cpp_typesupport_target}")
ament_target_dependencies(recovery_preview_node
  rclcpp rcl_interfaces sensor_msgs geometry_msgs std_msgs visualization_msgs tf2 tf2_ros)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(scan_preprocessor PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(scan_preprocessor_node PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(vehicle_side_estimator PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(vehicle_side_estimator_node PRIVATE -Wall -Wextra -Wpedantic)
  foreach(target plate_observation plate_detector plate_observation_node plate_image_probe observation_candidates observation_candidates_node candidate_stability observation_candidate_stabilizer_node recovery_preview recovery_contour recovery_preview_node)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endforeach()
endif()

install(TARGETS scan_preprocessor vehicle_side_estimator plate_observation observation_candidates candidate_stability recovery_preview recovery_contour EXPORT scan_preprocessorTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS plate_detector LIBRARY DESTINATION lib)
install(DIRECTORY "${ONNXRUNTIME_ROOT}/lib/" DESTINATION lib FILES_MATCHING PATTERN "libonnxruntime.so*")
install(FILES "${ONNXRUNTIME_ROOT}/LICENSE" "${ONNXRUNTIME_ROOT}/ThirdPartyNotices.txt"
  DESTINATION share/${PROJECT_NAME}/third_party/onnxruntime)
install(TARGETS scan_preprocessor_node vehicle_side_estimator_node plate_observation_node plate_window_decision_node plate_image_probe observation_candidates_node observation_candidate_stabilizer_node recovery_preview_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/mock_recovery_request.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_preprocessor test/test_scan_preprocessor.cpp)
  target_link_libraries(test_scan_preprocessor scan_preprocessor)
  ament_add_gtest(test_vehicle_side_estimator test/test_vehicle_side_estimator.cpp)
  target_link_libraries(test_vehicle_side_estimator vehicle_side_estimator)
  ament_add_gtest(test_plate_observation test/test_plate_observation.cpp)
  target_link_libraries(test_plate_observation plate_observation)
  ament_add_gtest(test_plate_detector test/test_plate_detector.cpp)
  target_link_libraries(test_plate_detector plate_detector)
  ament_add_gtest(test_observation_candidates test/test_observation_candidates.cpp)
  target_link_libraries(test_observation_candidates observation_candidates)
  ament_add_gtest(test_candidate_stability test/test_candidate_stability.cpp)
  target_link_libraries(test_candidate_stability candidate_stability)
  ament_add_gtest(test_recovery_preview test/test_recovery_preview.cpp)
  target_link_libraries(test_recovery_preview recovery_preview)
  ament_add_gtest(test_recovery_contour test/test_recovery_contour.cpp)
  target_link_libraries(test_recovery_contour recovery_contour)
endif()

ament_export_targets(scan_preprocessorTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rosidl_default_runtime)
ament_package()
