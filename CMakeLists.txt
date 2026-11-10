cmake_minimum_required(VERSION 3.16)
project(kid_rl LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(dynamixel_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

if(DEFINED ENV{ONNXRUNTIME_ROOT})
  set(_onnxruntime_default "$ENV{ONNXRUNTIME_ROOT}")
elseif(EXISTS "$ENV{HOME}/opt/onnxruntime/current/include/onnxruntime_cxx_api.h")
  # The deployment machine keeps the pinned SDK in the user's opt directory.
  set(_onnxruntime_default "$ENV{HOME}/opt/onnxruntime/current")
else()
  set(_onnxruntime_default "/opt/onnxruntime/current")
endif()
set(ONNXRUNTIME_ROOT "${_onnxruntime_default}" CACHE PATH
  "ONNX Runtime C/C++ SDK root")
# sudo 없이 설치한 경우를 위해 $HOME/opt 도 같이 본다. 다른 위치에 두었으면
# colcon build --cmake-args -DONNXRUNTIME_ROOT=<sdk 경로> 로 넘길 것.
# 파이썬 휠(site-packages/onnxruntime/capi) 안의 .so 는 헤더가 없어 쓸 수 없다.
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}/include" "$ENV{HOME}/opt/onnxruntime/current/include"
  REQUIRED)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS "${ONNXRUNTIME_ROOT}/lib" "$ENV{HOME}/opt/onnxruntime/current/lib"
  REQUIRED)

add_library(onnxruntime::onnxruntime SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")

add_library(${PROJECT_NAME} SHARED
  src/csv_logger.cpp
  src/imu.cpp
  src/kid_rl_controller.cpp
  src/policy_runner.cpp
  src/rx_missing_monitor.cpp
  src/motor_telemetry_monitor.cpp
  src/bus_timing_monitor.cpp
  src/external_commands.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp controller_interface dynamixel_interfaces geometry_msgs
  hardware_interface pluginlib rclcpp
  rclcpp_lifecycle realtime_tools std_msgs)
target_link_libraries(${PROJECT_NAME}
  "$<BUILD_INTERFACE:onnxruntime::onnxruntime>")
# BUILD_INTERFACE 로만 링크하면 IMPORTED 타깃의 디렉터리가 설치 RPATH 에 남지 않는다.
# 그러면 설치된 libkid_rl.so 에 RUNPATH 가 없어서 pluginlib 가
# "libonnxruntime.so.1: cannot open shared object file" 로 로드에 실패한다
# (ONNX Runtime 이 ldconfig 경로에 없는 경우).
get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE)

if(BUILD_TESTING)
  enable_testing()
  add_executable(policy_runner_test test/policy_runner_test.cpp)
  target_link_libraries(policy_runner_test PRIVATE ${PROJECT_NAME})
  # 배포 모델(위상 없음, 84)과 위상 모델(86) 둘 다 불러와 기댓값과 대조한다.
  add_test(NAME policy_runner_test COMMAND policy_runner_test
    "${CMAKE_CURRENT_SOURCE_DIR}/models/policy.onnx"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/data/policy.golden")
  add_test(NAME policy_runner_gait_phase_test COMMAND policy_runner_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/data/policy_gait_phase.onnx"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/data/policy_gait_phase.golden")
  # 위상 관측이 mjlab 학습 env 가 실제로 넣은 값과 같은 시각·같은 값인지.
  add_executable(gait_phase_test test/gait_phase_test.cpp)
  target_include_directories(gait_phase_test PRIVATE include)
  add_test(NAME gait_phase_test COMMAND gait_phase_test
    "${CMAKE_CURRENT_SOURCE_DIR}/test/data/gait_phase_mjlab.csv")
  # hurdle_vision 44 칸이 학습(tasks/hurdle.py)과 같은 순서·간격인지.
  add_executable(hurdle_vision_test test/hurdle_vision_test.cpp)
  target_include_directories(hurdle_vision_test PRIVATE include)
  add_test(NAME hurdle_vision_test COMMAND hurdle_vision_test)
  # 전압·PWM 진단: xacro 선언으로 켜지는지, PWM 부호 변환, CSV 열.
  add_executable(motor_telemetry_test test/motor_telemetry_test.cpp)
  target_link_libraries(motor_telemetry_test PRIVATE ${PROJECT_NAME})
  add_test(NAME motor_telemetry_test COMMAND motor_telemetry_test
    "${CMAKE_CURRENT_BINARY_DIR}/motor_telemetry_test.csv")
endif()

pluginlib_export_plugin_description_file(
  controller_interface kid_rl_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config description docs launch models test DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/kid_rl_policy.sh
  DESTINATION lib/${PROJECT_NAME})
install(FILES kid_rl_plugin.xml README.md
  DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp controller_interface dynamixel_interfaces geometry_msgs
  hardware_interface pluginlib rclcpp
  rclcpp_lifecycle realtime_tools std_msgs)
ament_package()
