cmake_minimum_required(VERSION 3.8)
project(flying_pen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ 표준
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)  # 패키지 share 경로 찾으려고
find_package(crazyflie_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)   # ★ 추가
# 실행파일 추가
add_executable(data_logging src/data_logging.cpp)
ament_target_dependencies(data_logging
  rclcpp
  std_msgs
  geometry_msgs
  crazyflie_interfaces
  tf2
  tf2_ros
)

# 실행파일 추가 (새로운 data_player)
add_executable(data_player src/data_player.cpp)
ament_target_dependencies(data_player
  rclcpp
  std_msgs
  ament_index_cpp  
)

# 실행파일 추가 (새로운 rviz_visual)
add_executable(rviz_visual src/rviz_visual.cpp)
ament_target_dependencies(rviz_visual
  rclcpp
  std_msgs
  tf2_ros
  visualization_msgs  
)

add_executable(command_publisher src/command_publisher.cpp)
ament_target_dependencies(command_publisher
  rclcpp
  std_msgs
  crazyflie_interfaces
)

# Eigen3은 ament dependency가 아니라 include 디렉토리로 직접 추가
target_include_directories(data_logging PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_include_directories(rviz_visual PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(command_publisher ncurses)

# 설치 (ros2 run 할 수 있게)
install(TARGETS
  data_logging
  data_player
  rviz_visual
  command_publisher  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# bag 디렉토리 안의 csv도 패키지 share에 깔아두기
install(DIRECTORY bag/
  DESTINATION share/${PROJECT_NAME}/bag
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
