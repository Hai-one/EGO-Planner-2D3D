cmake_minimum_required(VERSION 3.8)
project(ego_planner_real)

find_package(ament_cmake REQUIRED)

# 控制脚本
install(PROGRAMS
  scripts/ego_ctrl.sh
  DESTINATION lib/${PROJECT_NAME}
)

# launch / config / doc
install(
  DIRECTORY launch config doc
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
