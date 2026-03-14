cmake_minimum_required(VERSION 3.8)
project(bot_localization) # 替换为你的包名，例如 bot_localization

# 找到必要的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# 声明对 robot_localization 的依赖
find_package(robot_localization REQUIRED)

# 如果有自定义节点或库，在这里添加
# add_executable(your_node src/your_node.cpp)
# ament_target_dependencies(your_node rclcpp robot_localization)

# 安装 launch 文件和配置文件
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 声明对 robot_localization 的依赖，以便下游包能找到它
ament_export_dependencies(robot_localization)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 根据需要设置 linter
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()