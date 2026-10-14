cmake_minimum_required(VERSION 3.8)
project(xhand_control_ros2)

# set(CMAKE_BUILD_TYPE Debug)

# 设置 C++14 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 找到所需的依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xhand_control_interfaces REQUIRED)
find_package(xhand_control HINTS ${CMAKE_CURRENT_SOURCE_DIR}/xhand_control/share REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

# 包含目录
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/xhand_control/include
)

# 添加可执行文件
add_executable(${PROJECT_NAME}_node
  src/xhand_control_ros2_node.cpp
  src/xhand_control_ros2/xhand_control_ros2.cpp
)

# 连接依赖库
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  xhand_control_interfaces
  CURL
  nlohmann_json
)

# 包含其他目录
target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/xhand_control/include/
)

# 链接库
target_link_libraries(
  ${PROJECT_NAME}_node
  xhand_control
)

set(CMAKE_BUILD_RPATH "${CMAKE_INSTALL_PREFIX}/lib:/opt/ros/$ENV{ROS_DISTRO}/lib")

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:/opt/ros/$ENV{ROS_DISTRO}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 设置 RPATH 确保共享库被正确加载
set(LIBRARY_PATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")
set_target_properties(${PROJECT_NAME}_node PROPERTIES
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}:${CMAKE_INSTALL_PREFIX}/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# 安装可执行文件
install(TARGETS 
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装共享库文件
install(DIRECTORY xhand_control/lib/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.so"
  PATTERN ".svn" EXCLUDE
)

# 安装配置和启动文件
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 配置测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
