cmake_minimum_required(VERSION 3.10)

# 项目名称和版本
project(xhand_control_test VERSION 1.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

find_package(xhand_control HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../share REQUIRED)
set(CMAKE_INSTALL_RPATH ${CMAKE_CURRENT_SOURCE_DIR}/../lib)
# 添加头文件路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib
)
# 定义源文件
set(TEST_SOURCE_FILES
    src/ethercat_test.cpp
    src/serial_test.cpp
)

# 添加测试程序
add_executable(test_serial src/serial_test.cpp)
add_executable(test_ethercat src/ethercat_test.cpp)

# 链接库
target_link_libraries(test_serial PRIVATE xhand_control CURL::libcurl nlohmann_json::nlohmann_json)
target_link_options(test_serial PRIVATE -Wl,--disable-new-dtags)
target_link_libraries(test_ethercat PUBLIC xhand_control CURL::libcurl nlohmann_json::nlohmann_json)
target_link_options(test_ethercat PRIVATE -Wl,--disable-new-dtags)

