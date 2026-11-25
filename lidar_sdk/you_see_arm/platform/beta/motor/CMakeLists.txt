cmake_minimum_required(VERSION 3.21)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(beta_motor C)
endif()

add_executable(demo_i2c_motor demo_i2c_motor.c)
target_compile_features(demo_i2c_motor PRIVATE c_std_11)
target_compile_options(demo_i2c_motor PRIVATE -Wall -Wextra -Werror)
set_target_properties(demo_i2c_motor PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/platform/bin")

# 显式安装平台组件，避免普通应用打包顺带安装板级程序。
install(TARGETS demo_i2c_motor
    RUNTIME DESTINATION usr/libexec/lidar/platform
    COMPONENT lidar_platform EXCLUDE_FROM_ALL)
