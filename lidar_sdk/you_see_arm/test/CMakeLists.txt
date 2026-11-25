# 最低的CMake版本要求
cmake_minimum_required(VERSION 3.10)

# 设置项目名称和使用的编程语言
project(YouSeeArmTest C CXX)

# 启用CTest测试工具，这样可以使用add_test命令添加测试
enable_testing()

# 寻找GTest库
find_package(GTest REQUIRED)

if(GTest_FOUND)
    message("GTest library found")
else()
    message(FATAL_ERROR "Cannot find GTest library")
endif()

# 设置一些变量，表示你的测试目标的名字
set(BASE_TEST base_test)
set(MEMORY_POOL_TEST memory_pool_test)
set(NETWORK_TEST network_test)

# 创建可执行文件目标
add_executable(${BASE_TEST} test_core_ys_base.cpp ../core/ys_base.c ../third_party/log.c/log.c)
add_executable(${MEMORY_POOL_TEST} test_core_ys_memory_pool.cpp ../core/ys_base.c ../core/ys_memory_pool.c ../third_party/log.c/log.c)
add_executable(${NETWORK_TEST} test_core_ys_network.cpp ../core/ys_core_network.c ../third_party/log.c/log.c)

# 链接GTest库到你的测试目标，使得你的测试目标可以使用GTest库
# PRIVATE意味着库将被链接到目标项目，但不会被传递到其他依赖此目标的项目
target_link_libraries(${BASE_TEST} PRIVATE GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)
target_link_libraries(${MEMORY_POOL_TEST} PRIVATE GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)
target_link_libraries(${NETWORK_TEST} PRIVATE GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main)

# 添加一个CTest测试
add_test(NAME base_test COMMAND base_test)
add_test(NAME memory_pool_test COMMAND memory_pool_test)
add_test(NAME network_test COMMAND network_test)
