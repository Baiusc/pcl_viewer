cmake_minimum_required(VERSION 3.15)
project(PCL_VIEWER)

### ---------------- 环境配置 ------------------

set(CMAKE_INSTALL_PREFIX "../install" CACHE PATH "安装根目录" FORCE) # 设置install目录
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/") # 添加额外的 CMake 模块
include(CMakeSetCompilerOptions) # 导入多平台编译设置
include(CMakeLocalCompile) # 导入本地环境变量
include(CMakePolicies) # 导入 cmake 策略
include(CMakeInclude)
include(Install)

### ---------------- 寻找 PCL 库 ------------------

find_package(PCL REQUIRED)
if(PCL_FOUND)
    message("=============================================")
    message("PCL_DIR: ${PCL_DIR}")
    message("PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
    message("PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
    message("=============================================")
    # 链接
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    include_directories(${PCL_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "PCL not found. Please specify the PCL installation path manually.")
endif()

### ---------------- 寻找 VTK 库 ------------------

find_package(VTK REQUIRED)
if(VTK_FOUND)
    message("VTK_DIR: ${VTK_DIR}")
    message("VTK_USE_FILE: ${VTK_USE_FILE}")
else()
    message(FATAL_ERROR "VTK not found. Please specify the VTK installation path manually.")
endif()

### ---------------- 本项目设置 ------------------

# 头文件和源文件列表
file(GLOB INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# 设置头文件分组（适用于 Visual Studio）
source_group("Header Files" FILES ${INCLUDE})

# 创建可执行文件
add_executable(${PROJECT_NAME})

# 添加源文件和头文件目录
target_sources(${PROJECT_NAME} PRIVATE ${SOURCE} ${INCLUDE})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 链接 PCL 和其他库
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

### ---------------- 其它 ------------------

# 将 DLL 复制到运行时输出目录（仅适用于 Windows 且动态库/可执行文件）
if(WIN32)
    # 获取目标的类型属性
    get_target_property(TARGET_TYPE ${PROJECT_NAME} TYPE)
    # 如果目标类型不是 STATIC_LIBRARY，则添加自定义命令
    if(NOT TARGET_TYPE STREQUAL "STATIC_LIBRARY")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}> $<TARGET_FILE_DIR:${PROJECT_NAME}>
            COMMAND_EXPAND_LISTS)
    else()
        message(STATUS "${PROJECT_NAME} is a static library; skipping runtime DLL copy.")
    endif()
endif()
# 打印编译相关的变量值
message(STATUS "==============Build Type: ${CMAKE_BUILD_TYPE} ==============")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "CMAKE_EXE_LINKER_FLAGS_DEBUG: ${CMAKE_EXE_LINKER_FLAGS_DEBUG}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "CMAKE_EXE_LINKER_FLAGS_RELEASE: ${CMAKE_EXE_LINKER_FLAGS_RELEASE}")