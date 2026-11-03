cmake_minimum_required(VERSION 3.15)
project(cloud_viewer LANGUAGES C CXX)

# ------------------------------------------------------------------------------
# 基础设置
#
# 注意：Lite 主程序与旧调试程序分离为不同 target。
# 新 target 固定 C++17，不使用 -march=native，保证可移植（见设计文档 2.2）。
# ------------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(ViewerDependencies)

# ------------------------------------------------------------------------------
# 构建选项
# ------------------------------------------------------------------------------

option(CLOUD_VIEWER_BUILD_LEGACY "构建旧调试程序 PCL_VIEWER_LEGACY（默认关闭）" OFF)
option(CLOUD_VIEWER_BUILD_TESTS  "构建单元/集成测试" ON)

# 在线 SDK 后端开关：未启用时工厂返回 BackendUnavailable，不伪装成功。
option(CLOUD_VIEWER_WITH_ALPHA_SDK  "启用 alpha 转镜雷达 SDK 后端" OFF)
option(CLOUD_VIEWER_WITH_BETA_SDK   "启用 beta 薄款转镜雷达 SDK 后端" OFF)
option(CLOUD_VIEWER_WITH_FL6031_SDK "启用 FL6031 SDK 后端" OFF)
option(CLOUD_VIEWER_WITH_FL6532_SDK "启用 FL6532 SDK 后端" OFF)

# ------------------------------------------------------------------------------
# cv_core：业务数据真值层（PointData / PointMatrix / 帧模型）
#
# 该层不依赖 PCL、VTK、SDK 与 UI。point_type.cpp 目前仍承载 Converter 与
# 6532 标定实现，二者均为纯标准库代码，因此直接复用，不在 M1 重写。
# ------------------------------------------------------------------------------

add_library(cv_core STATIC
    src/cloud_viewer/core/CloudFrame.cpp
    src/point_type.cpp
)
target_include_directories(cv_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(cv_core PUBLIC cxx_std_17)

# ------------------------------------------------------------------------------
# cv_io：离线解析层（嗅探 / 帧索引 / 按需解码）
#
# 转镜与固态 TXT 复用现有 pulse_data_processor.cpp；
# CSV / PCD 解析在本层实现，避免牵连缺失 Boost 依赖的 PCL IO / Visualization。
# ------------------------------------------------------------------------------

add_library(cv_io STATIC
    src/cloud_viewer/io/Loaders.cpp
    src/pulse_data_processor.cpp
)
target_include_directories(cv_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cv_io PUBLIC cv_core)

# ------------------------------------------------------------------------------
# cv_render：显示派生层（PointMatrix -> PCL 显示云 + 原始索引映射）
# ------------------------------------------------------------------------------

add_library(cv_render STATIC
    src/cloud_viewer/render/CloudRenderBridge.cpp
    src/cloud_viewer/render/PclVtkRenderer.cpp
    src/cloud_viewer/render/UiOverlay.cpp
    src/cloud_viewer/color/ColorMapper.cpp
    src/cloud_viewer/scene/SceneGraph.cpp
)
target_include_directories(cv_render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cv_render PUBLIC cv_core viewer_deps)

# ------------------------------------------------------------------------------
# cv_stream：在线后端注册（真实 SDK 未启用时返回 BackendUnavailable）
# ------------------------------------------------------------------------------

add_library(cv_stream STATIC
    src/cloud_viewer/preprocess/Cs96BetaCalibration.cpp
    src/cloud_viewer/stream/BetaUdpStream.cpp
    src/cloud_viewer/stream/DeviceDiscovery.cpp
    src/cloud_viewer/stream/RadarStream.cpp
    src/cloud_viewer/stream/UdpRadarStream.cpp
)
target_include_directories(cv_stream PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# 只链接 RANGE 核心源文件，不引入算法工程的 main、离线标定器和全局构建选项。
set(CLOUD_VIEWER_RANGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../algo_solid_lidar" CACHE PATH "RANGE API 工程目录")
find_path(CV_LZ4_INCLUDE lz4frame.h)
find_library(CV_LZ4_LIBRARY NAMES lz4)
if(NOT CV_LZ4_INCLUDE OR NOT CV_LZ4_LIBRARY)
    message(FATAL_ERROR "在线 RANGE LZ4 解析需要 lz4frame.h 和 liblz4")
endif()
add_library(cv_range6532 STATIC
    ${CLOUD_VIEWER_RANGE_ROOT}/src/range6532_pipeline.cpp
    ${CLOUD_VIEWER_RANGE_ROOT}/src/range6532_protocol.cpp)
target_include_directories(cv_range6532 PUBLIC ${CLOUD_VIEWER_RANGE_ROOT}/include)
target_sources(cv_stream PRIVATE src/cloud_viewer/stream/RangeStreamDecoder.cpp
    src/cloud_viewer/stream/RadarControl.cpp)
target_include_directories(cv_stream PRIVATE ${CV_LZ4_INCLUDE})
find_package(Threads REQUIRED)
target_link_libraries(cv_stream PUBLIC cv_core cv_range6532 ${CV_LZ4_LIBRARY} Threads::Threads)
if(WIN32)
    target_link_libraries(cv_stream PUBLIC ws2_32)
endif()

foreach(_sdk_option ALPHA_SDK BETA_SDK FL6031_SDK FL6532_SDK)
    if(CLOUD_VIEWER_WITH_${_sdk_option})
        target_compile_definitions(cv_stream PUBLIC CLOUD_VIEWER_WITH_${_sdk_option})
    endif()
endforeach()

# ------------------------------------------------------------------------------
# cv_preprocess：可选标定模块（默认编译，但运行时必须显式配置才应用）
# ------------------------------------------------------------------------------

add_library(cv_preprocess STATIC
    src/cloud_viewer/preprocess/FramePreprocessor.cpp
    src/cloud_viewer/preprocess/OnlineCalibration.cpp
)
target_include_directories(cv_preprocess PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cv_preprocess PUBLIC cv_core)

# ------------------------------------------------------------------------------
# cv_platform：路径与原生文件对话框
# ------------------------------------------------------------------------------

add_library(cv_platform STATIC
    src/cloud_viewer/platform/AppSettings.cpp
    src/cloud_viewer/platform/FileDialog.cpp
    src/cloud_viewer/platform/NetworkSetup.cpp
    src/cloud_viewer/platform/RadarLauncher.cpp
)
target_include_directories(cv_platform PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cv_platform PUBLIC cv_core)
if(WIN32)
    target_link_libraries(cv_platform PUBLIC comdlg32 shell32 ole32)
endif()

# ------------------------------------------------------------------------------
# cv_app：应用控制器（数据源、帧切换、着色、选点、HUD）
# ------------------------------------------------------------------------------

add_library(cv_app STATIC
    src/cloud_viewer/app/ViewerApp.cpp
)
target_include_directories(cv_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(cv_app PUBLIC cv_render cv_io cv_platform cv_core cv_stream cv_preprocess viewer_deps)

# ------------------------------------------------------------------------------
# cloud_viewer：Lite 主程序
# ------------------------------------------------------------------------------

add_executable(cloud_viewer
    src/cloud_viewer_main.cpp
)
target_link_libraries(cloud_viewer PRIVATE
    cv_app cv_render cv_io cv_stream cv_preprocess cv_core viewer_deps)

if(WIN32)
    # 将运行时 DLL 复制到输出目录，避免手动配置 PATH。
    add_custom_command(TARGET cloud_viewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_RUNTIME_DLLS:cloud_viewer> $<TARGET_FILE_DIR:cloud_viewer>
        COMMAND_EXPAND_LISTS)
endif()

# ------------------------------------------------------------------------------
# 旧调试程序：默认不构建，保证实验代码退出 Lite 构建
# ------------------------------------------------------------------------------

if(CLOUD_VIEWER_BUILD_LEGACY)
    file(GLOB LEGACY_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
    add_executable(PCL_VIEWER_LEGACY ${LEGACY_SOURCES})
    target_include_directories(PCL_VIEWER_LEGACY PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(PCL_VIEWER_LEGACY PRIVATE viewer_deps)
    set_target_properties(PCL_VIEWER_LEGACY PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)
endif()

# ------------------------------------------------------------------------------
# 测试
# ------------------------------------------------------------------------------

if(CLOUD_VIEWER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
