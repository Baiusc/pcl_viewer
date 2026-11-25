cmake_minimum_required(VERSION 3.20)

# 找到Qt的测试模块

# 包含上级目录的变量
message(STATUS "QT_VERSION_MAJOR from parent scope: ${QT_VERSION_MAJOR}")

# 找到Qt的测试模块
if(QT_VERSION_MAJOR EQUAL 6)
    find_package(Qt6 REQUIRED COMPONENTS Test)
    message(STATUS "Using Qt6 Test module")
else()
    find_package(Qt5 REQUIRED COMPONENTS Test)
    message(STATUS "Using Qt5 Test module")
endif()

# 包含公共头文件
include_directories(
    ${YS_PRIVATE_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR} / tests
)

# 添加测试
add_executable(lidar_network_protocol_test
    lidar_network_protocol_test.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/network_protocol.cpp
)
target_include_directories(lidar_network_protocol_test PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(lidar_network_protocol_test PRIVATE Qt${QT_VERSION_MAJOR}::Network Qt${QT_VERSION_MAJOR}::Test)
add_test(NAME LidarNetworkProtocolTest COMMAND lidar_network_protocol_test)

add_executable(sn_management_test sn_management_test.cpp)
target_include_directories(sn_management_test PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(sn_management_test PRIVATE ys_scene Qt${QT_VERSION_MAJOR}::Test)
add_test(NAME SnManagementTest COMMAND sn_management_test)

add_executable(node_manager_test
    node_manager_test.h
    node_manager_test.cpp
)

# FL6532 RANGE(0x2E) 组帧器测试：合成 UDP 包验证组帧状态机，无需硬件
add_executable(fl6532_range_parser_test
    fl6532_range_parser_test.h
    fl6532_range_parser_test.cpp
)

# MF150 标定 runner 迁移到算法统一库(新版 API)的冒烟测试
add_executable(mf150_calib_runner_test
    mf150_calib_runner_test.h
    mf150_calib_runner_test.cpp
)

# 雷达型号 -> 属性配置 key 映射测试（纯函数，无需硬件）
add_executable(lidar_model_config_test
    lidar_model_config_test.h
    lidar_model_config_test.cpp
    ${CMAKE_SOURCE_DIR}/business/lidar_properties_config/lidar_model_config.h
    ${CMAKE_SOURCE_DIR}/business/lidar_properties_config/lidar_model_config.cpp
)

# 0x0302 系统固件版本解析测试：合成 200 字节载荷验证字段解析、64 位构建时间与追溯语义推导，无需硬件
add_executable(firmware_version_test
    firmware_version_test.h
    firmware_version_test.cpp
    ${CMAKE_SOURCE_DIR}/business/firmware_version/firmware_version_codec.h
    ${CMAKE_SOURCE_DIR}/business/firmware_version/firmware_version_codec.cpp
)

# CS96 Beta 连续直方图：完整度、56 SLOT 组帧及按 SLOT 灰度显示，无需硬件
add_executable(raw_histogram_test
    raw_histogram_test.h
    raw_histogram_test.cpp
)

# FL6532 Range LZ4 压缩链路测试：SDK 组帧 + LZ4F 解压 + 按 push 逆转置 + 算法点云输出，无需硬件
add_executable(fl6532_range_lz4_test
    fl6532_range_lz4_test.h
    fl6532_range_lz4_test.cpp
)

# CS96-gamma(1578B / 120 点每 SLOT) 包长识别与成帧判定测试：纯函数，无需硬件
add_executable(pointcloud_frame_policy_test
    pointcloud_frame_policy_test.h
    pointcloud_frame_policy_test.cpp
)

# 链接所需的库
target_link_libraries(node_manager_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGL
    Qt${QT_VERSION_MAJOR}::OpenGLWidgets
    Qt${QT_VERSION_MAJOR}::PrintSupport
    Qt${QT_VERSION_MAJOR}::Network
    ys_scene
    ys_lidar_sdk_static
    repair_ground_static
    crashpad::crashpad
    ${OPENGL_LIBRARIES}
    opengl32
)

target_link_libraries(fl6532_range_parser_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    Qt${QT_VERSION_MAJOR}::Widgets
    ys_scene
    ys_lidar_sdk_static
    repair_ground_static
    crashpad::crashpad
    ${OPENGL_LIBRARIES}
    opengl32
)

# 测试需构造 IOContext(io_context.h 在 USE_IOCP 下才提供完整定义)
target_compile_definitions(fl6532_range_parser_test PRIVATE USE_IOCP=1)
target_include_directories(fl6532_range_parser_test PRIVATE
    ${CMAKE_SOURCE_DIR}/you_see_sdk
    ${CMAKE_SOURCE_DIR}/you_see_sdk/you_see
)

target_link_libraries(mf150_calib_runner_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    Qt${QT_VERSION_MAJOR}::Widgets
    ys_scene
    ys_lidar_sdk_static
    repair_ground_static
    crashpad::crashpad
    ${OPENGL_LIBRARIES}
    opengl32
)

target_link_libraries(lidar_model_config_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    ys_lidar_sdk_static
)

# 被测转换函数需包含 lumomo_ui 的纯数据头（lumosy/buddies 是 lumomo_ui 的 include 根）
target_link_libraries(firmware_version_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    ys_lidar_sdk_static
)

target_include_directories(firmware_version_test PRIVATE
    ${CMAKE_SOURCE_DIR}/lumosy/buddies
)

target_link_libraries(raw_histogram_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    Qt${QT_VERSION_MAJOR}::Widgets
    ys_scene
    ys_lidar_sdk_static
    repair_ground_static
    crashpad::crashpad
    ${OPENGL_LIBRARIES}
    opengl32
)

# 测试需构造 IOContext(io_context.h 在 USE_IOCP 下才提供完整定义)
target_compile_definitions(raw_histogram_test PRIVATE USE_IOCP=1)
target_include_directories(raw_histogram_test PRIVATE
    ${CMAKE_SOURCE_DIR}/you_see_sdk
    ${CMAKE_SOURCE_DIR}/you_see_sdk/you_see
)

target_link_libraries(fl6532_range_lz4_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    Qt${QT_VERSION_MAJOR}::Widgets
    ys_scene
    ys_lidar_sdk_static
    repair_ground_static
    crashpad::crashpad
    lz4::lz4
    ${OPENGL_LIBRARIES}
    opengl32
)

# 测试需构造 IOContext(io_context.h 在 USE_IOCP 下才提供完整定义)
target_compile_definitions(fl6532_range_lz4_test PRIVATE USE_IOCP=1)
target_include_directories(fl6532_range_lz4_test PRIVATE
    ${CMAKE_SOURCE_DIR}/you_see_sdk
    ${CMAKE_SOURCE_DIR}/you_see_sdk/you_see
    ${CMAKE_SOURCE_DIR}/component
    ${CMAKE_SOURCE_DIR}/business
)

# 被测纯函数头 + SDK 协议常量/结构体(PointCloudNetPacket / CartesianPoint)
target_link_libraries(pointcloud_frame_policy_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    ys_lidar_sdk_static
)

target_include_directories(pointcloud_frame_policy_test PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/component
    ${CMAKE_SOURCE_DIR}/you_see_sdk
    ${CMAKE_SOURCE_DIR}/you_see_sdk/include
)

# 添加测试到CTest
add_executable(gray_image_test gray_image_test.cpp)
target_link_libraries(gray_image_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Network
    ys_scene
)
target_include_directories(gray_image_test PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/you_see_sdk
    ${CMAKE_SOURCE_DIR}/third_party
)
add_test(NAME GrayImageTest COMMAND gray_image_test)

# 接收器启动、广播状态合并和握手回归；只使用本机 UDP，无需雷达。
add_executable(lidar_connection_test lidar_connection_test.cpp)
target_link_libraries(lidar_connection_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    Qt${QT_VERSION_MAJOR}::Network
    ys_lidar_sdk_static
)
target_include_directories(lidar_connection_test PRIVATE ${CMAKE_SOURCE_DIR}/you_see_sdk)
add_test(NAME LidarConnectionTest COMMAND lidar_connection_test)
set_tests_properties(LidarConnectionTest PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)

add_executable(node_manager_state_test node_manager_state_test.cpp)
target_link_libraries(node_manager_state_test PRIVATE Qt${QT_VERSION_MAJOR}::Test ys_scene)
target_include_directories(node_manager_state_test PRIVATE ${CMAKE_SOURCE_DIR})
add_test(NAME NodeManagerStateTest COMMAND node_manager_state_test)
set_tests_properties(NodeManagerStateTest PROPERTIES TIMEOUT 15 ENVIRONMENT "QT_QPA_PLATFORM=offscreen")

add_test(NAME NodeManagerTest COMMAND node_manager_test)
add_test(NAME Fl6532RangeParserTest COMMAND fl6532_range_parser_test)
add_test(NAME Mf150CalibRunnerTest COMMAND mf150_calib_runner_test)
add_test(NAME LidarModelConfigTest COMMAND lidar_model_config_test)
add_test(NAME FirmwareVersionTest COMMAND firmware_version_test)
add_test(NAME RawHistogramTest COMMAND raw_histogram_test)
add_test(NAME Fl6532RangeLz4Test COMMAND fl6532_range_lz4_test)
add_test(NAME PointcloudFramePolicyTest COMMAND pointcloud_frame_policy_test)

# CS96 Gamma 原始点云：协议、乱序/重复/丢包/回绕、原始码值落盘与本机 UDP 保存全链路。
add_executable(cs96_gamma_raw_test cs96_gamma_raw_test.cpp)
target_link_libraries(cs96_gamma_raw_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    Qt${QT_VERSION_MAJOR}::Network
    ys_scene
)
target_include_directories(cs96_gamma_raw_test PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/component
    ${CMAKE_SOURCE_DIR}/you_see_sdk
)
add_test(NAME Cs96GammaRawTest COMMAND cs96_gamma_raw_test)
set_tests_properties(Cs96GammaRawTest PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)

# Gamma 直方图：30 包单 SLOT 组帧、120×512 显示/落盘与本机 UDP 连续采集全链路。
add_executable(cs96_gamma_histogram_test cs96_gamma_histogram_test.cpp)
target_link_libraries(cs96_gamma_histogram_test PRIVATE
    Qt${QT_VERSION_MAJOR}::Test
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Network
    ys_scene
)
target_include_directories(cs96_gamma_histogram_test PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/component
    ${CMAKE_SOURCE_DIR}/third_party
    ${CMAKE_SOURCE_DIR}/you_see_sdk
)
add_test(NAME Cs96GammaHistogramTest COMMAND cs96_gamma_histogram_test)
set_tests_properties(Cs96GammaHistogramTest PROPERTIES
    TIMEOUT 30 RUN_SERIAL TRUE ENVIRONMENT "QT_QPA_PLATFORM=offscreen")

# CPU rendering preparation and queued frame ownership; no window or LiDAR required.
add_executable(pointcloud_render_frame_test pointcloud_render_frame_test.cpp)
target_link_libraries(pointcloud_render_frame_test PRIVATE Qt${QT_VERSION_MAJOR}::Test ys_scene)
target_include_directories(pointcloud_render_frame_test PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/component
    ${CMAKE_SOURCE_DIR}/business
    ${CMAKE_SOURCE_DIR}/you_see_sdk
)
add_test(NAME PointcloudRenderFrameTest COMMAND pointcloud_render_frame_test)
set_tests_properties(PointcloudRenderFrameTest PROPERTIES TIMEOUT 30)

# Desktop OpenGL smoke test with synthetic points; no device discovery or hardware access.
add_executable(pointcloud_display_test pointcloud_display_test.cpp)
target_link_libraries(pointcloud_display_test PRIVATE Qt${QT_VERSION_MAJOR}::Test ys_scene)
target_include_directories(pointcloud_display_test PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/component
    ${CMAKE_SOURCE_DIR}/you_see_sdk
)
add_test(NAME PointcloudDisplayTest COMMAND pointcloud_display_test)
set_tests_properties(PointcloudDisplayTest PROPERTIES TIMEOUT 30 LABELS "gui;opengl")

add_executable(management_test management_test.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/management_protocol.cpp)
target_include_directories(management_test PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(management_test PRIVATE Qt${QT_VERSION_MAJOR}::Test)
target_compile_options(management_test PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
target_compile_definitions(management_test PRIVATE MANAGEMENT_EXAMPLES="${CMAKE_SOURCE_DIR}/docs/lidar-management-v1.examples.json")
add_test(NAME ManagementTest COMMAND management_test)

find_package(CURL REQUIRED)
add_executable(management_http_test management_http_test.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/management_protocol.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/management_client.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/management_client.h)
target_include_directories(management_http_test PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(management_http_test PRIVATE Qt${QT_VERSION_MAJOR}::Test Qt${QT_VERSION_MAJOR}::Network CURL::libcurl)
target_compile_options(management_http_test PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
target_compile_definitions(management_http_test PRIVATE MANAGEMENT_EXAMPLES="${CMAKE_SOURCE_DIR}/docs/lidar-management-v1.examples.json")
add_test(NAME ManagementHttpTest COMMAND management_http_test)
