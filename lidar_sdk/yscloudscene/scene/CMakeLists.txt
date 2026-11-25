# ys_scene 库的CMakeLists.txt

# 设置包含目录
set(YS_PRIVATE_INCLUDE_DIR 
    ${CMAKE_SOURCE_DIR}/you_see_sdk
    ${CMAKE_SOURCE_DIR}/business
    ${CMAKE_SOURCE_DIR}/component
    ${CMAKE_SOURCE_DIR}/algorithm
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/third_party/
    ${CMAKE_SOURCE_DIR}/third_party/qt-toast/src
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/matrix_lidar_sdk
)

include_directories(${YS_PRIVATE_INCLUDE_DIR})

set(TRANSLATIONS_DIR ${CMAKE_SOURCE_DIR}/translations)  # 上级目录的translations
set(QM_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/sources/Internationalization)
set(TS_FILES 
    ${TRANSLATIONS_DIR}/YSCloudScene_zh_CN.ts
    ${TRANSLATIONS_DIR}/YSCloudScene_en_US.ts)

# 输出路径信息
message(STATUS "============ Translation Configuration ==========")
message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message(STATUS "TRANSLATIONS_DIR: ${TRANSLATIONS_DIR}")
message(STATUS "QM_OUTPUT_DIR: ${QM_OUTPUT_DIR}")
message(STATUS "TS_FILES: ${TS_FILES}")

# 检查翻译源文件是否存在
foreach(TS_FILE ${TS_FILES})
    if(EXISTS ${TS_FILE})
        message(STATUS "✓ Found TS file: ${TS_FILE}")
    else()
        message(WARNING "✗ Missing TS file: ${TS_FILE}")
    endif()
endforeach()

# 检查生成的QM文件是否存在
set(QM_FILES
    ${TRANSLATIONS_DIR}/YSCloudScene_zh_CN.qm
    ${TRANSLATIONS_DIR}/YSCloudScene_en_US.qm)

foreach(QM_FILE ${QM_FILES})
    if(EXISTS ${QM_FILE})
        message(STATUS "✓ Found QM file: ${QM_FILE}")
    else()
        message(STATUS "ℹ QM file not found (will be generated): ${QM_FILE}")
    endif()
endforeach()

# 确保翻译文件目录存在
file(MAKE_DIRECTORY ${TRANSLATIONS_DIR})
file(MAKE_DIRECTORY ${QM_OUTPUT_DIR})

message(STATUS "Created directories if not exist:")
message(STATUS "  - ${TRANSLATIONS_DIR}")
message(STATUS "  - ${QM_OUTPUT_DIR}")

# 添加翻译文件复制任务
add_custom_command(
    OUTPUT ${QM_OUTPUT_DIR}/YSCloudScene_zh_CN.qm ${QM_OUTPUT_DIR}/YSCloudScene_en_US.qm
    COMMAND ${CMAKE_COMMAND} -E echo "=== Starting translation file copy ==="
    COMMAND ${CMAKE_COMMAND} -E echo "Source directory: ${TRANSLATIONS_DIR}"
    COMMAND ${CMAKE_COMMAND} -E echo "Target directory: ${QM_OUTPUT_DIR}"
    COMMAND ${CMAKE_COMMAND} -E make_directory ${QM_OUTPUT_DIR}
    COMMAND ${CMAKE_COMMAND} -E echo "Copying YSCloudScene_zh_CN.qm..."
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${TRANSLATIONS_DIR}/YSCloudScene_zh_CN.qm
        ${QM_OUTPUT_DIR}/YSCloudScene_zh_CN.qm
    COMMAND ${CMAKE_COMMAND} -E echo "Copying YSCloudScene_en_US.qm..."
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${TRANSLATIONS_DIR}/YSCloudScene_en_US.qm
        ${QM_OUTPUT_DIR}/YSCloudScene_en_US.qm
    COMMAND ${CMAKE_COMMAND} -E echo "=== Translation file copy completed ==="
    COMMENT "Copying translation files from ${TRANSLATIONS_DIR} to ${QM_OUTPUT_DIR}"
    DEPENDS 
        ${TRANSLATIONS_DIR}/YSCloudScene_zh_CN.qm
        ${TRANSLATIONS_DIR}/YSCloudScene_en_US.qm
    VERBATIM
)

# 更新翻译目标
add_custom_target(update_translations
    COMMAND ${CMAKE_COMMAND} -E echo "=== Starting translation update ==="
    COMMAND ${CMAKE_COMMAND} -E echo "Working directory: ${CMAKE_SOURCE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E echo "Translations directory: ${TRANSLATIONS_DIR}"
    COMMAND ${CMAKE_COMMAND} -E echo "Output directory: ${QM_OUTPUT_DIR}"
    COMMAND ${CMAKE_COMMAND} -E echo "Updating translations"
    COMMAND ${CMAKE_COMMAND} -E make_directory ${TRANSLATIONS_DIR}
    COMMAND ${CMAKE_COMMAND} -E echo "Running lupdate..."
    COMMAND Qt${QT_VERSION_MAJOR}::lupdate ${CMAKE_SOURCE_DIR} -ts ${TS_FILES}
    COMMAND ${CMAKE_COMMAND} -E echo "Running lrelease..."
    COMMAND Qt${QT_VERSION_MAJOR}::lrelease ${TS_FILES}
    COMMAND ${CMAKE_COMMAND} -E echo "Copying generated QM files..."
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${TRANSLATIONS_DIR}/YSCloudScene_zh_CN.qm
        ${QM_OUTPUT_DIR}/YSCloudScene_zh_CN.qm
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${TRANSLATIONS_DIR}/YSCloudScene_en_US.qm
        ${QM_OUTPUT_DIR}/YSCloudScene_en_US.qm
    COMMAND ${CMAKE_COMMAND} -E echo "=== Translation update completed ==="
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Updating and releasing translations"
    VERBATIM
)

# 创建翻译准备目标
add_custom_target(prepare_translations
    COMMAND ${CMAKE_COMMAND} -E echo "=== Preparing translations ==="
    COMMAND ${CMAKE_COMMAND} -E echo "Checking for required QM files..."
    DEPENDS ${QM_OUTPUT_DIR}/YSCloudScene_zh_CN.qm ${QM_OUTPUT_DIR}/YSCloudScene_en_US.qm
)

# 检查最终输出文件
set(FINAL_QM_FILES
    ${QM_OUTPUT_DIR}/YSCloudScene_zh_CN.qm
    ${QM_OUTPUT_DIR}/YSCloudScene_en_US.qm)

message(STATUS "Expected final QM files:")
foreach(FINAL_QM_FILE ${FINAL_QM_FILES})
    message(STATUS "  - ${FINAL_QM_FILE}")
    if(EXISTS ${FINAL_QM_FILE})
        message(STATUS "    Status: ✓ Exists")
    else()
        message(STATUS "    Status: ℹ Will be generated during build")
    endif()
endforeach()

message(STATUS "=== End Translation Configuration ===")

# 资源文件配置
set(RESOURCE_FILES 
    ${CMAKE_SOURCE_DIR}/sources/ys.qrc)

# 根据Qt版本添加资源文件
if(${QT_VERSION_MAJOR} EQUAL 6)
    qt_add_resources(QT_RESOURCES ${RESOURCE_FILES})
else()
    qt5_add_resources(QT_RESOURCES ${RESOURCE_FILES})
endif()

# 资源初始化器文件
set(RESOURCE_INIT_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/resource_initializer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/resource_initializer.cpp
)

# 项目源文件
set(PROJECT_SOURCES
        ${CMAKE_SOURCE_DIR}/mainwindow.cpp
        ${CMAKE_SOURCE_DIR}/mainwindow.h
        ${CMAKE_SOURCE_DIR}/mainwindow.ui
        ${CMAKE_SOURCE_DIR}/toolset/file_handler.h
        ${CMAKE_SOURCE_DIR}/toolset/file_handler.cpp
        ${CMAKE_SOURCE_DIR}/business/app_manager.h
        ${CMAKE_SOURCE_DIR}/business/app_manager.cpp
        ${CMAKE_SOURCE_DIR}/business/raw_histogram_parser.h
        ${CMAKE_SOURCE_DIR}/business/raw_histogram_parser.cpp
        ${CMAKE_SOURCE_DIR}/business/cs96_gamma_histogram.h
        ${CMAKE_SOURCE_DIR}/business/cs96_gamma_histogram.cpp
        ${CMAKE_SOURCE_DIR}/business/gray_tone_mapper.h
        ${CMAKE_SOURCE_DIR}/business/gray_tone_mapper.cpp
        ${CMAKE_SOURCE_DIR}/business/gray_image_controller.h
        ${CMAKE_SOURCE_DIR}/business/gray_image_controller.cpp
        ${CMAKE_SOURCE_DIR}/business/raw_histogram_parse_imp.h
        ${CMAKE_SOURCE_DIR}/business/raw_histogram_parse_imp.cpp
        ${CMAKE_SOURCE_DIR}/business/origin_point_parser.h
        ${CMAKE_SOURCE_DIR}/business/origin_point_parser.cpp
        ${CMAKE_SOURCE_DIR}/business/origin_point_parser_imp.h
        ${CMAKE_SOURCE_DIR}/business/origin_point_parser_imp.cpp
        ${CMAKE_SOURCE_DIR}/business/cs96_gamma_raw.h
        ${CMAKE_SOURCE_DIR}/business/cs96_gamma_raw.cpp
        ${CMAKE_SOURCE_DIR}/business/fl6532_point_parser.h
        ${CMAKE_SOURCE_DIR}/business/fl6532_point_parser.cpp
        ${CMAKE_SOURCE_DIR}/business/fl6532_range_parser.h
        ${CMAKE_SOURCE_DIR}/business/fl6532_range_parser.cpp
        ${CMAKE_SOURCE_DIR}/business/pointcloud_parser.h
        ${CMAKE_SOURCE_DIR}/business/pointcloud_parser.cpp
        ${CMAKE_SOURCE_DIR}/business/pointcloud_parser_imp.h
        ${CMAKE_SOURCE_DIR}/business/pointcloud_parser_imp.cpp
        ${CMAKE_SOURCE_DIR}/business/minidump.h
        ${CMAKE_SOURCE_DIR}/business/minidump.cpp
        ${CMAKE_SOURCE_DIR}/business/ysc_file.h
        ${CMAKE_SOURCE_DIR}/business/ysc_file.cpp
        ${CMAKE_SOURCE_DIR}/business/pointcloud_file_manager.h
        ${CMAKE_SOURCE_DIR}/business/pointcloud_file_manager.cpp
        ${CMAKE_SOURCE_DIR}/business/matrix_lidar/matrix_lidar_adapter.h
        ${CMAKE_SOURCE_DIR}/business/matrix_lidar/matrix_lidar_adapter.cpp
        ${CMAKE_SOURCE_DIR}/component/image_widget.h
        ${CMAKE_SOURCE_DIR}/component/image_widget.cpp
        ${CMAKE_SOURCE_DIR}/component/raw_display_widget.h
        ${CMAKE_SOURCE_DIR}/component/raw_display_widget.cpp
        ${CMAKE_SOURCE_DIR}/component/raw_customplot.h
        ${CMAKE_SOURCE_DIR}/component/raw_customplot.cpp
        ${CMAKE_SOURCE_DIR}/component/motor_widget.h
        ${CMAKE_SOURCE_DIR}/component/motor_widget.cpp
        ${CMAKE_SOURCE_DIR}/component/motor_widget.ui
        ${CMAKE_SOURCE_DIR}/component/roi_select.h
        ${CMAKE_SOURCE_DIR}/component/roi_select.cpp
        ${CMAKE_SOURCE_DIR}/component/roi_select.ui
        ${CMAKE_SOURCE_DIR}/component/pointcloud/pointcloud_display.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/pointcloud_render_frame.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/pointcloud_render_mailbox.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/pointcloud_display.cpp
        ${CMAKE_SOURCE_DIR}/component/pointcloud/playback_control.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/playback_control.cpp
        ${CMAKE_SOURCE_DIR}/component/pointcloud/rotation_indicator.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/rotation_indicator.cpp
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/picked_point_info.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/pick_data_provider.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/pick_algorithm.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/pick_algorithm.cpp
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/single_pick_base.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/single_pick_base.cpp
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/single_pick_overlay.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/single_pick_overlay.cpp
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/single_pick_list.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/single_pick_list.cpp
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/roi_pick_engine.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/roi_pick_engine.cpp
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/pick_info_panel.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/pick_info_panel.cpp
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/mf150_origin_pick_provider.h
        ${CMAKE_SOURCE_DIR}/component/pointcloud/picking/mf150_origin_pick_provider.cpp
        ${CMAKE_SOURCE_DIR}/component/styleditem/center_editem_delegate.h
        ${CMAKE_SOURCE_DIR}/component/styleditem/center_editem_delegate.cpp
        ${CMAKE_SOURCE_DIR}/component/colormapper/colormapper.h
        ${CMAKE_SOURCE_DIR}/component/colormapper/colormapper.cpp
        ${CMAKE_SOURCE_DIR}/component/interval_record/record_interval.h
        ${CMAKE_SOURCE_DIR}/component/interval_record/record_interval.cpp
        ${CMAKE_SOURCE_DIR}/component/crash_reporter/crash_reporter.h
        ${CMAKE_SOURCE_DIR}/component/crash_reporter/crash_reporter.cpp
        ${CMAKE_SOURCE_DIR}/component/toast/toast_tips.h
        ${CMAKE_SOURCE_DIR}/component/toast/toast_tips.cpp
        ${CMAKE_SOURCE_DIR}/component/custom_slider.h
        ${CMAKE_SOURCE_DIR}/component/custom_slider.cpp
        ${CMAKE_SOURCE_DIR}/component/range_slider.h
        ${CMAKE_SOURCE_DIR}/component/range_slider.cpp
        ${CMAKE_SOURCE_DIR}/component/gl_trace.h
        ${CMAKE_SOURCE_DIR}/component/gl_trace.cpp
        ${CMAKE_SOURCE_DIR}/component/ys_scene_def.h
        ${CMAKE_SOURCE_DIR}/component/gui_parameters.h
        ${CMAKE_SOURCE_DIR}/component/gui_parameters.cpp
        ${CMAKE_SOURCE_DIR}/component/continuoussaveraw.h
        ${CMAKE_SOURCE_DIR}/component/continuoussaveraw.cpp
        ${CMAKE_SOURCE_DIR}/component/continuoussaveraw.ui
        ${CMAKE_SOURCE_DIR}/component/continuoussaveorigin.h
        ${CMAKE_SOURCE_DIR}/component/continuoussaveorigin.cpp
        ${CMAKE_SOURCE_DIR}/component/continuoussaveorigin.ui
        ${CMAKE_SOURCE_DIR}/component/continuousdealpoint.h
        ${CMAKE_SOURCE_DIR}/component/continuousdealpoint.cpp
        ${CMAKE_SOURCE_DIR}/component/continuousdealpoint.ui
        ${CMAKE_SOURCE_DIR}/component/colormaptype_distancethreshold.h
        ${CMAKE_SOURCE_DIR}/component/colormaptype_distancethreshold.cpp
        ${CMAKE_SOURCE_DIR}/component/colormaptype_distancethreshold.ui
        ${CMAKE_SOURCE_DIR}/component/node_manager/node_manager.h
        ${CMAKE_SOURCE_DIR}/component/node_manager/node_manager.cpp
        ${CMAKE_SOURCE_DIR}/component/node_detail_panel/node_detail_panel.h
        ${CMAKE_SOURCE_DIR}/component/node_detail_panel/node_detail_panel.cpp
        ${CMAKE_SOURCE_DIR}/component/switch_button/switch_button.h
        ${CMAKE_SOURCE_DIR}/component/switch_button/switch_button.cpp
        ${CMAKE_SOURCE_DIR}/component/ip_address/ipaddress_widget.h
        ${CMAKE_SOURCE_DIR}/component/ip_address/ipaddress_widget.cpp
        ${CMAKE_SOURCE_DIR}/component/collapsible_groupbox/collapsible_groupbox.h
        ${CMAKE_SOURCE_DIR}/component/collapsible_groupbox/collapsible_groupbox.cpp
        ${CMAKE_SOURCE_DIR}/component/status_panel/status_panel.h
        ${CMAKE_SOURCE_DIR}/component/status_panel/status_panel.cpp
        ${CMAKE_SOURCE_DIR}/component/network_settings/network_settings_widget.h
        ${CMAKE_SOURCE_DIR}/component/network_settings/network_settings_widget.cpp
        ${CMAKE_SOURCE_DIR}/component/tools/ip_utils.h
        ${CMAKE_SOURCE_DIR}/component/tools/ip_utils.cpp
        ${CMAKE_SOURCE_DIR}/component/matrix_lidar_options/matrix_lidar_options.h
        ${CMAKE_SOURCE_DIR}/component/matrix_lidar_options/matrix_lidar_options.cpp
        ${CMAKE_SOURCE_DIR}/component/matrix_lidar_options/matrix_lidar_options.ui
        ${CMAKE_SOURCE_DIR}/component/mf150_preview_config/mf150_preview_config.h
        ${CMAKE_SOURCE_DIR}/component/mf150_preview_config/mf150_preview_config.cpp
        ${CMAKE_SOURCE_DIR}/component/cs96_beta_preview_config/cs96_beta_preview_config.h
        ${CMAKE_SOURCE_DIR}/component/cs96_beta_preview_config/cs96_beta_preview_config.cpp
        ${CMAKE_SOURCE_DIR}/business/mf150_calib_controller.h
        ${CMAKE_SOURCE_DIR}/business/mf150_calib_controller.cpp
        ${CMAKE_SOURCE_DIR}/business/cs96_beta_calib_params.h
        ${CMAKE_SOURCE_DIR}/business/cs96_beta_calib_params.cpp
        ${CMAKE_SOURCE_DIR}/business/cs96_beta_param_loader.h
        ${CMAKE_SOURCE_DIR}/business/cs96_beta_param_loader.cpp
        ${CMAKE_SOURCE_DIR}/business/lidar_properties_config/lidar_model_config.h
        ${CMAKE_SOURCE_DIR}/business/lidar_properties_config/lidar_model_config.cpp
        ${CMAKE_SOURCE_DIR}/business/firmware_version/firmware_version_codec.h
        ${CMAKE_SOURCE_DIR}/business/firmware_version/firmware_version_codec.cpp
        ${CMAKE_SOURCE_DIR}/third_party/qcustomplot/qcustomplot.h
        ${CMAKE_SOURCE_DIR}/third_party/qcustomplot/qcustomplot.cpp
        ${CMAKE_SOURCE_DIR}/third_party/qt-toast/src/Enums.h
        ${CMAKE_SOURCE_DIR}/third_party/qt-toast/src/Toast.h
        ${CMAKE_SOURCE_DIR}/third_party/qt-toast/src/Toast.cpp
        ${RESOURCE_INIT_FILES}
)

# 创建ys_scene静态库，添加QT_RESOURCES作为源文件
add_library(ys_scene STATIC
    ${PROJECT_SOURCES}
    ${QT_RESOURCES}  # 使用Qt处理后的资源
    ${QM_OUTPUT_DIR}/YSCloudScene_zh_CN.qm  # 添加翻译文件
    ${QM_OUTPUT_DIR}/YSCloudScene_en_US.qm
)

target_compile_features(ys_scene PUBLIC cxx_std_20)

# 确保资源处理在库构建前完成
add_dependencies(ys_scene prepare_translations)

# 设置编译选项
target_compile_definitions(ys_scene PRIVATE
    QCUSTOMPLOT_USE_OPENGL
    QT_STATICPLUGIN  # 添加这个定义以支持静态链接Qt插件
    YS_HAS_MF150_PICK
)

# 链接依赖库
target_link_libraries(ys_scene PUBLIC
    lumosy::lumomo_ui
    Qt${QT_VERSION_MAJOR}::Network
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGL
    Qt${QT_VERSION_MAJOR}::OpenGLWidgets 
    Qt${QT_VERSION_MAJOR}::PrintSupport
    Qt${QT_VERSION_MAJOR}::Svg
    ys_lidar_sdk_static 
    repair_ground_static 
    mf150_calib_runner
    ALGO_SOLID_LIDAR
    matrix_lidar_sdk_static
    lz4::lz4
    ${OPENGL_LIBRARIES} 
    opengl32
    crashpad::crashpad
)

target_link_libraries(ys_scene PRIVATE TiDfu::Widgets)

# ConcurrentQueue 依赖处理 (Match you_see_sdk logic)
find_package(concurrentqueue CONFIG QUIET)
if(concurrentqueue_FOUND)
    target_link_libraries(ys_scene PUBLIC concurrentqueue::concurrentqueue)
    # Fix for vcpkg structure where concurrentqueue.h is in moodycamel subdir
    find_path(CONCURRENTQUEUE_MOODYCAMEL_DIR NAMES concurrentqueue.h PATH_SUFFIXES concurrentqueue/moodycamel)
    if(CONCURRENTQUEUE_MOODYCAMEL_DIR)
        target_include_directories(ys_scene PUBLIC ${CONCURRENTQUEUE_MOODYCAMEL_DIR})
    endif()
else()
    # Fallback to third_party
    target_include_directories(ys_scene PUBLIC ${CMAKE_SOURCE_DIR}/you_see_sdk/third_party/concurrentqueue)
endif()

# 输出调试信息
message(STATUS "ys_scene: Translations directory: ${TRANSLATIONS_DIR}")
message(STATUS "ys_scene: Resource output directory: ${QM_OUTPUT_DIR}")
message(STATUS "ys_scene: QRC files: ${RESOURCE_FILES}")
message(STATUS "ys_scene: Qt resources: ${QT_RESOURCES}")

find_package(CURL REQUIRED)
target_sources(ys_scene PRIVATE
    ${CMAKE_SOURCE_DIR}/business/device_management/management_protocol.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/management_client.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/management_client.h
    ${CMAKE_SOURCE_DIR}/business/device_management/network_protocol.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/network_access.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/sn_protocol.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/sn_discovery.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/sn_client.cpp
    ${CMAKE_SOURCE_DIR}/business/device_management/sn_client.h
    ${CMAKE_SOURCE_DIR}/component/device_management/network_dialog.cpp
    ${CMAKE_SOURCE_DIR}/component/device_management/sn_dialog.cpp
    ${CMAKE_SOURCE_DIR}/component/device_management/sn_dialog.h
    ${CMAKE_SOURCE_DIR}/component/device_management/mode_management_widget.cpp
    ${CMAKE_SOURCE_DIR}/component/device_management/mode_management_widget.h
)
target_link_libraries(ys_scene PRIVATE CURL::libcurl Qt${QT_VERSION_MAJOR}::Concurrent)
if(WIN32)
    target_link_libraries(ys_scene PRIVATE dnsapi)
endif()
