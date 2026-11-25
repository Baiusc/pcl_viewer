cmake_minimum_required(VERSION 3.20)

project(YSCloudScene VERSION 2.0.10 LANGUAGES CXX)

### ---------------- Qt配置 ------------------

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 优先查找Qt6，如果未找到则查找Qt5
find_package(Qt6 COMPONENTS Widgets OpenGL OpenGLWidgets PrintSupport Network Svg LinguistTools Concurrent OPTIONAL_COMPONENTS)
find_package(crashpad CONFIG REQUIRED)
# LZ4(vcpkg)：FL6532 Range LZ4 压缩帧解压
find_package(lz4 CONFIG REQUIRED)
#find_package(eclipse-paho-mqtt-c CONFIG REQUIRED)
#find_package(PahoMqttCpp CONFIG REQUIRED)

if(Qt6_FOUND)
    set(QT_VERSION_MAJOR 6)
    message(STATUS "Found Qt6")
else()
    find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL PrintSupport Network Svg LinguistTools Concurrent)
    set(QT_VERSION_MAJOR 5)
    message(STATUS "Found Qt5")
endif()

# 添加SDK子目录
add_subdirectory(you_lib)
# you_lib 的 Windows 网络实现使用 WIN32 宏；自定义 CMAKE_CXX_FLAGS 时不能依赖编译器默认 flags。
if(WIN32)
    target_compile_definitions(you_lib_ys_net PUBLIC WIN32)
endif()
add_subdirectory(you_see_sdk)
add_subdirectory(matrix_lidar_sdk)
add_subdirectory(lumosy)
# 为子项目repair_ground设置编译选项
set(REPAIR_GROUND_OPTIMIZATION ON CACHE BOOL "Enable high optimization for repair_ground")

# 算法目录
set(YS_ALGORITHM_DIR ${CMAKE_SOURCE_DIR}/algorithm)
link_directories(${YS_ALGORITHM_DIR})
add_subdirectory(${YS_ALGORITHM_DIR}/repair_ground)
add_subdirectory(${YS_ALGORITHM_DIR}/algo_solid_lidar)

# 标定 runner 独立目标（链接算法统一库 ALGO_SOLID_LIDAR：FL6031 + FL6532 RANGE）
add_library(mf150_calib_runner STATIC
    business/mf150_calibration_runner.h
    business/mf150_calibration_runner.cpp
    business/cs96_raw_save.h
    business/cs96_raw_save.cpp
)
target_include_directories(mf150_calib_runner PUBLIC ${CMAKE_SOURCE_DIR})
target_include_directories(mf150_calib_runner PUBLIC ${CMAKE_SOURCE_DIR}/you_see_sdk/include)
target_link_libraries(mf150_calib_runner PUBLIC ALGO_SOLID_LIDAR)

# 添加固件烧录 SDK 和 ys_scene 库
add_subdirectory(sdk/ti_dfu)
add_subdirectory(scene)

# 主程序源文件
set(MAIN_SOURCES
    main.cpp
)

if(WIN32)
    list(APPEND MAIN_SOURCES app.rc)
endif()

# 根据Qt版本设置可执行文件目标
if(${QT_VERSION_MAJOR} EQUAL 6)
    qt_add_executable(YSCloudScene
        MANUAL_FINALIZATION
        ${MAIN_SOURCES}
    )
else()
    # 添加WIN32标识使其成为纯GUI应用
    add_executable(YSCloudScene 
        WIN32
        ${MAIN_SOURCES}
    )
endif()
# 链接依赖库
target_link_libraries(YSCloudScene PRIVATE  
    Qt${QT_VERSION_MAJOR}::Network
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGL
    Qt${QT_VERSION_MAJOR}::OpenGLWidgets 
    Qt${QT_VERSION_MAJOR}::PrintSupport
    ys_scene
    ys_lidar_sdk_static 
    repair_ground_static 
    ${OPENGL_LIBRARIES} 
    opengl32
)

target_link_libraries(YSCloudScene PRIVATE crashpad::crashpad)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    QCUSTOMPLOT_USE_OPENGL
)

# 安装目标
include(GNUInstallDirs)
install(TARGETS YSCloudScene
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 如果使用Qt6，调用qt_finalize_executable
if(${QT_VERSION_MAJOR} EQUAL 6)
    qt_finalize_executable(YSCloudScene)
endif()

# Windows下自动部署Qt依赖库到构建目录
if(WIN32 AND ${QT_VERSION_MAJOR} EQUAL 6)
    find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${_qt6Core_install_prefix}/bin")

    if(WINDEPLOYQT_EXECUTABLE)
        add_custom_command(TARGET YSCloudScene POST_BUILD
            COMMAND "${WINDEPLOYQT_EXECUTABLE}"
                --no-compiler-runtime
                --no-system-d3d-compiler
                --no-opengl-sw
                "$<TARGET_FILE:YSCloudScene>"
            COMMENT "Deploying Qt libraries to build directory..."
        )
        message(STATUS "Found windeployqt: ${WINDEPLOYQT_EXECUTABLE}")
    else()
        message(WARNING "windeployqt not found. Qt libraries will not be automatically deployed.")
    endif()
endif()

# 拷贝设备属性配置到 exe 同级（属性树按型号加载 lidar_properties_config/<型号>/，排除源码文件）
add_custom_command(TARGET YSCloudScene POST_BUILD
    COMMAND ${CMAKE_COMMAND}
        -DSRC_DIR=${CMAKE_SOURCE_DIR}/business/lidar_properties_config
        -DDST_DIR=$<TARGET_FILE_DIR:YSCloudScene>
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/copy_lidar_properties_config.cmake
    COMMENT "Copying lidar_properties_config next to YSCloudScene.exe..."
    VERBATIM
)

# 针对Windows的特殊处理
if (WIN32)
    set_target_properties(YSCloudScene PROPERTIES
        WIN32_EXECUTABLE OFF
    )
endif()

enable_testing()
add_subdirectory(tests)

# ============ CPack 打包配置 (ZIP 压缩包) ============
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
set(CPACK_PACKAGE_VENDOR "YuSheng")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "YSCloudScene - 激光雷达点云可视化与处理")

set(CPACK_INSTALL_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/cpack_install.cmake")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/cpack_install.cmake"
"# 复制主程序
file(COPY \"${CMAKE_CURRENT_BINARY_DIR}/YSCloudScene.exe\"
     DESTINATION \"\${CPACK_TEMPORARY_DIRECTORY}\"
)

# 复制 MSVC 运行时
foreach(_lib \${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS})
    get_filename_component(_name \"\${_lib}\" NAME)
    if(EXISTS \"\${_lib}\")
        file(COPY \"\${_lib}\" DESTINATION \"\${CPACK_TEMPORARY_DIRECTORY}\")
    endif()
endforeach()

# 复制所有 DLL
file(GLOB _all_dlls \"${CMAKE_CURRENT_BINARY_DIR}/*.dll\")
if(_all_dlls)
    foreach(_dll \${_all_dlls})
        file(COPY \"\${_dll}\" DESTINATION \"\${CPACK_TEMPORARY_DIRECTORY}\")
    endforeach()
endif()

# 复制 Qt 插件目录
set(_plugin_dirs platforms imageformats styles iconengines tls networkinformation generic translations)
foreach(_dir \${_plugin_dirs})
    if(EXISTS \"${CMAKE_CURRENT_BINARY_DIR}/\${_dir}\")
        file(COPY \"${CMAKE_CURRENT_BINARY_DIR}/\${_dir}\"
             DESTINATION \"\${CPACK_TEMPORARY_DIRECTORY}\"
        )
    endif()
endforeach()

# 复制 lidar_properties_config 目录（属性树按型号加载）
if(EXISTS \"${CMAKE_CURRENT_BINARY_DIR}/lidar_properties_config\")
    file(COPY \"${CMAKE_CURRENT_BINARY_DIR}/lidar_properties_config\"
         DESTINATION \"\${CPACK_TEMPORARY_DIRECTORY}\"
    )
endif()
")

include(CPack)
