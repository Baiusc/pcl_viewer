cmake_minimum_required(VERSION 3.21)
include(cmake/Configuration.cmake)
project(you_see LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 发布版沿用原应用的 -O2，调试版由 CMake 保留调试信息。
add_compile_options("$<$<CONFIG:Release>:-O2>")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_subdirectory(network)

# Beta 板级初始化程序独立交付，不进入第三分区应用升级包。
if(MODEL STREQUAL "CS96-beta")
    add_subdirectory(platform/beta/motor)
endif()

# 所有模块共用编译定义，但可选算法仅链接到 CS96 业务模块。
add_library(ys_options INTERFACE)
target_compile_definitions(ys_options INTERFACE
    ${YS_MACRO}=1 MODEL="${MODEL}" LOCAL_TEST=$<BOOL:${LOCAL_TEST}>
    YS_REFLECTIVITY=$<BOOL:${YS_REFLECTIVITY}> BETA_SPOT_SLOPE=$<BOOL:${YS_SPOT_SLOPE}>
    YS_WORK_MODE_MASK=${YS_WORK_MODE_MASK})
if(YS_RUNTIME_MODES)
    target_compile_definitions(ys_options INTERFACE
        YS_FORMAT_HISTOGRAM=${YS_FORMAT_HISTOGRAM}
        YS_FORMAT_POINTCLOUD=${YS_FORMAT_POINTCLOUD}
        YS_FORMAT_GRAY=${YS_FORMAT_GRAY})
endif()
target_compile_options(ys_options INTERFACE -Wall)
target_include_directories(ys_options INTERFACE
    "${CMAKE_BINARY_DIR}/generated" "${CMAKE_SOURCE_DIR}/third_party/dbus/include"
    "${CMAKE_SOURCE_DIR}/third_party/libuv/include" "${CMAKE_SOURCE_DIR}/third_party/lz4")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated")
add_custom_target(gen_version
    COMMAND sh "${CMAKE_SOURCE_DIR}/scripts/gen_version.sh" "${CMAKE_BINARY_DIR}/generated/ys_version_gen.h"
    BYPRODUCTS "${CMAKE_BINARY_DIR}/generated/ys_version_gen.h"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" COMMENT "生成应用版本信息")

file(GLOB_RECURSE COMMON_SOURCES CONFIGURE_DEPENDS common/*.c core/*.c)
file(GLOB_RECURSE BUSINESS_SOURCES CONFIGURE_DEPENDS ys_business/*.c)
string(JSON model_count LENGTH "${YS_CATALOG}" models)
math(EXPR last "${model_count}-1")
foreach(i RANGE ${last})
    string(JSON other MEMBER "${YS_CATALOG}" models ${i})
    if(NOT other STREQUAL MODEL)
        string(JSON suffix GET "${YS_CATALOG}" models "${other}" suffix)
        list(FILTER BUSINESS_SOURCES EXCLUDE REGEX "_${suffix}\\.c$")
    endif()
endforeach()
if(MODEL MATCHES "^CS96-")
    list(FILTER BUSINESS_SOURCES EXCLUDE REGEX "ys_decode_mf150_common\\.c$")
else()
    # MF150 使用自己的测距/SDK 数据，不加载 CS96 标定表和共享解码器。
    list(FILTER BUSINESS_SOURCES EXCLUDE REGEX "ys_decode_common\\.c$")
    list(FILTER BUSINESS_SOURCES EXCLUDE REGEX "ys_algo_(calib_reflectivity|spot_slope|horizontal_corners)\\.c$")
endif()
add_library(ys_core OBJECT ${COMMON_SOURCES})
add_library(ys_business OBJECT ${BUSINESS_SOURCES})
target_link_libraries(ys_core PUBLIC ys_options)
target_link_libraries(ys_business PUBLIC ys_options)
add_dependencies(ys_core gen_version)

set(THIRD_PARTY_SOURCES
    third_party/C-Thread-Pool/thpool.c third_party/cJSON/cJSON.c third_party/kfifo/kfifo.c
    third_party/log.c/log.c third_party/iniparser/dictionary.c third_party/iniparser/iniparser.c
    third_party/minicsv/minicsv.c third_party/lz4/lz4.c third_party/lz4/lz4frame.c
    third_party/lz4/lz4hc.c third_party/lz4/xxhash.c)
add_library(ys_third_party STATIC ${THIRD_PARTY_SOURCES})
target_link_libraries(ys_third_party PRIVATE ys_options)
add_executable(you_see)
if(MODEL STREQUAL "CS96-beta")
    target_sources(you_see PRIVATE manager/fpga_spi.c)
endif()
target_link_libraries(you_see PRIVATE ys_core ys_business ys_third_party lidar_network)
set_target_properties(you_see PROPERTIES LINKER_LANGUAGE CXX BUILD_WITH_INSTALL_RPATH TRUE BUILD_RPATH "$ORIGIN/../lib" INSTALL_RPATH "$ORIGIN/../lib")

# 独立目标避免算法子项目改写父项目工具链、安装位置或编译演示程序。
foreach(pair "${YS_REFLECTIVITY};reflectivity;calib_reflectivity" "${YS_SPOT_SLOPE};spotslope;spot_slope_alog")
    list(GET pair 0 enabled)
    list(GET pair 1 name)
    list(GET pair 2 directory)
    if(enabled)
        file(GLOB sources CONFIGURE_DEPENDS "algo/${directory}/src/*.cpp")
        if(NOT sources)
            message(FATAL_ERROR "缺少算法源码 algo/${directory}，请初始化对应子模块")
        endif()
        add_library(ys_${name} STATIC ${sources})
        target_include_directories(ys_${name} PUBLIC "algo/${directory}/include")
        target_link_libraries(you_see PRIVATE ys_${name})
    endif()
endforeach()

include(cmake/Driver.cmake)
if(YS_RUNTIME_MODES)
    add_subdirectory(manager)
endif()
if(LOCAL_TEST)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(DBUS REQUIRED IMPORTED_TARGET dbus-1)
    pkg_check_modules(LIBUV REQUIRED IMPORTED_TARGET libuv)
    target_link_libraries(you_see PRIVATE PkgConfig::DBUS PkgConfig::LIBUV)
else()
    target_link_libraries(you_see PRIVATE "${CMAKE_SOURCE_DIR}/lib/${MODEL}/libdbus.a" "${CMAKE_SOURCE_DIR}/lib/${MODEL}/libuv.a")
endif()

if(YS_DRIVER STREQUAL "sdk")
    file(GLOB_RECURSE SDK_SOURCES CONFIGURE_DEPENDS third_party/fs_sdk/*.c)
    file(GLOB_RECURSE SDK_HEADERS CONFIGURE_DEPENDS third_party/fs_sdk/*.h)
    set(SDK_DIRS "${CMAKE_SOURCE_DIR}/third_party/fs_sdk")
    foreach(header IN LISTS SDK_HEADERS)
        get_filename_component(directory "${header}" DIRECTORY)
        list(APPEND SDK_DIRS "${directory}")
    endforeach()
    list(REMOVE_DUPLICATES SDK_DIRS)
    target_include_directories(ys_options INTERFACE ${SDK_DIRS})
    add_library(ys_sdk STATIC ${SDK_SOURCES})
    target_link_libraries(ys_sdk PRIVATE ys_options)
    set(SDK_LIB_DIR "${CMAKE_SOURCE_DIR}/third_party/fs_sdk/bsp/rockchip/rv1126b/sdk_libs")
    target_link_libraries(you_see PRIVATE ys_sdk "${SDK_LIB_DIR}/libdata_parse.a" "${SDK_LIB_DIR}/lib_FL6x32_algo.so" "${SDK_LIB_DIR}/libfspc_support.a")
    install(DIRECTORY "${SDK_LIB_DIR}/" DESTINATION lib FILES_MATCHING PATTERN "*.so*")
endif()

# 系统库置于依赖它们的静态库之后，兼容旧版交叉链接器。
target_link_libraries(you_see PRIVATE pthread dl m)

# 配置只写入产物目录，不修改仓库模板和设备标定数据。
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/conf/ys_conf.ini")
file(READ "${CMAKE_SOURCE_DIR}/conf/ys_conf.ini" runtime_config)
string(REGEX REPLACE "work_mode[ \t]*=[ \t]*\"[0-9]+\"" "work_mode = \"${YS_DEFAULT_WORK_MODE}\"" runtime_config "${runtime_config}")
# 明确生成运行算法默认值；设备配置与标定数据由后续管理服务合成。
string(REPLACE "[system]" "[system]\nenable_reflectivity = \"0\"\nenable_spot_slope = \"0\"" runtime_config "${runtime_config}")
file(WRITE "${CMAKE_BINARY_DIR}/generated/ys_conf.ini" "${runtime_config}")
file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/generated/build-info.json" CONTENT
"{\n  \"model\": \"${MODEL}\", \"data_mode\": \"${DATA_MODE}\",\n  \"reflectivity\": $<IF:$<BOOL:${YS_REFLECTIVITY}>,true,false>, \"spotslope\": $<IF:$<BOOL:${YS_SPOT_SLOPE}>,true,false>,\n  \"driver_kind\": \"${YS_DRIVER}\", \"driver_mode\": \"${DRIVER_RCV_MODE}\",\n  \"compiler\": \"${CMAKE_C_COMPILER}\", \"compiler_version\": \"${CMAKE_C_COMPILER_VERSION}\",\n  \"build_type\": \"${CMAKE_BUILD_TYPE}\"\n}\n")
add_custom_target(build_manifest ALL
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/scripts/build.py" manifest "${CMAKE_BINARY_DIR}"
    DEPENDS you_see WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" COMMENT "记录构建来源及产物校验值")
add_dependencies(build_manifest lidar_netd lidar_netctl lidar_network.cgi lidar_network_status.cgi)
if(YS_RUNTIME_MODES)
    add_dependencies(build_manifest lidar_mgrd lidar_mgrctl lidar_crypto lidar_boot lidar_netd lidar_netctl)
endif()
install(TARGETS you_see RUNTIME DESTINATION bin)
install(FILES "${CMAKE_BINARY_DIR}/generated/ys_conf.ini" DESTINATION conf)
install(FILES "${CMAKE_BINARY_DIR}/manifest.json" DESTINATION .)
