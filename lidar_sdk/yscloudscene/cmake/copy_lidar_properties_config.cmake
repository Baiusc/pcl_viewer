# 将 business/lidar_properties_config 中的属性配置文件（*.json）拷贝到目标运行目录，
# 排除源码文件(*.h/*.cpp)，保证运行时目录干净。
# 用法: cmake -DSRC_DIR=<源配置目录> -DDST_DIR=<exe 所在目录> -P copy_lidar_properties_config.cmake
if(NOT DEFINED SRC_DIR OR NOT DEFINED DST_DIR)
    message(FATAL_ERROR "SRC_DIR / DST_DIR must be set")
endif()

file(REMOVE_RECURSE "${DST_DIR}/lidar_properties_config")

file(COPY "${SRC_DIR}"
     DESTINATION "${DST_DIR}"
     PATTERN "*.h" EXCLUDE
     PATTERN "*.cpp" EXCLUDE
)

message(STATUS "Copied lidar_properties_config to ${DST_DIR}/lidar_properties_config")
