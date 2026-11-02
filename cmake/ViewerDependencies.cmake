# ------------------------------------------------------------------------------
# cloud_viewer 依赖统一入口
#
# 依赖策略：
#   1. 优先使用标准 PCLConfig（要求 Boost system/iostreams/filesystem/serialization 齐全）。
#   2. 若 Boost 组件不全，自动降级为「手动 PCL 路径」：只链接不依赖 Boost 动态库的
#      pcl_common / pcl_kdtree / pcl_octree / pcl_search / pcl_filters。
#   3. 渲染统一由 VTK 9 承担；降级模式不使用 PCLVisualizer（其依赖缺失的 boost 1.71）。
#   4. 所有依赖通过 INTERFACE target 暴露，避免全局 include/link 污染。
#
# 该降级不是架构妥协：渲染层本就以 VTK 为底层，PCL 仅提供点类型与 KDTree。
# ------------------------------------------------------------------------------

set(CLOUD_VIEWER_PCL_MODE "auto" CACHE STRING "PCL 接入模式: auto | standard | manual")
set_property(CACHE CLOUD_VIEWER_PCL_MODE PROPERTY STRINGS auto standard manual)

set(_cv_use_std_pcl FALSE)

if(CLOUD_VIEWER_PCL_MODE STREQUAL "standard")
    set(_cv_use_std_pcl TRUE)
elseif(CLOUD_VIEWER_PCL_MODE STREQUAL "auto")
    find_package(Boost QUIET COMPONENTS system iostreams filesystem serialization)
    if(Boost_FOUND)
        set(_cv_use_std_pcl TRUE)
    else()
        message(STATUS "[cloud_viewer] Boost 组件不完整，PCL 采用手动模式（不含 io/visualization）。")
    endif()
else()
    message(STATUS "[cloud_viewer] PCL 强制手动模式。")
endif()

if(_cv_use_std_pcl)
    find_package(PCL 1.10 REQUIRED COMPONENTS common io kdtree search visualization)
    set(CLOUD_VIEWER_PCL_STANDARD TRUE)
else()
    set(CLOUD_VIEWER_PCL_STANDARD FALSE)

    # 跨平台手动定位 PCL 头文件与库目录。
    if(WIN32)
        set(_cv_pcl_root "$ENV{PCL_ROOT}")
        if(NOT _cv_pcl_root)
            set(_cv_pcl_root "C:/Program Files/PCL 1.14.1")
        endif()
        set(_cv_pcl_include "${_cv_pcl_root}/include/pcl-1.14")
        set(_cv_pcl_libdir "${_cv_pcl_root}/lib")
    else()
        set(_cv_pcl_include "/usr/local/include/pcl-1.14")
        set(_cv_pcl_libdir "/usr/local/lib")
        if(NOT EXISTS "${_cv_pcl_include}")
            set(_cv_pcl_include "/usr/include/pcl-1.14")
        endif()
    endif()

    if(NOT EXISTS "${_cv_pcl_include}/pcl/point_types.h")
        message(FATAL_ERROR
            "[cloud_viewer] 未找到 PCL 头文件（${_cv_pcl_include}）。"
            "请安装 PCL 或设置 PCL_ROOT。")
    endif()

    add_library(cv_pcl_manual INTERFACE)
    target_include_directories(cv_pcl_manual INTERFACE
        "${_cv_pcl_include}"
        "${_cv_pcl_include}/.."
    )
    if(EXISTS "/usr/include/eigen3")
        target_include_directories(cv_pcl_manual INTERFACE "/usr/include/eigen3")
    endif()
    target_link_directories(cv_pcl_manual INTERFACE "${_cv_pcl_libdir}")
    target_link_libraries(cv_pcl_manual INTERFACE
        pcl_common pcl_octree pcl_kdtree pcl_search pcl_filters)
endif()

# ------------------------------------------------------------------------------
# VTK：渲染层统一依赖
# ------------------------------------------------------------------------------

find_package(VTK 9.0 REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    CommonColor
    CommonMath
    CommonTransforms
    CommonMisc
    FiltersCore
    FiltersGeneral
    FiltersSources
    FiltersGeometry
    RenderingCore
    RenderingOpenGL2
    RenderingAnnotation
    RenderingContext2D
    RenderingFreeType
    InteractionStyle
    InteractionWidgets
    IOImage
)

if(NOT TARGET viewer_deps)
    add_library(viewer_deps INTERFACE)
    target_include_directories(viewer_deps INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

    if(TARGET cv_pcl_manual)
        target_link_libraries(viewer_deps INTERFACE cv_pcl_manual)
    else()
        target_include_directories(viewer_deps INTERFACE ${PCL_INCLUDE_DIRS})
        if(PCL_DEFINITIONS)
            target_compile_definitions(viewer_deps INTERFACE ${PCL_DEFINITIONS})
        endif()
        if(PCL_LIBRARY_DIRS)
            target_link_directories(viewer_deps INTERFACE ${PCL_LIBRARY_DIRS})
        endif()
        target_link_libraries(viewer_deps INTERFACE ${PCL_LIBRARIES})
    endif()

    target_link_libraries(viewer_deps INTERFACE
        VTK::CommonCore
        VTK::CommonDataModel
        VTK::CommonColor
        VTK::CommonMath
        VTK::CommonTransforms
        VTK::CommonMisc
        VTK::FiltersCore
        VTK::FiltersGeneral
        VTK::FiltersSources
        VTK::FiltersGeometry
        VTK::RenderingCore
        VTK::RenderingOpenGL2
        VTK::RenderingAnnotation
        VTK::RenderingContext2D
        VTK::RenderingFreeType
        VTK::InteractionStyle
        VTK::InteractionWidgets
        VTK::IOImage
    )
endif()

message(STATUS "[cloud_viewer] VTK_DIR: ${VTK_DIR}")
message(STATUS "[cloud_viewer] PCL standard config: ${CLOUD_VIEWER_PCL_STANDARD}")
