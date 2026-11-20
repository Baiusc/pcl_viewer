# ------------------------------------------------------------------------------
# cloud_viewer 依赖统一入口
#
# 环境基线：
#   Ubuntu 24.04：PCL 1.14.0 / VTK 9.1 / Boost 1.83 / Eigen 3.4（全部系统 apt）
#   Windows 11 + MSYS2(MinGW64)：PCL 1.14 / VTK 9.x / Eigen 3.4（工具链自带，
#   无完整 Boost 动态库，PCLConfig 不可用）
#
# PCL 接入策略（CLOUD_VIEWER_PCL_MODE）：
#   auto（默认）—— 先尝试标准 PCLConfig；找不到则自动降级为「手动 PCL 路径」，
#                  只链接不依赖 Boost 动态库的 pcl_common/octree/kdtree/search/filters。
#   standard    —— 强制标准 PCLConfig（要求 Boost 组件齐全）。
#   manual      —— 强制手动路径（MSYS2/MinGW 这类无 PCLConfig 的环境）。
# 该降级不是架构妥协：渲染层本就以 VTK 为底层，PCL 仅提供点类型与 KDTree；
# 而 PCLVisualizer / pcl_io / pcl_visualization 需要完整 Boost，MinGW 端没有。
#
# 约定：渲染统一由 VTK 9 承担，PCL 只提供点类型、KDTree 与算法。
# ------------------------------------------------------------------------------

set(CLOUD_VIEWER_PCL_MODE "auto" CACHE STRING "PCL 接入模式: auto | standard | manual")
set_property(CACHE CLOUD_VIEWER_PCL_MODE PROPERTY STRINGS auto standard manual)

set(_cv_use_std_pcl FALSE)

# Windows：若 PCL_ROOT 下确实有 PCLConfig，则交给标准查找（Ubuntu 由 apt 的 PCLConfig 自动定位）。
if(WIN32 AND DEFINED ENV{PCL_ROOT} AND NOT PCL_DIR)
    if(EXISTS "$ENV{PCL_ROOT}/cmake/PCLConfig.cmake")
        set(PCL_DIR "$ENV{PCL_ROOT}/cmake")
    endif()
endif()

if(CLOUD_VIEWER_PCL_MODE STREQUAL "standard")
    set(_cv_use_std_pcl TRUE)
elseif(CLOUD_VIEWER_PCL_MODE STREQUAL "auto")
    find_package(PCL 1.10 QUIET COMPONENTS common io kdtree search visualization)
    if(PCL_FOUND)
        set(_cv_use_std_pcl TRUE)
    else()
        message(STATUS "[cloud_viewer] 未找到完整 PCLConfig，PCL 采用手动模式（不含 io/visualization）。")
    endif()
else()
    message(STATUS "[cloud_viewer] PCL 强制手动模式。")
endif()

if(_cv_use_std_pcl)
    set(CLOUD_VIEWER_PCL_STANDARD TRUE)
else()
    set(CLOUD_VIEWER_PCL_STANDARD FALSE)

    # 手动模式：按候选根目录探测 include/pcl-* 与 lib 位置（覆盖 MSYS2 / Linux / Windows 安装包）。
    if(WIN32)
        # 从 C/CXX 编译器位置反推 MinGW 前缀（形如 <prefix>/bin/g++.exe），
        # 这样工具链放在共享盘、C 盘或任意目录都能定位到同前缀下的 PCL/Eigen，
        # 不再依赖 MSYS2 挂载点 /mingw64（原生 cmake.exe 会把它解析成 C:/mingw64）。
        get_filename_component(_cv_cc_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
        get_filename_component(_cv_mingw_prefix "${_cv_cc_dir}/.." ABSOLUTE)
        set(_cv_pcl_roots
            "$ENV{PCL_ROOT}"
            "${_cv_mingw_prefix}"
            "C:/Program Files/PCL 1.14.1")
    else()
        set(_cv_pcl_roots "$ENV{PCL_ROOT}" "/usr/local" "/usr")
    endif()

    set(_cv_pcl_include "")
    set(_cv_pcl_libdir "")
    foreach(_root IN LISTS _cv_pcl_roots)
        if(_root AND NOT _cv_pcl_include)
            file(GLOB _cv_pcl_dirs "${_root}/include/pcl-1.1[0-9]")
            if(_cv_pcl_dirs)
                list(GET _cv_pcl_dirs 0 _cv_pcl_include)
                set(_cv_pcl_libdir "${_root}/lib")
            endif()
        endif()
    endforeach()

    # MSYS2 常见位置（PCL_ROOT 未设置时兜底）：只在编译器前缀没命中时才用到。
    if(NOT _cv_pcl_include AND WIN32)
        file(GLOB _cv_pcl_dirs "${_cv_mingw_prefix}/include/pcl-1.1[0-9]")
        if(_cv_pcl_dirs)
            list(GET _cv_pcl_dirs 0 _cv_pcl_include)
            set(_cv_pcl_libdir "${_cv_mingw_prefix}/lib")
        endif()
    endif()

    if(NOT _cv_pcl_include OR NOT EXISTS "${_cv_pcl_include}/pcl/point_types.h")
        message(FATAL_ERROR
            "[cloud_viewer] 未找到 PCL 头文件（尝试过: ${_cv_pcl_roots}）。"
            "请安装 PCL 或设置 PCL_ROOT，或用 -DCLOUD_VIEWER_PCL_MODE=standard 指定标准配置。")
    endif()

    message(STATUS "[cloud_viewer] PCL manual: ${_cv_pcl_include}")

    add_library(cv_pcl_manual INTERFACE)
    target_include_directories(cv_pcl_manual INTERFACE
        "${_cv_pcl_include}"
        "${_cv_pcl_include}/.."
    )
    # Eigen 由各平台自行定位：Linux 常见 /usr/include/eigen3，其余取 PCL 前缀的同级 include/eigen3
    # （MSYS2/PCL 安装包布局：<prefix>/include/pcl-1.14 与 <prefix>/include/eigen3 同级）。
    if(EXISTS "/usr/include/eigen3")
        target_include_directories(cv_pcl_manual INTERFACE "/usr/include/eigen3")
    elseif(EXISTS "${_cv_pcl_include}/../eigen3")
        target_include_directories(cv_pcl_manual INTERFACE "${_cv_pcl_include}/../eigen3")
    endif()
    target_link_directories(cv_pcl_manual INTERFACE "${_cv_pcl_libdir}")
    target_link_libraries(cv_pcl_manual INTERFACE
        pcl_common pcl_octree pcl_kdtree pcl_search pcl_filters)
endif()

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
    # 平台交互器（Windows: vtkWin32RenderWindowInteractor / Linux: vtkXRenderWindowInteractor）
    RenderingUI
    RenderingAnnotation
    RenderingContext2D
    RenderingFreeType
    InteractionStyle
    InteractionWidgets
    IOImage
)

# 所有依赖通过 INTERFACE target 暴露，避免全局 include/link 污染。
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
        VTK::RenderingUI
        VTK::RenderingAnnotation
        VTK::RenderingContext2D
        VTK::RenderingFreeType
        VTK::InteractionStyle
        VTK::InteractionWidgets
        VTK::IOImage
    )
endif()

# ------------------------------------------------------------------------------
# VTK 9 模块必须显式 autoinit
#
# VTK 9 把「创建具体窗口/渲染器」的类放在各模块的 .so/.dll 里，靠模块的
# vtk_module_init_* 建立对象工厂。若只 target_link_libraries 而不 autoinit：
#   - Linux：默认的 --as-needed 还会把未被直接引用的 libvtkRenderingOpenGL2 丢掉；
#   - Windows(MinGW)：DLL 自己的静态初始化不会被带进来；
# 结果 vtkRenderWindow::New() 得到没有后端的基类：
#   GetClassName() == "vtkRenderWindow"（而非 vtkXOpenGLRenderWindow / vtkWin32OpenGLRenderWindow）
#   GetMapped() == 0，窗口不弹出；GetInteractor() == nullptr，Start() 立刻返回。
# 表现为程序打印完日志后静默退出（Linux，返回码 0），或进程空转但无窗口（Windows）。
#
# 只能作用在具体 target 上（INTERFACE target 挂不了源码），故封装成函数逐个调用。
# 可用 CV_DEBUG_WINDOW=1 运行程序自检实际类名。
# ------------------------------------------------------------------------------
function(cv_enable_vtk_autoinit target)
    if(COMMAND vtk_module_autoinit)
        vtk_module_autoinit(TARGETS ${target} MODULES ${VTK_LIBRARIES})
    endif()
endfunction()

if(CLOUD_VIEWER_PCL_STANDARD)
    message(STATUS "[cloud_viewer] PCL: ${PCL_VERSION} @ ${PCL_DIR}")
else()
    message(STATUS "[cloud_viewer] PCL: manual @ ${_cv_pcl_include}")
endif()
message(STATUS "[cloud_viewer] VTK: ${VTK_VERSION} @ ${VTK_DIR}")
