# ------------------------------------------------------------------------------
# cloud_viewer 依赖统一入口
#
# 环境基线（Ubuntu 24.04，全部来自系统 apt，机器上只有这一套）：
#   PCL 1.14.0 / VTK 9.1 / Boost 1.83 / Eigen 3.4
# Windows 端通过环境变量 PCL_ROOT 指向 PCL 1.14.1 安装目录（含 3rdParty）。
#
# 历史说明：原先有一层「Boost 组件不全 → 降级为手动 PCL 路径」的防御逻辑，
# 是为 Ubuntu 20.04 时期自编译的 PCL 1.14.1（DT_NEEDED 指向 Boost 1.71）与
# 系统 Boost 1.83 冲突而设。该副本已于 2026-11-20 清理，环境里只剩一套自洽的
# 库，故该分支连同 CLOUD_VIEWER_PCL_MODE 开关一并移除。
#
# 约定：渲染统一由 VTK 9 承担，PCL 只提供点类型、KDTree 与算法。
# ------------------------------------------------------------------------------

# Windows：让标准 PCLConfig 能被找到（Linux 由 apt 的 PCLConfig 自动定位）。
if(WIN32 AND DEFINED ENV{PCL_ROOT} AND NOT PCL_DIR)
    set(PCL_DIR "$ENV{PCL_ROOT}/cmake")
endif()

find_package(PCL 1.10 REQUIRED COMPONENTS common io kdtree search visualization)

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

# 所有依赖通过 INTERFACE target 暴露，避免全局 include/link 污染。
if(NOT TARGET viewer_deps)
    add_library(viewer_deps INTERFACE)
    target_include_directories(viewer_deps INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
    )
    if(PCL_DEFINITIONS)
        target_compile_definitions(viewer_deps INTERFACE ${PCL_DEFINITIONS})
    endif()
    if(PCL_LIBRARY_DIRS)
        target_link_directories(viewer_deps INTERFACE ${PCL_LIBRARY_DIRS})
    endif()

    target_link_libraries(viewer_deps INTERFACE
        ${PCL_LIBRARIES}
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

# ------------------------------------------------------------------------------
# VTK 9 模块必须显式 autoinit
#
# VTK 9 把「创建具体窗口/渲染器」的类放在各模块的 .so 里，靠模块的
# vtk_module_init_* 建立对象工厂。若只 target_link_libraries 而不 autoinit，
# Ubuntu 默认的 --as-needed 还会把未被直接引用的 libvtkRenderingOpenGL2 丢掉，
# 结果 vtkRenderWindow::New() 得到没有后端的基类：
#   GetClassName() == "vtkRenderWindow"（而非 vtkXOpenGLRenderWindow）
#   GetMapped() == 0，窗口不弹出；GetInteractor() == nullptr，Start() 立刻返回。
# 表现为程序打印完日志后静默退出（返回码 0）。
# ------------------------------------------------------------------------------
function(cv_enable_vtk_autoinit target)
    if(COMMAND vtk_module_autoinit)
        vtk_module_autoinit(TARGETS ${target} MODULES ${VTK_LIBRARIES})
    endif()
endfunction()

message(STATUS "[cloud_viewer] PCL: ${PCL_VERSION} @ ${PCL_DIR}")
message(STATUS "[cloud_viewer] VTK: ${VTK_VERSION} @ ${VTK_DIR}")
