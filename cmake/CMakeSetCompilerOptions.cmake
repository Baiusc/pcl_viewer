# 设置 C++ 标准为 C++20
set( CMAKE_CXX_STANDARD 20 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS NO )
# # 设置默认构建类型 （慎用）
# if(MSVC)
#     set(CMAKE_BUILD_TYPE "Release")
# else()
#     set(CMAKE_BUILD_TYPE "Release")
# endif()
# 设置安装路径
set(CMAKE_INSTALL_PREFIX "../install" CACHE PATH "安装根目录" FORCE)
# 指定 Debug 模式下生成的可执行文件或库的后缀
set(CMAKE_DEBUG_POSTFIX "d")
# 公共编译选项
if(MSVC)
    message("======================MSVC=======================")
    # MSVC 特定设置
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8") # 修改 MSVC 默认的 UTF-8 编码支持
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0 /wd4530 /wd4006 /wd4828") # 关闭所有警告 / 关闭异常处理相关警告 / 关闭符号重复定义警告
    # Debug 模式编译选项
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Od /Zi") # 链接到 Debug 版本的多线程运行时库 / 禁用所有优化 / 生成调试信息
    # Release 模式编译选项 (为了 Release 模式下断点调试)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Od /Zi") # 链接到 Release 版本的多线程运行时库 / 禁用所有优化 / 生成调试信息
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG") # 为 Release 模式下的链接过程添加调试信息
elseif(UNIX)
    message("======================UNIX=======================")
    # GCC/Clang 特定设置
    set(PLATFORM_COMPILE_FLAGS "-march=native")
    set(COMMON_COMPILE_FLAGS "${COMMON_COMPILE_FLAGS} -fopenmp") # 编译阶段启用 OpenMP 支持
    # set(COMMON_COMPILE_FLAGS "${COMMON_COMPILE_FLAGS} -Wall")  # 编译阶段启用所有 warning 警告
    set(COMMON_COMPILE_FLAGS "${COMMON_COMPILE_FLAGS} -Wno-sign-compare") # 禁用 由编译器产生的关于 有符号整数与无符号整数比较 时的警告
    set(CMAKE_CXX_FLAGS "${PLATFORM_COMPILE_FLAGS} ${COMMON_COMPILE_FLAGS} $ENV{CXXFLAGS}")
    # Debug 模式编译选项
    set(CMAKE_CXX_FLAGS_DEBUG "" CACHE STRING "Clear default debug flags" FORCE) # 清空
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -ggdb" CACHE STRING "Custom debug flags" FORCE) # 生成调试信息 g2 等级 / 禁用所有优化 / 为 GDB 生成调试信息
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g") 
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fopenmp") # 链接阶段启用 OpenMP 支持
    # Release 模式编译选项 (为了 Release 模式下断点调试)
    set(CMAKE_CXX_FLAGS_RELEASE "" CACHE STRING "Clear default release flags" FORCE) # 清空
    set(CMAKE_CXX_FLAGS_RELEASE "-g -O0 -ggdb" CACHE STRING "Custom release flags" FORCE) # 生成调试信息 g2 等级 / 禁用所有优化 / 为 GDB 生成调试信息
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -g") 
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -fopenmp") # 链接阶段启用 OpenMP 支持
endif()

# 使用 ccache 加速编译
find_program( CCACHE_PROGRAM ccache )
if ( CCACHE_PROGRAM )
    set( CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM} )
    set( CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM} )
endif()

# 针对不同平台进行设置
if ( UNIX )
    # 设置为位置无关代码 (PIC) 用于共享库
    set( CMAKE_POSITION_INDEPENDENT_CODE ON )
elseif( MSVC )
    # MSVC 特定设置
    add_definitions(-DNOMINMAX -D_CRT_SECURE_NO_WARNINGS -D__STDC_LIMIT_MACROS)

    # 启用多线程编译
    option( OPTION_MP_BUILD "Check to activate multithreaded compilation with MSVC" ON )
    if( ${OPTION_MP_BUILD} )
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") 
    endif()

    # 禁用 checked iterators (SECURE_SCL)
    set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /D _SECURE_SCL=0")

    # 启用 Visual Leak Detector (VLD) 检测内存泄漏
    option( OPTION_USE_VISUAL_LEAK_DETECTOR "Check to activate compilation (in debug) with Visual Leak Detector" OFF )
    if( ${OPTION_USE_VISUAL_LEAK_DETECTOR} )
        set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /D USE_VLD" )
    endif()
endif()

