# 本地环境配置模板

message("================== 设置应用安装根目录")
set(CMAKE_INSTALL_PREFIX "../install" CACHE PATH "安装根目录" FORCE)

# 判断操作系统并设置路径
if(WIN32)
    # Windows 下的路径
    # set(OpenCV_DIR "D:/ExternalLibraries/opencv/build")
    set(PCL_PATH "D:/ExternalLibraries/PCL1.14.1")
elseif(UNIX)
    # Linux 下的路径
    # set(OpenCV_DIR "/home/user/ExternalLibraries/opencv/build") 
    # set(PCL_PATH "/home/user/ExternalLibraries/PCL1.14.1")      
endif()

message("OpenCV_DIR: ${OpenCV_DIR}")
message("PCL_PATH: ${PCL_PATH}")
