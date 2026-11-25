@echo off
setlocal

:: 设置Qt路径（根据实际安装路径修改）
set QT_DIR=C:\Users\11042\Qt6\6.10.2\msvc2022_64
set PATH=%QT_DIR%\bin;%PATH%

:: 设置应用程序路径（根据实际路径修改）
set APP_PATH=..\out\build\ninja-x64-debug
set DEPLOY_DIR=.\YSCloudScene
:: 创建部署目录
if not exist %DEPLOY_DIR% mkdir %DEPLOY_DIR%

:: 复制主程序和相关dll
copy "%APP_PATH%\*.exe" "%DEPLOY_DIR%"
copy "%APP_PATH%\*.dll" "%DEPLOY_DIR%"

:: 运行 windeployqt
windeployqt.exe %DEPLOY_DIR%\YSCloudScene.exe

:: 复制设备属性配置目录（属性树按型号加载）
if exist "%APP_PATH%\lidar_properties_config" (
    xcopy /E /I /Y "%APP_PATH%\lidar_properties_config" "%DEPLOY_DIR%\lidar_properties_config" >nul
)

:: 复制 Visual C++ 运行时库（如果需要）
copy "C:\Windows\system32\kernel32.dll" %DEPLOY_DIR%
copy "C:\Windows\system32\MSVCP140D.dll" %DEPLOY_DIR%
copy "C:\Windows\system32\MSVCP140_1D.dll" %DEPLOY_DIR%
copy "C:\Windows\system32\MSVCP140_2D.dll" %DEPLOY_DIR%
copy "C:\Windows\system32\WS2_32.dll" %DEPLOY_DIR%
copy "C:\Windows\system32\IPHLPAPI.DLL" %DEPLOY_DIR%
copy "C:\Windows\system32\VCRUNTIME140D.dll" %DEPLOY_DIR%
copy "C:\Windows\system32\VCRUNTIME140_1D.dll" %DEPLOY_DIR%
copy "C:\Windows\system32\ucrtbased.dll" %DEPLOY_DIR%

echo Deployment completed!
pause