@echo off
rem ---------------------------------------------------------------------------
rem Local MSVC + Ninja build entry (Windows).
rem
rem Loads vcvars64 then drives an existing build dir. CMake's POST_BUILD rule
rem copies the exe and runtime DLLs to dist-local, so dist-local is the only
rem directory you need to run.
rem
rem usage:
rem   tools\build-win-msvc.bat                  build default target in build-ninja
rem   tools\build-win-msvc.bat build-ninja all  build every target
rem
rem env:
rem   CV_VCVARS  path to vcvars64.bat (default VS2022 BuildTools)
rem   CV_NINJA   path to ninja.exe   (default C:/msys64/usr/bin/ninja.exe)
rem ---------------------------------------------------------------------------
setlocal

set "BUILD_DIR=%~1"
if "%BUILD_DIR%"=="" set "BUILD_DIR=%~dp0..\build-ninja"

set "TARGET=%~2"
if "%TARGET%"=="" set "TARGET=cloud_viewer"

if "%CV_VCVARS%"=="" set "CV_VCVARS=C:/Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%CV_VCVARS%" (
    echo [ERR] vcvars64.bat not found: "%CV_VCVARS%"
    echo       set CV_VCVARS to your local vcvars64.bat
    exit /b 1
)

call "%CV_VCVARS%" >nul
if errorlevel 1 (
    echo [ERR] vcvars64.bat failed
    exit /b 1
)

if "%CV_NINJA%"=="" set "CV_NINJA=C:/msys64/usr/bin/ninja.exe"
if not exist "%CV_NINJA%" set "CV_NINJA=ninja"

echo [build] dir=%BUILD_DIR% target=%TARGET%
"%CV_NINJA%" -C "%BUILD_DIR%" %TARGET%
exit /b %ERRORLEVEL%
