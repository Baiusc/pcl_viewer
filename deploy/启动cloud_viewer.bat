@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo ============================================
echo   cloud_viewer Lite - UTF-8 console
echo ============================================
echo.
cloud_viewer.exe %*
echo.
echo [Finished, exit code = %ERRORLEVEL%]
echo Press any key to close this window...
pause >nul
