@echo off
chcp 65001 >nul
setlocal
echo ============================================
echo   cloud_viewer 首次使用初始化
echo ============================================
echo.

set CONF_DIR=%USERPROFILE%\.config\cloud_viewer
if not exist "%CONF_DIR%" mkdir "%CONF_DIR%"

if exist "%CONF_DIR%\beta_ssh.conf" (
    echo [SKIP] 已存在 %CONF_DIR%\beta_ssh.conf
) else (
    > "%CONF_DIR%\beta_ssh.conf" echo BETA_SSH_PASS=111
    echo [OK]   已写入 %CONF_DIR%\beta_ssh.conf ^(雷达默认密码 111^)
)
echo.

echo 放行 UDP 55000/50007 入站（需要管理员权限，失败不影响广播发现）：
net session >nul 2>&1
if errorlevel 1 (
    echo [SKIP] 当前不是管理员，跳过防火墙规则。
    echo        首次启动如弹出"Windows 安全警报"，请点"允许访问"。
) else (
    netsh advfirewall firewall add rule name="cloud_viewer UDP" dir=in action=allow protocol=UDP localport=55000,50007 >nul
    echo [OK]   已添加防火墙规则 cloud_viewer UDP
)

echo.
echo 初始化完成。下一步：双击 启动cloud_viewer.bat
echo.
pause >nul
