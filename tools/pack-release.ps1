# 把 build-msys2 的 Release 产物打成一个可拷走的便携目录 + zip。
#
# 用法（在项目根目录）：
#   powershell -ExecutionPolicy Bypass -File tools\pack-release.ps1
#   powershell -ExecutionPolicy Bypass -File tools\pack-release.ps1 -ZipPath D:\cloud_viewer_rel.zip
#
# 前置条件：已经用 build-win-msys2.sh 编译过，build-msys2\cloud_viewer.exe 存在。
# 产物：<项目>\dist-release\（目录）、<项目>\..\cloud_viewer_rel.zip（压缩包）

param(
    [string]$BuildDir = "build-msys2",
    [string]$StageDir = "dist-release",
    [string]$ZipPath  = ""
)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"   # Compress-Archive 的进度条会刷屏
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$exe = Join-Path $BuildDir "cloud_viewer.exe"
if (-not (Test-Path $exe)) {
    throw "找不到 $exe，请先运行 build-win-msys2.sh build"
}

if ([string]::IsNullOrWhiteSpace($ZipPath)) {
    $ZipPath = Join-Path (Split-Path -Parent $root) "cloud_viewer_rel.zip"
}

if (Test-Path $StageDir) { Remove-Item $StageDir -Recurse -Force }
New-Item -ItemType Directory -Path $StageDir, "$StageDir\calib" -Force | Out-Null

# 1) 主程序
Copy-Item $exe $StageDir

# 2) 依赖 DLL：取 build-msys2 与 dist-msys2 的并集。
#    构建目录里的 post-build 只拷 VTK 那一批（约 56 个），
#    缺 libstdc++-6 / libfreetype-6 / libpng16-16 等运行时依赖，必须靠 dist-msys2 补齐。
$dlls = @{}
foreach ($dir in @($BuildDir, "dist-msys2")) {
    if (Test-Path $dir) {
        Get-ChildItem $dir -Filter *.dll -File | ForEach-Object { $dlls[$_.Name] = $_.FullName }
    }
}
if ($dlls.Count -eq 0) { throw "build-msys2 与 dist-msys2 里都没有 DLL" }
$dlls.Values | ForEach-Object { Copy-Item $_ $StageDir -Force }
Write-Host ("DLL: {0} 个" -f $dlls.Count)

# 3) 标定表 + 本地配置 + SSH 客户端（来自 dist-msys2，属运行时数据不进版本库）
Copy-Item "dist-msys2\calib\cs96beta_calibration.csv" "$StageDir\calib\" -ErrorAction SilentlyContinue
if (-not (Test-Path "$StageDir\calib\cs96beta_calibration.csv")) {
    Write-Warning "没有找到 calib\cs96beta_calibration.csv，包里将缺少标定表"
}
foreach ($f in @("cloud_viewer.ini", "plink.exe")) {
    $src = Join-Path "dist-msys2" $f
    if (Test-Path $src) { Copy-Item $src $StageDir } else { Write-Warning "dist-msys2 里缺少 $f" }
}

# 4) 启动器与说明（deploy\ 受版本管理，改文案请改这里）
foreach ($f in Get-ChildItem "deploy" -File) {
    Copy-Item $f.FullName $StageDir
}

# 5) 依赖闭环自检：非系统 DLL 必须都在包里
$objdump = $null
$candidates = @(
    "$env:CV_MSYS2_ROOT\mingw64\bin\objdump.exe",
    "C:\Repos_Win11_C\Toolchain\msys64\mingw64\bin\objdump.exe"
)
foreach ($c in $candidates) { if ($c -and (Test-Path $c)) { $objdump = $c; break } }

if ($objdump) {
    $imports = @{}
    foreach ($bin in @(Get-ChildItem "$StageDir\*" -Include *.exe, *.dll -File)) {
        & $objdump -p $bin.FullName 2>$null |
            Select-String 'DLL Name: ' |
            ForEach-Object { $imports[$_.Line.Split(':')[1].Trim().ToLower()] = $true }
    }
    $have = @{}
    Get-ChildItem $StageDir -Filter *.dll -File | ForEach-Object { $have[$_.Name.ToLower()] = $true }
    # objdump 输出的是 "KERNEL32.dll" 形式，白名单统一补上 .dll 再比。
    $system = 'kernel32','user32','gdi32','advapi32','shell32','ole32','comdlg32','opengl32',
              'ws2_32','wsock32','iphlpapi','psapi','rpcrt4','usp10','msvcrt','d3d11','dxgi',
              'dwrite','d3dcompiler_47','version','oleaut32','shlwapi','imm32','winmm','crypt32',
              'setupapi','dwmapi','uxtheme','mpr','uuid','netapi32','pdh','secur32','bcrypt',
              'ntdll','msimg32','oleacc','windowscodecs','wtsapi32','cfgmgr32','powrprof',
              'dnsapi','winhttp','wininet','gdiplus','comctl32','dbghelp'
    $system = $system | ForEach-Object { "$_.dll" }
    $missing = $imports.Keys | Where-Object { -not $have[$_] -and ($system -notcontains $_) }
    if ($missing) {
        # 缺依赖的包拷到别的机器必定起不来，这里直接失败而不是留个警告。
        throw ("以下依赖不在包里，也不是已知系统 DLL: " + ($missing -join ', ') +
               "`n请把这些 DLL 从工具链 mingw64\bin 补到 dist-msys2 后重试。")
    }
    Write-Host "依赖闭环自检: 通过（所有非系统依赖都在包内）"
} else {
    Write-Warning "找不到 objdump，跳过依赖自检"
}

# 6) 压缩
if (Test-Path $ZipPath) { Remove-Item $ZipPath -Force }
Compress-Archive -Path "$StageDir\*" -DestinationPath $ZipPath -CompressionLevel Optimal
$size = (Get-Item $ZipPath).Length / 1MB
Write-Host ""
Write-Host ("目录: {0}" -f (Resolve-Path $StageDir))
Write-Host ("压缩包: {0}  ({1:N1} MB)" -f (Resolve-Path $ZipPath), $size)
