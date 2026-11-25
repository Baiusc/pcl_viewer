# 配置 repo 子仓库的 git 排除规则，避免 SourceTree 显示未跟踪文件
# 用法: 在 repo init && repo sync 后运行此脚本
# 运行: powershell -ExecutionPolicy Bypass -File setup_repo_exclude.ps1

$excludeFile = ".git\info\exclude"
$excludeDir = ".git\info"

if (-not (Test-Path $excludeDir)) {
    New-Item -ItemType Directory -Path $excludeDir -Force | Out-Null
}

$marker = "# Repo 管理的子仓库"
$content = @"

$marker
you_see_sdk/
algorithm/repair_ground/
algorithm/algo_solid_lidar/
matrix_lidar_sdk/
you_lib/
lumosy/
"@

$existing = Get-Content $excludeFile -Raw -ErrorAction SilentlyContinue
if ($existing -match [regex]::Escape($marker)) {
    Write-Output "已配置过，跳过。"
} else {
    Add-Content -Path $excludeFile -Value $content
    Write-Output "已配置 .git/info/exclude，子仓库目录将不再显示为未跟踪文件。"
}
