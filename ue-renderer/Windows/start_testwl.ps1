# testwl Shipping 启动脚本
# 用法: .\start_testwl.ps1 [-Mode headless|windowed|fullscreen|perf] [-Map <map_path>] [-ExtraArgs <args>]
#
# 示例:
#   .\start_testwl.ps1                          # 默认 headless 模式
#   .\start_testwl.ps1 -Mode windowed           # 窗口模式
#   .\start_testwl.ps1 -Mode perf -ExtraArgs "-capturemode=everyframe"
#   .\start_testwl.ps1 -Mode headless -Map /Env_MultiBS_Data/Maps/Map_MultiBS.Map_MultiBS

param(
    [ValidateSet('headless', 'windowed', 'fullscreen', 'perf')]
    [string]$Mode = 'headless',

    [string]$Map = '/Env_MultiBS_Data/Maps/Map_MultiBS.Map_MultiBS',

    [string]$ExtraArgs = ''
)

$ErrorActionPreference = 'Stop'

$ScriptDir = $PSScriptRoot
$ExePath = Join-Path $ScriptDir 'testwl.exe'

if (-not (Test-Path $ExePath)) {
    Write-Host "[ERROR] 找不到 testwl.exe: $ExePath" -ForegroundColor Red
    exit 1
}

$baseArgs = @($Map)

switch ($Mode) {
    'headless' {
        $baseArgs += @('-windowed', '-resx=1', '-resy=1', '-renderoffscreen', '-nosound', '-minimalviewport')
        Write-Host "[INFO] 模式: Headless (离屏渲染, 无声音, 最小视口)" -ForegroundColor Green
    }
    'windowed' {
        $baseArgs += @('-windowed', '-resx=1024', '-resy=768')
        Write-Host "[INFO] 模式: Windowed (1024x768)" -ForegroundColor Green
    }
    'fullscreen' {
        $baseArgs += @('-fullscreen')
        Write-Host "[INFO] 模式: Fullscreen" -ForegroundColor Green
    }
    'perf' {
        $baseArgs += @(
            '-windowed', '-resx=1', '-resy=1',
            '-renderoffscreen', '-nosound', '-minimalviewport',
            '-NoTextureStreaming', '-nomansky',
            '-nopauseonbackgroundtaskloss'
        )
        Write-Host "[INFO] 模式: Perf (性能优先)" -ForegroundColor Green
    }
}

Write-Host "[INFO] 地图: $Map"
Write-Host "[INFO] 可执行文件: $ExePath"

if (-not [string]::IsNullOrEmpty($ExtraArgs)) {
    Write-Host "[INFO] 额外参数: $ExtraArgs"
    $allArgs = $baseArgs + ($ExtraArgs -split '\s+')
} else {
    $allArgs = $baseArgs
}

Write-Host "`n[INFO] 启动中, 首次加载可能需要数分钟...`n" -ForegroundColor Cyan

& $ExePath $allArgs

$exitCode = $LASTEXITCODE
if ($exitCode -ne 0) {
    Write-Host "`n[WARN] 进程退出码: $exitCode" -ForegroundColor Yellow
}
