@echo off
REM ============================================================
REM  testwl Shipping 启动脚本
REM  用法: start_testwl.bat [mode] [extra_args...]
REM
REM  mode:
REM    headless   (默认) 离屏渲染, 无声音, 最小视口, 适合仿真
REM    windowed   窗口模式 1024x768, 带声音
REM    fullscreen 全屏模式
REM    perf       性能测试模式 (离屏 + 高帧率优先)
REM
REM  示例:
REM    start_testwl.bat
REM    start_testwl.bat headless
REM    start_testwl.bat windowed
REM    start_testwl.bat perf -captureconfig=scenario_perf_6cam.json
REM ============================================================

setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "EXE=%SCRIPT_DIR%testwl.exe"

if not exist "%EXE%" (
    echo [ERROR] 找不到 testwl.exe: %EXE%
    pause
    exit /b 1
)

REM ── 解析模式参数 ──
set "MODE=%~1"
if "%MODE%"=="" set "MODE=headless"

shift

set "EXTRA_ARGS=%*"

REM ── 基础参数: 默认地图 ──
set "MAP=/Env_MultiBS_Data/Maps/Map_MultiBS.Map_MultiBS"

REM ── 按模式配置参数 ──
if /i "%MODE%"=="headless" (
    set "ARGS=%MAP% -windowed -resx=1 -resy=1 -renderoffscreen -nosound -minimalviewport"
    echo [INFO] 模式: Headless (离屏渲染, 无声音)
) else if /i "%MODE%"=="windowed" (
    set "ARGS=%MAP% -windowed -resx=1024 -resy=768"
    echo [INFO] 模式: Windowed (1024x768)
) else if /i "%MODE%"=="fullscreen" (
    set "ARGS=%MAP% -fullscreen"
    echo [INFO] 模式: Fullscreen
) else if /i "%MODE%"=="perf" (
    set "ARGS=%MAP% -windowed -resx=1 -resy=1 -renderoffscreen -nosound -minimalviewport -NoTextureStreaming -nomansky -nopauseonbackgroundtaskloss"
    echo [INFO] 模式: Perf (性能优先)
) else (
    echo [WARN] 未知模式 "%MODE%", 使用默认 headless
    set "ARGS=%MAP% -windowed -resx=1 -resy=1 -renderoffscreen -nosound -minimalviewport"
)

REM ── 追加额外参数 ──
if not "%EXTRA_ARGS%"=="" (
    set "ARGS=%ARGS% %EXTRA_ARGS%"
)

echo [INFO] 可执行文件: %EXE%
echo [INFO] 启动参数: %ARGS%
echo [INFO] 启动中, 首次加载可能需要数分钟, 请耐心等待...
echo.

"%EXE%" %ARGS%

endlocal
