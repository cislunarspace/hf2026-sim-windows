# start.ps1 — OpenSim release 包一键启动（Windows 版）
# 启动 Redis + bridge + 前端静态服务。引擎和 UE 不在此启动 ——
# 用户在前端点赛题时由 competition/bridge 按需 spawn。
# 复刻 start.sh 的「只起前端基础设施」模型。

#Requires -Version 5.1
[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# 推导包根（脚本所在目录）
$PACK_ROOT = $PSScriptRoot
Set-Location $PACK_ROOT

# 端口可配（默认 + env 覆盖）
$OPENSIM_REDIS_PORT = if ($env:OPENSIM_REDIS_PORT) { [int]$env:OPENSIM_REDIS_PORT } else { 6379 }
$OPENSIM_WS_PORT    = if ($env:OPENSIM_WS_PORT)    { [int]$env:OPENSIM_WS_PORT }    else { 8080 }
$OPENSIM_CAM_PORT   = if ($env:OPENSIM_CAM_PORT)   { [int]$env:OPENSIM_CAM_PORT }   else { 8081 }
$OPENSIM_CAM_WS_PORT = if ($env:OPENSIM_CAM_WS_PORT) { [int]$env:OPENSIM_CAM_WS_PORT } else { 8082 }
$OPENSIM_WEB_PORT   = if ($env:OPENSIM_WEB_PORT)   { [int]$env:OPENSIM_WEB_PORT }   else { 3000 }

$RUN_DIR  = Join-Path $PACK_ROOT 'run'
$LOG_DIR  = Join-Path $RUN_DIR 'logs'
$PID_DIR  = Join-Path $RUN_DIR 'pids'

$dirs = @($LOG_DIR, $PID_DIR, (Join-Path $RUN_DIR 'redis'))
foreach ($d in $dirs) {
    if (-not (Test-Path $d)) { New-Item -ItemType Directory -Path $d -Force | Out-Null }
}

# ── 工具函数 ──
function Test-PortInUse {
    param([int]$Port)
    try {
        $conn = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction Stop
        return $null -ne $conn -and @($conn).Count -gt 0
    } catch {
        return $false
    }
}

function Pick-FreePort {
    # -Taken 传入本脚本此前已分配的端口集合。Pick-FreePort 只检测系统 listen 端口,
    # 看不到"本脚本即将启动但还没 listen"的端口;若不带上 -Taken,5 个端口会被
    # 分到同一空闲端口(如 8080/8081 被系统占用时,WS 与 CAM_WS 都会落在 8082,
    # bridge 启动后 CameraWs listen 报 EADDRINUSE,相机画面起不来)。
    param([int]$Start, [string]$Label, [int[]]$Taken = @())
    $p = $Start
    while (((Test-PortInUse -Port $p) -or ($Taken -contains $p)) -and $p -lt ($Start + 100)) {
        $p++
    }
    if ((Test-PortInUse -Port $p) -or ($Taken -contains $p)) {
        Write-Host "✗ $Label 端口 ${Start}~$($Start + 100) 全被占用，请用环境变量指定" -ForegroundColor Red
        exit 1
    }
    if ($p -ne $Start) {
        Write-Host "  $Label 端口 $Start 被占用，改用 $p" -ForegroundColor Yellow
    }
    return $p
}

function Test-PidAlive {
    param([int]$ProcessId)
    if (-not $ProcessId) { return $false }
    $null = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    return $?
}

function Stop-Pid {
    param([int]$ProcessId, [switch]$Force)
    if (-not $ProcessId) { return }
    try {
        if ($Force) {
            Stop-Process -Id $ProcessId -Force -ErrorAction SilentlyContinue
        } else {
            Stop-Process -Id $ProcessId -ErrorAction SilentlyContinue
        }
    } catch {}
}

function Get-PythonExe {
    # 优先使用发行包内捆绑的 Python,实现不依赖目标机系统 Python。
    $bundled = Join-Path $PSScriptRoot 'python\python.exe'
    if (Test-Path $bundled) { return $bundled }

    # 回退:目标机系统 Python(兼容旧包/开发场景)
    $candidates = @(
        'C:\Python314\python.exe',
        'C:\Python313\python.exe',
        'C:\Python312\python.exe',
        'C:\Python311\python.exe'
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) { return $c }
    }
    if (Get-Command python -ErrorAction SilentlyContinue) { return 'python' }
    if (Get-Command py -ErrorAction SilentlyContinue)     { return 'py' }
    return $null
}

# ── 0. 关闭本包的现存进程（上次异常退出留下的孤儿） ──
Write-Host ''
Write-Host '[0/4] 清理现存进程...'

# pidfile 记录的进程
$pidFiles = @(Get-ChildItem -Path $PID_DIR -Filter '*.pid' -File -ErrorAction SilentlyContinue)
foreach ($pf in $pidFiles) {
    $pidVal = 0
    if ([int]::TryParse((Get-Content $pf.FullName -Raw).Trim(), [ref]$pidVal)) {
        if (Test-PidAlive -Pid $pidVal) {
            Write-Host "  停止残留 $($pf.BaseName) (PID $pidVal)"
            Stop-Pid -Pid $pidVal
        }
    }
    Remove-Item $pf.FullName -Force -ErrorAction SilentlyContinue
}

# 外部残留进程（bridge / opensim-sim / 本包 static-server）
$allProcs = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue
$patterns = @(
    'dist-bridge[\\/]bridge[\\/]index\.js',
    'opensim-sim',
    'static-server\.js.*frontend'
)
foreach ($pat in $patterns) {
    $matched = $allProcs | Where-Object { $_.CommandLine -and $_.CommandLine -match $pat }
    if ($matched) {
        $pidArr = @()
        foreach ($mm in $matched) { $pidArr += $mm.ProcessId.ToString() }
        $pidList = $pidArr -join ', '
        Write-Host "  清理残留 $pat (PID: $pidList)"
        foreach ($m in $matched) {
            try { Stop-Process -Id $m.ProcessId -Force -ErrorAction SilentlyContinue } catch {}
        }
    }
}

# UE 孤儿进程（按 config/renderers/*.json 的 workdir 清理，跳过 template 与占位符）
$renderersDir = Join-Path $PACK_ROOT 'config\renderers'
if (Test-Path $renderersDir) {
    $renderers = @(Get-ChildItem -Path $renderersDir -Filter '*.json' -File -ErrorAction SilentlyContinue)
    foreach ($r in $renderers) {
        if ($r.Name -like '*.template.json') { continue }
        $workdir = $null
        try {
            $cfg = Get-Content $r.FullName -Raw | ConvertFrom-Json
            $workdir = $cfg.executable.workdir
        } catch {}
        if (-not $workdir) { continue }
        if ($workdir -like '<*>') { continue }  # 占位符跳过

        # Windows 无 /proc，用 CIM 查 CommandLine 匹配 workdir
        $procs = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue
        foreach ($p in $procs) {
            if (-not $p.CommandLine) { continue }
            if ($p.CommandLine -match [regex]::Escape($workdir)) {
                Write-Host "  清理 UE 孤儿 (PID $($p.ProcessId), cwd $workdir)"
                try { Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue } catch {}
            }
        }
    }
}
Start-Sleep -Seconds 1

# ── 环境检测 ──
$python = Get-PythonExe
if (-not $python) {
    Write-Host '✗ 缺少 Python。请先运行: .\setup.ps1' -ForegroundColor Red
    exit 1
}
$pyOk = & $python -c 'import redis, yaml' 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host '✗ 缺少 Python 依赖（redis/pyyaml）。请先运行: .\setup.ps1' -ForegroundColor Red
    exit 1
}

# ── 端口冲突处理 ──
# Pick-FreePort 的 -Taken 传入此前已分配的端口,保证 REDIS/WS/CAM/CAMWS/WEB
# 五个端口两两不撞(否则 bridge 内多个 listen 会互相 EADDRINUSE,如 CAM_WS 撞 WS
# 时相机画面通道起不来、仿真却照常 —— 症状与根因隔了好几层,极难排查)。
$assigned = @()
$OPENSIM_REDIS_PORT  = Pick-FreePort -Start $OPENSIM_REDIS_PORT  -Label 'REDIS' -Taken $assigned; $assigned += $OPENSIM_REDIS_PORT
$OPENSIM_WS_PORT     = Pick-FreePort -Start $OPENSIM_WS_PORT     -Label 'WS'    -Taken $assigned; $assigned += $OPENSIM_WS_PORT
$OPENSIM_CAM_PORT    = Pick-FreePort -Start $OPENSIM_CAM_PORT    -Label 'CAM'   -Taken $assigned; $assigned += $OPENSIM_CAM_PORT
$OPENSIM_CAM_WS_PORT = Pick-FreePort -Start $OPENSIM_CAM_WS_PORT -Label 'CAMWS' -Taken $assigned; $assigned += $OPENSIM_CAM_WS_PORT
$OPENSIM_WEB_PORT    = Pick-FreePort -Start $OPENSIM_WEB_PORT    -Label 'WEB'   -Taken $assigned; $assigned += $OPENSIM_WEB_PORT

# ── 同步 redis_port 到 scenario.json ──
# 引擎从 scenario.json 读 redis_port，不读环境变量；若不同步，两端在不同 Redis 上。
# 先备份原始 scenario.json，stop.ps1 退出时恢复，避免污染 release 包原始文件。
Write-Host "  同步 redis_port=$OPENSIM_REDIS_PORT 到 scenario.json..."
$backupDir = Join-Path $RUN_DIR 'scenario-backup'
if (-not (Test-Path $backupDir)) { New-Item -ItemType Directory -Path $backupDir -Force | Out-Null }
$scenarioDirs = @(Get-ChildItem -Path (Join-Path $PACK_ROOT 'competition\scenarios') -Directory -ErrorAction SilentlyContinue)
foreach ($sd in $scenarioDirs) {
    $sj = Join-Path $sd.FullName 'scenario.json'
    if (-not (Test-Path $sj)) { continue }
    # 备份原始文件（仅首次，避免覆盖原始备份）
    $backupFile = Join-Path $backupDir "$($sd.Name).scenario.json.bak"
    if (-not (Test-Path $backupFile)) {
        Copy-Item $sj $backupFile -Force
    }
    try {
        # Get-Content 默认按 BOM/UTF-8 自动识别编码读取。
        $cfg = Get-Content $sj -Raw | ConvertFrom-Json
        if (-not $cfg.simulation) {
            $cfg | Add-Member -NotePropertyName 'simulation' -NotePropertyValue ([PSCustomObject]@{})
        }
        $cfg.simulation | Add-Member -NotePropertyName 'redis_port' -NotePropertyValue $OPENSIM_REDIS_PORT -Force
        # 注意: Set-Content -Encoding UTF8 在 PS 5.1 写入 UTF-8 *with BOM*。
        # scenario.json 随后由 Python competition runner 用 json.loads(read_text(
        # encoding="utf-8")) 读取 —— Python 的 "utf-8" 不容忍 BOM,会抛
        # JSONDecodeError,导致 prepared scenario 写成 {} ,引擎报 "no entity"
        # 启动失败(曾引发 sim 黑框 + 无画面回归)。改用 .NET WriteAllText +
        # UTF8Encoding($false) 写无 BOM 的 UTF-8。
        $jsonText = $cfg | ConvertTo-Json -Depth 10
        [System.IO.File]::WriteAllText($sj, $jsonText, (New-Object System.Text.UTF8Encoding($false)))
    } catch {
        Write-Host "    ⚠️  改写 $sj 失败（手动检查 redis_port）" -ForegroundColor Yellow
    }
}

# ── 写 run/env.ps1（供 stop.ps1 / verify.ps1 dot-source） ──
$envContent = @"
`$env:OPENSIM_REDIS_PORT = '$OPENSIM_REDIS_PORT'
`$env:OPENSIM_WS_PORT = '$OPENSIM_WS_PORT'
`$env:OPENSIM_CAM_PORT = '$OPENSIM_CAM_PORT'
`$env:OPENSIM_CAM_WS_PORT = '$OPENSIM_CAM_WS_PORT'
`$env:OPENSIM_WEB_PORT = '$OPENSIM_WEB_PORT'
"@
# 用 .NET WriteAllText + UTF8Encoding($false) 写无 BOM,保持仓库约定(见 CLAUDE.md 坑清单#2)
$envPath = Join-Path $RUN_DIR 'env.ps1'
[System.IO.File]::WriteAllText($envPath, $envContent, (New-Object System.Text.UTF8Encoding($false)))

# 进程存活检测（幂等：已起则跳过）
function Test-Alive {
    param([string]$PidFile)
    if (-not (Test-Path $PidFile)) { return $false }
    $pidVal = 0
    if (-not [int]::TryParse((Get-Content $PidFile -Raw).Trim(), [ref]$pidVal)) { return $false }
    return Test-PidAlive -ProcessId $pidVal
}

Write-Host ''
Write-Host '=== OpenSim 启动中 ==='

# ── 1. Redis（纯内存模式；redis-windows 不支持 --daemonize，用 Start-Process 后台启动） ──
$redisPidFile = Join-Path $PID_DIR 'redis.pid'
if (-not (Test-Alive -PidFile $redisPidFile)) {
    Write-Host "[1/4] 启动 Redis (port $OPENSIM_REDIS_PORT, 纯内存)..."
    $redisServer = Join-Path $PACK_ROOT 'bin\redis-server.exe'
    $redisCli    = Join-Path $PACK_ROOT 'bin\redis-cli.exe'
    if (-not (Test-Path $redisServer)) {
        Write-Host "✗ Redis 二进制缺失: $redisServer" -ForegroundColor Red
        exit 1
    }
    $redisLog = Join-Path $LOG_DIR 'redis.log'
    $redisDir = Join-Path $RUN_DIR 'redis'
    $redisArgs = @(
        '--port', $OPENSIM_REDIS_PORT.ToString(),
        '--pidfile', $redisPidFile,
        '--logfile', $redisLog,
        '--dir', $redisDir,
        '--save', '""',
        '--appendonly', 'no'
    )
    $redisProc = Start-Process -FilePath $redisServer -ArgumentList $redisArgs `
        -NoNewWindow -PassThru -RedirectStandardOutput $redisLog -RedirectStandardError "$redisLog.err"
    $redisProc.Id | Out-File -FilePath $redisPidFile -Encoding ASCII -NoNewline

    # 轮询 redis-cli ping 直到 PONG
    $pong = $false
    for ($i = 1; $i -le 20; $i++) {
        $result = & $redisCli -p $OPENSIM_REDIS_PORT ping 2>$null
        if ($result -eq 'PONG') { $pong = $true; break }
        Start-Sleep -Milliseconds 250
    }
    if (-not $pong) {
        Write-Host '✗ Redis 启动失败' -ForegroundColor Red
        exit 1
    }
} else {
    Write-Host '[1/4] Redis 已在运行，跳过'
}

# ── 2. bridge ──
$bridgePidFile = Join-Path $PID_DIR 'bridge.pid'
if (-not (Test-Alive -PidFile $bridgePidFile)) {
    Write-Host "[2/4] 启动 bridge (WS :$OPENSIM_WS_PORT, CAM :$OPENSIM_CAM_PORT, CAMWS :$OPENSIM_CAM_WS_PORT)..."
    $nodeExe = Join-Path $PACK_ROOT 'bin\node.exe'
    $bridgeJs = Join-Path $PACK_ROOT 'visualization\dist-bridge\bridge\index.js'
    if (-not (Test-Path $nodeExe)) {
        Write-Host "✗ Node 二进制缺失: $nodeExe" -ForegroundColor Red
        exit 1
    }
    if (-not (Test-Path $bridgeJs)) {
        Write-Host "✗ bridge 编译产物缺失: $bridgeJs" -ForegroundColor Red
        exit 1
    }
    # 设置环境变量
    $env:NODE_PATH = Join-Path $PACK_ROOT 'lib\node_modules'
    $env:OPENSIM_SIM_BIN = Join-Path $PACK_ROOT 'opensim-sim.exe'
    $env:OPENSIM_RENDERERS_DIR = Join-Path $PACK_ROOT 'config\renderers'
    $env:OPENSIM_SCENARIOS_DIR = Join-Path $PACK_ROOT 'competition\scenarios'
    $env:PYTHON_BIN = $python
    $env:WS_PORT = $OPENSIM_WS_PORT.ToString()
    $env:CAM_HTTP_PORT = $OPENSIM_CAM_PORT.ToString()
    $env:CAM_WS_PORT = $OPENSIM_CAM_WS_PORT.ToString()
    $env:REDIS_HOST = '127.0.0.1'
    $env:REDIS_PORT = $OPENSIM_REDIS_PORT.ToString()
    # OPENSIM_RENDER_CTL_BIN 不设（opensim-render-ctl.exe 缺失，bridge 自动降级）

    $bridgeLog = Join-Path $LOG_DIR 'bridge.log'
    $bridgeErr = Join-Path $LOG_DIR 'bridge.err'
    $bridgeProc = Start-Process -FilePath $nodeExe -ArgumentList $bridgeJs `
        -NoNewWindow -PassThru -RedirectStandardOutput $bridgeLog -RedirectStandardError $bridgeErr
    $bridgeProc.Id | Out-File -FilePath $bridgePidFile -Encoding ASCII -NoNewline

    # 等待 bridge HTTP 端口起来
    $bridgeReady = $false
    for ($i = 1; $i -le 40; $i++) {
        try {
            $resp = Invoke-WebRequest -Uri "http://127.0.0.1:$OPENSIM_CAM_PORT/api/sim/status" -UseBasicParsing -TimeoutSec 1 -ErrorAction Stop
            if ($resp.StatusCode -eq 200) { $bridgeReady = $true; break }
        } catch {}
        Start-Sleep -Milliseconds 250
    }
    if (-not $bridgeReady) {
        Write-Host '⚠️  bridge HTTP 未就绪（可能仍在启动；查看 run\logs\bridge.log）' -ForegroundColor Yellow
    }
} else {
    Write-Host '[2/4] bridge 已在运行，跳过'
}

# ── 3. 前端静态服务 ──
$frontendPidFile = Join-Path $PID_DIR 'frontend.pid'
if (-not (Test-Alive -PidFile $frontendPidFile)) {
    Write-Host "[3/4] 启动前端静态服务 (:$OPENSIM_WEB_PORT)..."
    $nodeExe = Join-Path $PACK_ROOT 'bin\node.exe'
    $staticSrv = Join-Path $PACK_ROOT 'static-server.js'
    $frontendDir = Join-Path $PACK_ROOT 'frontend'
    $frontendLog = Join-Path $LOG_DIR 'frontend.log'
    $frontendErr = Join-Path $LOG_DIR 'frontend.err'

    $feProc = Start-Process -FilePath $nodeExe -ArgumentList $staticSrv, $frontendDir, $OPENSIM_WEB_PORT.ToString() `
        -NoNewWindow -PassThru -RedirectStandardOutput $frontendLog -RedirectStandardError $frontendErr
    $feProc.Id | Out-File -FilePath $frontendPidFile -Encoding ASCII -NoNewline

    # 等待前端起来
    $feReady = $false
    for ($i = 1; $i -le 20; $i++) {
        try {
            $resp = Invoke-WebRequest -Uri "http://127.0.0.1:$OPENSIM_WEB_PORT/" -UseBasicParsing -TimeoutSec 1 -ErrorAction Stop
            if ($resp.StatusCode -eq 200) { $feReady = $true; break }
        } catch {}
        Start-Sleep -Milliseconds 250
    }
    if (-not $feReady) {
        Write-Host '⚠️  前端未就绪（查看 run\logs\frontend.log）' -ForegroundColor Yellow
    }
} else {
    Write-Host '[3/4] 前端已在运行，跳过'
}

Write-Host ''
Write-Host '=========================================='
Write-Host '  OpenSim 已启动'
Write-Host '=========================================='
Write-Host "  浏览器访问: http://localhost:$OPENSIM_WEB_PORT"
Write-Host '  选赛题 → 「算法」框可填 module:Class（留空用 baseline）→ 点「开始仿真」'
Write-Host ''

# 自动打开浏览器（设 OPENSIM_NO_OPEN_BROWSER=1 可禁用）
if ($env:OPENSIM_NO_OPEN_BROWSER -ne '1') {
    $url = "http://localhost:$OPENSIM_WEB_PORT"
    try {
        Start-Process $url
        Write-Host "  已尝试打开浏览器：$url"
    } catch {
        Write-Host "  ⚠️  无法打开浏览器，请手动访问 $url" -ForegroundColor Yellow
    }
}
Write-Host ''
Write-Host "  日志: Get-Content $LOG_DIR\bridge.log -Wait"
Write-Host '  停止: .\stop.ps1'
Write-Host '  检查: .\verify.ps1'
Write-Host '=========================================='
