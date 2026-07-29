# run/run_fast.ps1 — 快速迭代验证：headless 短局仿真
#
# 背景：引擎固定 ~0.7x 实时（time_scale 无效，实验证实），600s 整局约 14 分钟。
# 快速迭代只能靠：缩短 -Duration + 复用引擎省 ~25s 冷启动。
#
# 用法：
#   powershell -File run/run_fast.ps1                          # coop_decoy 180s，新起引擎（干净状态）
#   powershell -File run/run_fast.ps1 -Duration 60 -ReuseEngine  # 复用已运行引擎，迭代最快
#   powershell -File run/run_fast.ps1 -Scenario search_track `
#       -Agent competition.user_algorithms.search_track.my_agent:MySearchTrackAgent
#
# 注意：-ReuseEngine 不重置引擎状态（UAV/车辆位置、sim_time 延续），
# 适合看行为、调逻辑；要可比对的得分，请用默认模式（每次新起引擎）。
param(
    [string]$Scenario = 'coop_decoy',
    [string]$Agent = 'competition.user_algorithms.coop_decoy.agent:CoopDecoyAgent',
    [int]$Duration = 180,
    [int]$Seed = -1,
    [switch]$ReuseEngine,
    [string]$Output = "output/fast_$Scenario"
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$redisPort = 6382
$python = Join-Path $root '.venv\Scripts\python.exe'
$redisCli = Join-Path $root 'bin\redis-cli.exe'

# 官方 scenario.json 保持原样（redis_port 6381）。本地端口适配放在
# output/ 下的副本里：复制官方 scenario 并把 redis_port 改成本地端口，
# 引擎 --config 与 CLI --scenario-json 都指向副本。
$officialScenario = Join-Path $root "competition\scenarios\$Scenario\scenario.json"
New-Item -ItemType Directory -Force $Output | Out-Null
$scenarioJson = Join-Path $root "$Output\scenario.local.json"
$text = [System.IO.File]::ReadAllText($officialScenario, [System.Text.Encoding]::UTF8)
$text = $text -replace '"redis_port":\s*\d+', "`"redis_port`": $redisPort"
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($scenarioJson, $text, $utf8NoBom)

# 1. Redis（引擎从 scenario 副本读端口 6382，必须与之一致）
if ((& $redisCli -p $redisPort ping 2>$null) -ne 'PONG') {
    Write-Host "[run_fast] 启动 Redis :$redisPort"
    Start-Process -FilePath (Join-Path $root 'bin\redis-server.exe') `
        -ArgumentList '--port', "$redisPort" -WindowStyle Hidden
    Start-Sleep 2
}

# 2. 引擎
$simArgs = @('--start-sim')
if ($ReuseEngine) {
    if (-not (Get-Process opensim-sim -ErrorAction SilentlyContinue)) {
        Write-Host "[run_fast] 启动引擎（复用模式，terrain 加载仅此次）"
        Start-Process -FilePath (Join-Path $root 'opensim-sim.exe') `
            -ArgumentList '--config', "`"$scenarioJson`"" -WindowStyle Hidden
        $ready = $false
        for ($i = 0; $i -lt 40 -and -not $ready; $i++) {
            Start-Sleep 3
            $ready = ((& $redisCli -p $redisPort pubsub numsub sim:commands |
                Select-Object -Last 1) -eq '1')
        }
        if (-not $ready) { throw '[run_fast] 引擎 120s 内未就绪' }
        Write-Host '[run_fast] 引擎就绪'
    }
    $simArgs = @('--no-start-sim')
}

# 3. 跑仿真（runner 结束会自己打印得分小结）
$cliArgs = @('-m', 'competition', 'run', '--scenario', $Scenario, '--agent', $Agent,
    '--scenario-json', $scenarioJson,
    '--duration', "$Duration", '--redis-port', "$redisPort",
    '--output', $Output) + $simArgs
if ($Seed -ge 0) { $cliArgs += @('--seed', "$Seed") }

Write-Host "[run_fast] 预计墙钟 ~$([int]($Duration / 0.7) + 15)s（sim ${Duration}s ÷ 0.7）"
$sw = [System.Diagnostics.Stopwatch]::StartNew()
& $python @cliArgs
$sw.Stop()
Write-Host ("[run_fast] 实际墙钟 {0:N1}s，评分见 {1}\*.evaluation.json" -f `
    $sw.Elapsed.TotalSeconds, $Output)
