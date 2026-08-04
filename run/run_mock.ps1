# run/run_mock.ps1 — 快速迭代：进程内 mock 仿真（无引擎/无 Redis）
#
# 背景：闭源引擎固定 ~0.7x 实时（time_scale 无效），600s 整局约 14 分钟；
# mock（competition/sdk/core/mock_client.py）替换引擎层，agent/评分/评测
# 链路原样复用，180s 局墙钟数秒。仅用于逻辑/策略迭代，最终分数以
# 真实局（run_fast.ps1）为准。
#
# 用法：
#   powershell -File run/run_mock.ps1                       # coop_decoy 180s
#   powershell -File run/run_mock.ps1 -Duration 600 -Seed 3 # 同 seed 可复现
#   powershell -File run/run_mock.ps1 -Scenario search_track `
#       -Agent competition.user_algorithms.search_track.my_agent:MySearchTrackAgent
param(
    [string]$Scenario = 'coop_decoy',
    [string]$Agent = 'competition.user_algorithms.coop_decoy.agent:CoopDecoyAgent',
    [int]$Duration = 180,
    [int]$Seed = -1,
    [string]$Output = "output/mock_$Scenario"
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$python = Join-Path $root '.venv\Scripts\python.exe'
$cliArgs = @('-m', 'competition', 'run', '--mock', '--scenario', $Scenario,
    '--agent', $Agent, '--duration', "$Duration", '--output', $Output)
if ($Seed -ge 0) { $cliArgs += @('--seed', "$Seed") }

$sw = [System.Diagnostics.Stopwatch]::StartNew()
& $python @cliArgs
$sw.Stop()
Write-Host ("[run_mock] 墙钟 {0:N1}s，评分见 {1}\*.evaluation.json" -f `
    $sw.Elapsed.TotalSeconds, $Output)
