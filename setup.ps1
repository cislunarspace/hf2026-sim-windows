# setup.ps1 — Windows 版环境检测与依赖安装
# 检测 python / pip / redis(python) / pyyaml 等依赖，缺失则自动 pip 安装。
# 幂等：已装的跳过，重复运行安全。

#Requires -Version 5.1
[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# ── 工具函数 ──
function Test-Command {
    param([string]$Name)
    $null -ne (Get-Command $Name -ErrorAction SilentlyContinue)
}

function Get-PythonExe {
    # 与 bridge/index.ts 一致的探测顺序，避免命中 MS Store stub
    $candidates = @(
        'C:\Python314\python.exe',
        'C:\Python313\python.exe',
        'C:\Python312\python.exe',
        'C:\Python311\python.exe'
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) { return $c }
    }
    if (Test-Command 'python') { return 'python' }
    if (Test-Command 'py')     { return 'py' }
    return $null
}

function Invoke-Python {
    param([string]$Script, [string]$PythonExe)
    # 用 try/catch 包裹并把 stderr → stdout，避免 Set-StrictMode 下
    # python -c 失败时抛 NativeCommandError 中断整个 setup 流程。
    try {
        $output = & $PythonExe -c $Script 2>&1
        return $LASTEXITCODE -eq 0
    } catch {
        return $false
    }
}

# ── 1. Python 解释器 ──
$python = Get-PythonExe
if (-not $python) {
    Write-Host '✗ 未找到 python。请从 https://www.python.org/downloads/ 安装 Python 3.10+，'
    Write-Host '  或用 winget install Python.Python.3.12 安装。'
    exit 1
}
Write-Host "✓ Python: $python"

# ── 2. pip ──
& $python -m pip --version 2>$null | Out-Null
$pipOk = $LASTEXITCODE -eq 0
if (-not $pipOk) {
    Write-Host '✗ pip 不可用，请重新安装 Python 时勾选 "Install pip"'
    exit 1
}
Write-Host '✓ pip 可用'

# ── 3. Python 包 redis / pyyaml ──
$needPip = @()
$redisOk = Invoke-Python -Script 'import redis' -PythonExe $python
if (-not $redisOk) { $needPip += 'redis' }
$yamlOk = Invoke-Python -Script 'import yaml' -PythonExe $python
if (-not $yamlOk) { $needPip += 'pyyaml' }

if ($needPip) {
    Write-Host "缺少 Python 包: $($needPip -join ', ')"
    & $python -m pip install --user @needPip
    if ($LASTEXITCODE -ne 0) {
        Write-Host '✗ pip install 失败，请手动运行: python -m pip install --user redis pyyaml'
        exit 1
    }
    Write-Host "✓ 已安装: $($needPip -join ', ')"
} else {
    Write-Host '✓ Python 包 redis + pyyaml 就绪'
}

# ── 4. 系统工具（Win10+ 内置） ──
if (Test-Command 'curl') {
    Write-Host '✓ curl 可用'
} else {
    Write-Host '⚠️  curl 不可用（Win10 1809+ 内置，建议升级 Windows）'
}
if (Test-Command 'tar') {
    Write-Host '✓ tar 可用'
} else {
    Write-Host '⚠️  tar 不可用（Win10 1809+ 内置，建议升级 Windows）'
}

# ── 5. 引擎二进制存在性（打包后才有） ──
$packRoot = $PSScriptRoot
$simExe = Join-Path $packRoot 'opensim-sim.exe'
if (Test-Path $simExe) {
    Write-Host "✓ opensim-sim.exe 就位"
} else {
    Write-Host '⚠️  opensim-sim.exe 缺失（如在源码仓库运行属正常；release 包内缺失说明打包失败）'
}

Write-Host ''
Write-Host '✓ 环境就绪，可运行: .\start.ps1'
