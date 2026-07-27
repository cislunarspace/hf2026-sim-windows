# setup.ps1 — Windows 版环境检测与依赖安装
# 用 uv 管理 Python 虚拟环境，安装 redis/pyyaml 等依赖。
# 幂等：已装的跳过，重复运行安全。

#Requires -Version 5.1
[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$packRoot = $PSScriptRoot

# ── 工具函数 ──
function Test-Command {
    param([string]$Name)
    $null -ne (Get-Command $Name -ErrorAction SilentlyContinue)
}

function Get-VcRedistInstalledVersion {
    # Checks registry for any MSVC 14.x (2015-2022) redistributable.
    # Returns the Major.Minor.Build string, or $null if not found.
    $keys = @(
        'HKLM:\SOFTWARE\Microsoft\VisualStudio\14.0\VC\Runtimes\x64',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\VisualStudio\14.0\VC\Runtimes\x64',
        'HKLM:\SOFTWARE\Microsoft\VisualStudio\14.1\VC\Runtimes\x64',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\VisualStudio\14.1\VC\Runtimes\x64',
        'HKLM:\SOFTWARE\Microsoft\VisualStudio\14.2\VC\Runtimes\x64',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\VisualStudio\14.2\VC\Runtimes\x64',
        'HKLM:\SOFTWARE\Microsoft\VisualStudio\14.3\VC\Runtimes\x64',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\VisualStudio\14.3\VC\Runtimes\x64',
        'HKLM:\SOFTWARE\Microsoft\VisualStudio\14.4\VC\Runtimes\x64',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\VisualStudio\14.4\VC\Runtimes\x64'
    )
    $latestBld = 0
    foreach ($k in $keys) {
        $v = Get-ItemProperty -Path $k -ErrorAction SilentlyContinue
        if ($v -and $v.Installed -and $v.Bld) {
            if ($v.Bld -gt $latestBld) { $latestBld = $v.Bld }
        }
    }
    if ($latestBld -gt 0) { return $latestBld }
    return $null
}

function Install-VcRedist {
    param([string]$InstallerPath)
    # Run Microsoft's installer silently. May require admin rights.
    # 0   = success
    # 1638 = already installed / newer version present
    # 3010 = success, reboot required
    $proc = Start-Process -FilePath $InstallerPath -ArgumentList '/install', '/passive', '/norestart' -Wait -PassThru
    return $proc.ExitCode
}

# ── 0. Microsoft Visual C++ 2015-2022 Redistributable (x64) ──
# The bundled Python 3.12 is built with MSVC and needs VCRUNTIME140 + UCRT.
# This step detects the redist; if absent, it runs the bundled installer.
# The installer may require administrator rights on some machines.
$packRoot = $PSScriptRoot
$vcInstaller = Join-Path $packRoot 'vc_redist.x64.exe'
$vcBld = Get-VcRedistInstalledVersion
if ($vcBld) {
    Write-Host "✓ VC++ Redistributable detected (build $vcBld)"
} else {
    Write-Host '⚠️  VC++ Redistributable not detected. This is required by the bundled Python.' -ForegroundColor Yellow
    if (Test-Path $vcInstaller) {
        Write-Host '  Installing bundled vc_redist.x64.exe silently...' -ForegroundColor Yellow
        $exitCode = Install-VcRedist -InstallerPath $vcInstaller
        switch ($exitCode) {
            0       { Write-Host '✓ VC++ Redistributable installed successfully.' Green }
            1638    { Write-Host '✓ VC++ Redistributable already present (or newer version installed).' Green }
            3010    { Write-Host '✓ VC++ Redistributable installed. A system reboot may be required.' Yellow }
            default {
                Write-Host "✗ VC++ Redistributable installer returned exit code $exitCode" -ForegroundColor Red
                Write-Host '  Please install it manually: https://aka.ms/vs/17/release/vc_redist.x64.exe' -ForegroundColor Red
                exit 1
            }
        }
    } else {
        Write-Host '✗ Bundled vc_redist.x64.exe not found. The bundled Python may fail to start.' -ForegroundColor Red
        Write-Host '  Please install it manually: https://aka.ms/vs/17/release/vc_redist.x64.exe' -ForegroundColor Red
        exit 1
    }
}

# ── 1. uv（Python 虚拟环境 & 依赖管理） ──
if (-not (Test-Command 'uv')) {
    Write-Host '✗ uv 未安装。uv 是管理 Python 虚拟环境的必需工具。' -ForegroundColor Red
    Write-Host '  安装方式（任选其一）：' -ForegroundColor Yellow
    Write-Host '    winget install astral-sh.uv'
    Write-Host '    powershell -ExecutionPolicy Bypass -c "irm https://astral.sh/uv/install.ps1 | iex"'
    Write-Host '  安装后重新运行 .\setup.ps1'
    exit 1
}
$uvVersion = & uv --version 2>$null
Write-Host "✓ uv: $uvVersion"

# ── 2. 捆绑 Python ──
$bundled = Join-Path $packRoot 'python\python.exe'
if (-not (Test-Path $bundled)) {
    Write-Host '✗ 捆绑 Python 缺失（python\python.exe），发行包不完整。' -ForegroundColor Red
    exit 1
}
Write-Host "✓ 捆绑 Python: $bundled"

# ── 3. Python 虚拟环境（uv 管理） ──
$venvDir = Join-Path $packRoot '.venv'
$venvPython = Join-Path $venvDir 'Scripts\python.exe'

if (Test-Path $venvPython) {
    Write-Host "✓ Python venv 已存在: $venvDir"
} else {
    Write-Host '创建 Python 虚拟环境（uv venv）...'
    & uv venv --python $bundled $venvDir
    if ($LASTEXITCODE -ne 0) {
        Write-Host '✗ uv venv 创建失败' -ForegroundColor Red
        exit 1
    }
    Write-Host "✓ Python venv 已创建: $venvDir"
}

# ── 4. 安装 Python 依赖 ──
Write-Host '安装 Python 依赖（redis + pyyaml）...'
& uv pip install --python $venvPython redis pyyaml
if ($LASTEXITCODE -ne 0) {
    Write-Host '✗ uv pip install 失败，请手动运行: uv pip install --python .venv\Scripts\python.exe redis pyyaml' -ForegroundColor Red
    exit 1
}
Write-Host '✓ Python 依赖已安装到 venv'

# ── 5. 验证 ──
& $venvPython -c 'import redis, yaml' 2>&1 | Out-Null
if ($LASTEXITCODE -eq 0) {
    Write-Host '✓ Python 依赖验证通过（redis + pyyaml）'
} else {
    Write-Host '⚠️  venv 的 redis/pyyaml 不可用，发行包可能损坏' -ForegroundColor Yellow
}

# ── 6. 系统工具（Win10+ 内置） ──
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

# ── 7. 引擎二进制存在性（打包后才有） ──
$simExe = Join-Path $packRoot 'opensim-sim.exe'
if (Test-Path $simExe) {
    Write-Host "✓ opensim-sim.exe 就位"
} else {
    Write-Host '⚠️  opensim-sim.exe 缺失（如在源码仓库运行属正常；release 包内缺失说明打包失败）'
}

Write-Host ''
Write-Host '✓ 环境就绪，可运行: .\start.ps1'
