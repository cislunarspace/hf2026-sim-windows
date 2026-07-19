# dump-sim-logs.ps1 - dump the two critical error logs after a stalled sim
# Usage: cd to package root, then run this script
# It will:
#   1. wait for you to click "Start Simulation" and see the stall
#   2. wait for the Start button to re-enable (sim crashed)
#   3. dump run/sim-output/sim.stderr.log and controller.stderr.log
# Output: a single combined file at run/logs/dump-<timestamp>.log

#Requires -Version 5.1
[CmdletBinding()]
param()

$ErrorActionPreference = 'Continue'
$PACK_ROOT = (Resolve-Path $PSScriptRoot).Path
$SIM_OUT = Join-Path $PACK_ROOT 'run\sim-output'
$LOG_DIR  = Join-Path $PACK_ROOT 'run\logs'
$STAMP    = Get-Date -Format 'yyyyMMdd-HHmmss'
$DUMP     = Join-Path $LOG_DIR ("dump-$STAMP.log")

if (-not (Test-Path $LOG_DIR)) { New-Item -ItemType Directory -Path $LOG_DIR -Force | Out-Null }

function Out {
    param([string]$Msg)
    Write-Host $Msg
    Add-Content -Path $DUMP -Value $Msg
}

Out "===== dump-sim-logs start: $STAMP ====="
Out "package root: $PACK_ROOT"
Out ""

Out "## STEP 1: In your browser, click 'Start Simulation'."
Out "## STEP 2: Wait until progress bar stalls (e.g. 5%) AND the button re-enables."
Out "##         That signals competition runner has crashed."
Out "## STEP 3: Press Enter here."
Out ""
Read-Host "Press Enter once the Start button has re-enabled"

# Wait a couple seconds so any buffered stderr is fully flushed
Start-Sleep -Seconds 2

Out ""
Out "===== run\sim-output\ directory listing ====="
if (Test-Path $SIM_OUT) {
    Get-ChildItem $SIM_OUT -ErrorAction SilentlyContinue | ForEach-Object {
        Out ("  {0,12}  {1:yyyy-MM-dd HH:mm:ss}  {2}" -f $_.Length, $_.LastWriteTime, $_.Name)
    }
} else {
    Out "  (directory does not exist: $SIM_OUT)"
}

Out ""
Out "===== sim.stderr.log (engine stderr - PRIMARY source of truth) ====="
$simErr = Join-Path $SIM_OUT 'sim.stderr.log'
if (Test-Path $simErr) {
    $size = (Get-Item $simErr).Length
    Out "  file size: $size bytes"
    if ($size -eq 0) {
        Out "  (empty - engine died before writing any output)"
    } else {
        Out "  ---------- BEGIN sim.stderr.log ----------"
        Get-Content $simErr -ErrorAction SilentlyContinue | ForEach-Object { Out ("  " + $_) }
        Out "  ---------- END sim.stderr.log ----------"
    }
} else {
    Out "  (file does not exist: $simErr)"
}

Out ""
Out "===== controller.stderr.log (python competition runner stderr) ====="
$ctrlErr = Join-Path $SIM_OUT 'controller.stderr.log'
if (Test-Path $ctrlErr) {
    $size = (Get-Item $ctrlErr).Length
    Out "  file size: $size bytes"
    if ($size -eq 0) {
        Out "  (empty)"
    } else {
        Out "  ---------- BEGIN controller.stderr.log ----------"
        Get-Content $ctrlErr -ErrorAction SilentlyContinue | ForEach-Object { Out ("  " + $_) }
        Out "  ---------- END controller.stderr.log ----------"
    }
} else {
    Out "  (file does not exist: $ctrlErr)"
}

Out ""
Out "===== run\logs\ (latest 20 entries) ====="
Get-ChildItem $LOG_DIR -Filter '*.log' -ErrorAction SilentlyContinue |
    Sort-Object LastWriteTime -Descending | Select-Object -First 20 |
    ForEach-Object {
        Out ("  {0,12}  {1:yyyy-MM-dd HH:mm:ss}  {2}" -f $_.Length, $_.LastWriteTime, $_.Name)
    }

Out ""
Out "===== latest controller / sim log tails ====="
foreach ($name in @('controller.stderr.log', 'sim.stderr.log')) {
    $p = Join-Path $SIM_OUT $name
    if (Test-Path $p) {
        Out ""
        Out "----- tail of $name (last 50 lines) -----"
        Get-Content $p -Tail 50 -ErrorAction SilentlyContinue | ForEach-Object { Out $_ }
    }
}

Out ""
Out "===== dump-sim-logs done ====="
Out "Please send run\logs\$((Split-Path $DUMP -Leaf)) to the developer."