# run/run_mock_batch.ps1 ? ?? mock ???? Tag ?? seed ???????
param(
    [string]$Seeds = "3;4;5;6;7;8;9;10",
    [int]$Duration = 600,
    [string]$Tag = "exp"
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$python = Join-Path $root '.venv\Scripts\python.exe'
$seedList = $Seeds.Split(';') | ForEach-Object { [int]$_ }
foreach ($s in $seedList) {
    $out = "output/mock_${Tag}_s$s"
    Write-Host "[batch] seed=$s -> $out"
    & $python -m competition run --mock --scenario coop_decoy `
        --agent competition.user_algorithms.coop_decoy.agent:CoopDecoyAgent `
        --duration $Duration --output $out --seed $s | Out-Null
    if ($LASTEXITCODE -ne 0) { Write-Host "[batch] seed=$s FAILED exit=$LASTEXITCODE" }
}
Write-Host "=== SUMMARY $Tag ==="
$tot = 0
foreach ($s in $seedList) {
    $f = Get-ChildItem "output/mock_${Tag}_s$s" -Filter '*.evaluation.json' -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($f) {
        $j = Get-Content $f.FullName -Raw | ConvertFrom-Json
        $killed = @(); $coop = 0
        foreach ($p in $j.per_target.PSObject.Properties) { $coop += $p.Value.coop_ticks; if ($p.Value.destroyed) { $killed += "$($p.Name)@$([math]::Round($p.Value.destroyed_at_s,1))" } }
        "{0}: destroyed={1} killed=[{2}] coop={3} base={4} pen={5}" -f $s, $j.n_destroyed, ($killed -join ','), $coop, [math]::Round($j.base_score,1), $j.penalty
        $tot += $j.n_destroyed
    } else {
        "{0}: NO EVAL JSON" -f $s
    }
}
Write-Host "TOTAL KILLS: $tot"
