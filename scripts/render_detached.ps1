# render_detached.ps1 — render a shot as a DETACHED process (survives the
# 10-min background-shell limit), then encode. Poll the .done sentinel.
#   .\scripts\render_detached.ps1 -Builder shots/x.py:build -Name x -Frames 800 -Poster 600
param(
    [Parameter(Mandatory)][string]$Builder,
    [Parameter(Mandatory)][string]$Name,
    [int]$Frames = 600,
    [int]$Subframes = 12,
    [int]$Poster = 400,
    [int]$W = 1920,
    [int]$H = 1080
)
$repo = Split-Path $PSScriptRoot -Parent
$cfg = Get-Content "$repo\config\assets.json" -Raw | ConvertFrom-Json
$log = "$repo\output\$Name.log"
$done = "$repo\output\$Name.done"
Remove-Item $done -ErrorAction SilentlyContinue

$inner = "& '$($cfg.isaac_python)' '$repo\lib\cine_capture_core.py' --builder '$Builder' --frames $Frames --subframes $Subframes --w $W --h $H --out '$repo\output\frames\$Name' --asset-root '$($cfg.asset_root)' *> '$log'; " +
         "if (Select-String -Path '$log' -Pattern 'done: $Frames' -Quiet) { " +
         "& 'C:\Program Files\Git\bin\bash.exe' '$repo\scripts\encode.sh' 'output/frames/$Name' 'output/videos/$Name' 60 $Poster; 'OK' | Out-File '$done' } " +
         "else { 'FAIL' | Out-File '$done' }"

Start-Process -FilePath "powershell.exe" -WindowStyle Hidden `
    -ArgumentList "-NoProfile", "-Command", $inner -WorkingDirectory $repo
Write-Output "detached render launched: $Name (frames=$Frames). Sentinel: $done"
