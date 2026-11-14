# Packs the committed tree plus all submodules into dist\g1d_ws-<commit>.tar.gz for the robot.
# Built from `git archive`, so files have LF line endings and the SDK's symlinks stay symlinks
# (a plain copy of a Windows checkout has neither; scripts/build_robot.sh repairs the symlinks).
#
#   powershell -ExecutionPolicy Bypass -File scripts\pack_for_robot.ps1
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root
$commit = (git rev-parse --short HEAD).Trim()
if (git status --porcelain --untracked-files=no) { Write-Warning "uncommitted changes are not packed" }
$tmp = Join-Path $env:TEMP "g1d_pack_$commit"
New-Item -ItemType Directory -Force $tmp, dist | Out-Null

$parts = @()
git archive --format=tar --prefix=g1d_ws/ -o "$tmp\main.tar" HEAD
$parts += "$tmp\main.tar"
$subs = git submodule status --recursive | ForEach-Object { ($_.Trim() -split '\s+')[1] }
foreach ($s in $subs) {
  $name = ($s -replace '[\\/]', '_')
  git -C $s archive --format=tar --prefix="g1d_ws/$s/" -o "$tmp\$name.tar" HEAD
  if ($LASTEXITCODE) { throw "git archive failed for $s" }
  $parts += "$tmp\$name.tar"
}
$out = Join-Path $Root "dist\g1d_ws-$commit.tar.gz"
$tarArgs = @("-czf", $out) + ($parts | ForEach-Object { "@$_" })
& tar @tarArgs
if ($LASTEXITCODE) { throw "tar failed" }
Remove-Item -Recurse -Force $tmp
$size = [math]::Round((Get-Item $out).Length / 1MB, 1)
Write-Host "packed $out ($size MB, commit $commit, $($subs.Count) submodules)"
Write-Host "robot: scp dist\g1d_ws-$commit.tar.gz unitree@192.168.123.164:~"
Write-Host "       tar xzf g1d_ws-$commit.tar.gz && sudo mv g1d_ws /opt/ && /opt/g1d_ws/scripts/build_robot.sh"
