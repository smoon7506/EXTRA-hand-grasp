# 파이(데몬 노드)로 코드를 보낸다. 보낼 목록은 pi_manifest.txt 에 있다.
#
#   .\deploy_pi.ps1                      기본 주소로
#   .\deploy_pi.ps1 wearlab@192.168.0.5  다른 주소로
#   .\deploy_pi.ps1 -DryRun              보낼 목록만 확인
#
# deploy_pi.sh 와 같은 일을 한다. 둘을 다 두는 이유: PowerShell 에서
# .sh 를 돌리려면 Git Bash 를 따로 열어야 하는데, 그 창에서는 비밀번호
# 프롬프트가 안 뜨는 상황이 생긴다. Windows 는 tar/scp/ssh 를 전부
# 기본 내장하므로 굳이 다른 셸이 필요 없다.
#
# --- tar 를 파이프하지 않는다 ---
# PowerShell 은 파이프를 텍스트로 다뤄서 tar 스트림을 망가뜨린다.
# 그래서 임시 파일로 만들어 scp 한다.
param(
    [string]$Target = "wearlab@192.168.137.236",
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Manifest = Join-Path $Here "pi_manifest.txt"
$RemoteDir = "EXTRA-hand-grasp"

if (-not (Test-Path $Manifest)) { Write-Error "$Manifest 가 없다."; exit 1 }

# 매니페스트를 tar 인자로 바꾼다.
$Includes = @()
$Excludes = @()
foreach ($line in Get-Content $Manifest -Encoding UTF8) {
    $t = $line.Trim()
    if ($t -eq "" -or $t.StartsWith("#")) { continue }
    $parts = $t -split '\s+', 2
    switch ($parts[0]) {
        "include" { $Includes += $parts[1] }
        "exclude" { $Excludes += "--exclude=$($parts[1])" }
    }
}
if ($Includes.Count -eq 0) { Write-Error "매니페스트에 include 가 없다."; exit 1 }

Push-Location $Here
try {
    $missing = $Includes | Where-Object { -not (Test-Path $_) }
    if ($missing) { $missing | ForEach-Object { Write-Host "[ERROR] 없는 경로: $_" }; exit 1 }

    if ($DryRun) {
        Write-Host "[대상] ${Target}:~/$RemoteDir"
        Write-Host "[보냄]"; $Includes | ForEach-Object { Write-Host "  $_" }
        Write-Host "[제외]"; $Excludes | ForEach-Object { Write-Host "  $($_ -replace '^--exclude=','')" }
        Write-Host "[파일]"
        $tmp = [IO.Path]::GetTempFileName()
        tar czf $tmp @Excludes @Includes
        tar tzf $tmp | ForEach-Object { Write-Host "  $_" }
        Remove-Item $tmp -Force
        exit 0
    }

    $Bundle = [IO.Path]::GetTempFileName()
    try {
        tar czf $Bundle @Excludes @Includes
        $kb = [math]::Round((Get-Item $Bundle).Length / 1KB)
        Write-Host "[INFO] 묶음 ${kb}KB -> ${Target}:~/$RemoteDir"

        scp $Bundle "${Target}:~/deploy.tgz"
        if ($LASTEXITCODE -ne 0) { Write-Error "scp 실패"; exit 1 }

        # 파이에 이미 있는 것을 지우지 않는다 -- tar 는 덮어쓰기만 한다.
        # 매니페스트에 없는 것(vendor/, venv/, roi.json)은 손대지 않는다.
        $remote = "mkdir -p ~/$RemoteDir && tar xzf ~/deploy.tgz -C ~/$RemoteDir " +
                  "&& rm -f ~/deploy.tgz " +
                  "&& find ~/$RemoteDir -name '__pycache__' -type d -prune -exec rm -rf {} + " +
                  "; echo '[INFO] 배포 완료' && ls -1 ~/$RemoteDir"
        ssh $Target $remote
        if ($LASTEXITCODE -ne 0) { Write-Error "ssh 실패"; exit 1 }
    }
    finally {
        if (Test-Path $Bundle) { Remove-Item $Bundle -Force }
    }

    Write-Host ""
    Write-Host "[다음] 파이에서 데몬을 다시 띄우세요:"
    Write-Host "  ssh $Target"
    Write-Host "  source ~/venv/bin/activate && cd ~/$RemoteDir/detection && python grasp_daemon.py"
}
finally { Pop-Location }
