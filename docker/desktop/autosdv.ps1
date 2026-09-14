# Start the AutoSDV desktop container, or open another shell in the one that is
# already running.  Windows PowerShell.  Linux and macOS: use autosdv.sh.
#
#   .\docker\desktop\autosdv.ps1          first run: pull, start, open a shell
#   .\docker\desktop\autosdv.ps1          again: a SECOND shell in the SAME container
#   .\docker\desktop\autosdv.ps1 -Gpu     pass an NVIDIA GPU through. Docker
#         Desktop does this through WSL2 and needs GPU support enabled there;
#         it cannot work on macOS at all.
#   .\docker\desktop\autosdv.ps1 -Pull    check for a newer image first
#   .\docker\desktop\autosdv.ps1 -Stop    stop and remove the container
#
# If the image was handed out as a file, `docker load` it first; this script
# then finds it locally and pulls nothing.
#
# The logging simulation needs two terminals -- one for the stack and one for
# the rosbag replay -- so running this twice is the normal case, not an error.
#
# If PowerShell refuses to run this ("running scripts is disabled"), allow
# local scripts for your own account, once:
#
#   Set-ExecutionPolicy -Scope CurrentUser RemoteSigned

[CmdletBinding()]
param(
    [switch]$Pull,
    [switch]$Stop,
    [switch]$Gpu
)

$ErrorActionPreference = 'Stop'

$Name  = if ($env:AUTOSDV_CONTAINER) { $env:AUTOSDV_CONTAINER } else { 'autosdv' }
$Image = if ($env:AUTOSDV_IMAGE)     { $env:AUTOSDV_IMAGE }     else { 'jerry73204/autosdv:desktop' }
$Port  = if ($env:AUTOSDV_PORT)      { $env:AUTOSDV_PORT }      else { '6080' }

# The repository, found from this script rather than from the caller's current
# directory, so the data mount is right wherever it is invoked from.
$Repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path

function Say($m) { Write-Host "  $m" }

function Show-PortHelp {
    Write-Host ""
    Write-Host "  Could not start the container. If the message above mentions port $Port,"
    Write-Host "  something else already has it -- often an earlier AutoSDV container."
    Write-Host ""
    Write-Host "      docker ps --filter publish=$Port"
    Write-Host ""
    Write-Host "  Then either remove the old container:"
    Write-Host ""
    Write-Host "      .\docker\desktop\autosdv.ps1 -Stop"
    Write-Host ""
    Write-Host "  or use a different port:"
    Write-Host ""
    Write-Host "      `$env:AUTOSDV_PORT='6081'; .\docker\desktop\autosdv.ps1"
    Write-Host ""
}

# The shell every terminal gets: ROS 2, Autoware and the workspace already
# sourced. A shell without them has no `ros2` command at all, and the error
# says only "command not found".
$Enter = @'
cd /opt/AutoSDV
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash
cat <<BANNER
  AutoSDV -- ROS 2, Autoware and the workspace are sourced.

  Two terminals: run this script again for the second one.
  Add --container-mode observable to play_launch: the default forks one
  process per composable node (126 processes, 4.6 GB) where observable keeps
  them in their containers (49 processes, 2.1 GB). On a memory-capped Docker
  Desktop VM the default gets nodes killed.

BANNER
exec bash
'@ -replace "`r`n", "`n"     # CRLF would reach bash as literal ^M and break every line


if ($Stop) {
    docker rm -f $Name 2>$null | Out-Null
    if ($LASTEXITCODE -eq 0) { Say "stopped and removed '$Name'" } else { Say "no container named '$Name'" }
    exit 0
}

docker info 2>$null | Out-Null
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "  Docker is not running. Start Docker Desktop and wait for the whale"
    Write-Host "  icon to stop animating, then run this again."
    Write-Host ""
    Write-Host "  Install: https://docs.docker.com/desktop/"
    Write-Host ""
    exit 1
}

$running = (docker inspect -f '{{.State.Running}}' $Name 2>$null)
if ($running -eq 'true') {
    # Report the port the container was actually started on, not the default
    # this invocation happens to carry.
    $actual = (docker inspect -f '{{with index .NetworkSettings.Ports "6080/tcp"}}{{(index . 0).HostPort}}{{end}}' $Name 2>$null)
    if ($actual) { $Port = $actual }
    if ($Gpu) {
        Write-Host ""
        Write-Host "  note: -Gpu only takes effect when the container is created, and '$Name'"
        Write-Host "  is already running. To switch it on, stop it first:"
        Write-Host ""
        Write-Host "      .\docker\desktop\autosdv.ps1 -Stop"
        Write-Host "      .\docker\desktop\autosdv.ps1 -Gpu"
        Write-Host ""
    }
    Say "attaching another shell to '$Name'"
    Say "desktop: http://localhost:$Port/vnc.html?autoconnect=1&resize=remote"
    Write-Host ""
    docker exec -it $Name bash -c $Enter
    exit $LASTEXITCODE
}

docker inspect $Name 2>$null | Out-Null
if ($LASTEXITCODE -eq 0) {
    Say "starting the existing container '$Name'"
    docker start $Name | Out-Null
    if ($LASTEXITCODE -ne 0) { Show-PortHelp; exit 1 }
} else {
    docker image inspect $Image 2>$null | Out-Null
    if ($Pull -or $LASTEXITCODE -ne 0) {
        Say "pulling $Image -- this is several gigabytes, and only happens once"
        Say "(if the image was handed out as a file, run 'docker load' on it first)"
        docker pull $Image
        if ($LASTEXITCODE -ne 0) { exit 1 }
    }

    if (-not (Test-Path (Join-Path $Repo 'data'))) {
        Write-Host "error: $Repo\data does not exist."
        Write-Host "       The map and rosbags live there and are not in the image."
        Write-Host "       Run this from a clone of the AutoSDV repository."
        exit 1
    }

    $gpuArgs = @()
    if ($Gpu) {
        # NVIDIA_DRIVER_CAPABILITIES must include graphics, not just the default
        # compute,utility: without it the driver exposes CUDA but no GL, and the
        # entrypoint's VirtualGL path finds a GPU it cannot draw with.
        $gpuArgs = @('--gpus','all','-e','NVIDIA_DRIVER_CAPABILITIES=all','-e','NVIDIA_VISIBLE_DEVICES=all')
        Say "passing through an NVIDIA GPU (requires GPU support in Docker Desktop/WSL2)"
    }
    Say "starting '$Name'"
    # -dit rather than -d: the image's command is bash, which exits immediately
    # without a terminal attached, and the container would stop with it.
    #
    # RMW_IMPLEMENTATION is set because the default, FastDDS, addresses
    # participants through ports derived from an index that runs out near 120
    # per host per domain -- and the logging simulation brings up 125. Past
    # that the stack keeps working while nothing new can join: a second
    # terminal's `ros2 bag play` publishes into the void, with no error
    # anywhere. Images built after this was found already set it.
    docker run -dit `
        --name $Name `
        -p "${Port}:6080" `
        -v "${Repo}\data:/opt/AutoSDV/data" `
        --shm-size=2gb `
        --cap-add=NET_ADMIN `
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp `
        @gpuArgs `
        $Image | Out-Null
    if ($LASTEXITCODE -ne 0) {
        # A failed `docker run` still leaves the named container behind, and the
        # next invocation would fail again in a way that no longer names why.
        docker rm -f $Name 2>$null | Out-Null
        Show-PortHelp
        exit 1
    }
}

Say "desktop: http://localhost:$Port/vnc.html?autoconnect=1&resize=remote"
Say "another terminal: run this script again"
Say "stop: .\docker\desktop\autosdv.ps1 -Stop"
Write-Host ""

docker exec -it $Name bash -c $Enter
exit $LASTEXITCODE
