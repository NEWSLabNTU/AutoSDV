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
#   .\docker\desktop\autosdv.ps1 -Workspace D:\path
#   .\docker\desktop\autosdv.ps1 -Image jerry73204/autosdv:base
#         mount that directory at /workspace instead of the repository this
#         script lives in ($env:AUTOSDV_WORKSPACE does the same)
#
# If the image was handed out as a file, `docker load` it first; this script
# then finds it locally and pulls nothing.
#
# The logging simulation needs two terminals -- one for the stack and one for
# the rosbag replay -- so running this twice is the normal case, not an error.
#
# Your checkout is mounted at /workspace, and lab work belongs there: it is the
# one directory whose contents survive -Stop, and your own editor is already
# looking at it.
#
# If PowerShell refuses to run this ("running scripts is disabled"), allow
# local scripts for your own account, once:
#
#   Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
#
# Works on Windows PowerShell 5.1 (what Windows ships) and on PowerShell 7.x.
# See the $ErrorActionPreference note below before changing it back: the two
# disagree about what a native command's stderr means, and 5.1 is the one
# students have.

[CmdletBinding()]
param(
    [switch]$Pull,
    [switch]$Stop,
    [switch]$Gpu,
    [string]$Workspace,
    [string]$Image
)

# NOT 'Stop', and this is the difference between working on the PowerShell
# Windows ships and working only on the one a developer installed.
#
# Windows PowerShell 5.1 turns a native command's stderr into a TERMINATING
# NativeCommandError when $ErrorActionPreference is 'Stop'. Every `docker ... 
# 2>$null` below writes to stderr as a matter of course -- `docker inspect` on a
# container that does not exist yet is the FIRST run, every time -- so the
# script died there instead of reading the non-zero exit code it was asking
# for. PowerShell 7 dropped that behaviour, so the script tested clean on 7.x
# and failed for every student on a stock Windows box.
#
# Nothing is lost: every native call here checks $LASTEXITCODE itself, which is
# what actually reports docker's failures. Cmdlet failures that must stop are
# checked explicitly instead.
$ErrorActionPreference = 'Continue'

$Name  = if ($env:AUTOSDV_CONTAINER) { $env:AUTOSDV_CONTAINER } else { 'autosdv' }
# -Image beats the environment variable, which beats the default. :desktop
# carries the AutoSDV workspace prebuilt, which is what the simulations need;
# :base carries the same ROS 2 and tools without it.
if (-not $Image) { $Image = $env:AUTOSDV_IMAGE }
if (-not $Image) { $Image = 'jerry73204/autosdv:desktop' }
$Port  = if ($env:AUTOSDV_PORT)      { $env:AUTOSDV_PORT }      else { '6080' }

# The repository, found from this script rather than from the caller's current
# directory, so the data mount is right wherever it is invoked from.
#
# Checked rather than left to $ErrorActionPreference, which is no longer 'Stop'
# -- see above. A $null here would otherwise reach `docker run -v` as a mount
# path beginning with a bare backslash.
$Repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..') -ErrorAction SilentlyContinue).Path
if (-not $Repo) {
    Write-Host "error: cannot locate the repository root from $PSScriptRoot."
    Write-Host "       Run this script from a clone of the AutoSDV repository."
    exit 1
}

# What gets mounted at /workspace: the repository by default, because that is
# what a student cloned and what their editor has open.
#
# Resolved to a full path, because `docker run -v` reads a relative path as a
# VOLUME NAME rather than a directory -- `-Workspace ..\labs` would silently
# create an empty named volume instead of failing.
if (-not $Workspace) { $Workspace = $env:AUTOSDV_WORKSPACE }
if (-not $Workspace) { $Workspace = $Repo }
$WorkspacePath = (Resolve-Path $Workspace -ErrorAction SilentlyContinue).Path
if (-not $WorkspacePath -or -not (Test-Path $WorkspacePath -PathType Container)) {
    Write-Host "error: -Workspace $Workspace is not a directory."
    exit 1
}
$Workspace = $WorkspacePath

function Say($m) { Write-Host "  $m" }

# Docker reports a mount source with whichever separators it was given, and
# Windows paths differ in case without differing at all, so compare on
# something normalised rather than on the raw strings.
function Format-MountPath($p) {
    if (-not $p) { return '' }
    ($p -replace '/', '\').TrimEnd('\')
}

# `docker exec` does NOT go through the entrypoint, so the shell it opens runs
# as whatever user the image declares -- root -- however carefully the
# entrypoint reconciled HOST_UID on startup. That is the two-terminal
# workflow's every second shell, so left alone a colcon build or a recorded bag
# from it lands root-owned in the student's own checkout.
#
# 1000 to match what this script passes as HOST_UID, for the reason given at
# the `docker run` below: PowerShell has no `id`.
#
# Asked, not assumed. An image without the UID-reconciling entrypoint has no
# passwd entry for 1000, and `docker exec -u` against one gives a shell that
# greets the student with "I have no name!", cannot write its home directory,
# and fails at ~/.ros logging. Where the entry exists we use it; where it does
# not, this behaves exactly as it did before.
function Get-ExecUserArgs {
    # Ask the container whether uid 1000 is a real account before handing it to
    # `docker exec -u`. Docker takes HOME from /etc/passwd when the uid is in
    # there and leaves HOME=/ when it is not, and / is not writable: the first
    # ROS 2 command then dies on its logger with
    #   failed to configure logging: Failed to create log directory: //.ros/log
    # naming neither HOME nor the user. Passing HOME explicitly as well costs
    # nothing and makes the dependency visible.
    $entry = docker exec $Name getent passwd 1000 2>$null
    if ($LASTEXITCODE -ne 0 -or -not $entry) { return @() }

    # NOT $home or $args: both are PowerShell automatic variables. $HOME is
    # read-only and assigning to it throws, and $args is the function's own
    # unbound-argument array.
    $execArgs = @('-u', '1000:1000')
    $homeDir = ($entry -split ':')[5]
    if ($homeDir) { $execArgs += @('-e', "HOME=$homeDir") }
    return $execArgs
}

# A MOUNT CANNOT BE ADDED TO A CONTAINER THAT ALREADY EXISTS -- not to a
# running one, and not to a stopped one either, because the mount list is fixed
# when the container is CREATED. A student who started theirs before
# /workspace existed -- yesterday, or with an older copy of this script --
# reaches it every day afterwards and finds no /workspace, with nothing to say
# why: `cd /workspace` reports only "No such file or directory", which reads as
# a mistake in the handout rather than as a stale container.
#
# Reported, not repaired. Recreating the container silently is the one thing
# not to do here: anything installed or left in the home directory inside it
# goes with it, and this script cannot know whether that matters.
function Show-WorkspaceMountNote {
    $mounted = (docker inspect -f '{{range .Mounts}}{{if eq .Destination "/workspace"}}{{.Source}}{{end}}{{end}}' $Name 2>$null)
    if ($mounted) { $mounted = ($mounted | Out-String).Trim() }
    if (-not $mounted) {
        Write-Host ""
        Write-Host "  note: '$Name' has no /workspace. It was created before this script"
        Write-Host "  mounted one, and a mount cannot be added to a container that already"
        Write-Host "  exists."
        Write-Host ""
        Write-Host "  Recreate it to get one:"
        Write-Host ""
        Write-Host "      .\docker\desktop\autosdv.ps1 -Stop"
        Write-Host "      .\docker\desktop\autosdv.ps1"
        Write-Host ""
        Write-Host "  Your checkout on the host is untouched by that -- but anything saved"
        Write-Host "  INSIDE the container is removed with it, so copy that out first if it"
        Write-Host "  matters:"
        Write-Host ""
        Write-Host "      docker cp ${Name}:/root/something ."
        Write-Host ""
    } elseif ((Format-MountPath $mounted) -ine (Format-MountPath $Workspace)) {
        Write-Host ""
        Write-Host "  note: '$Name' already has /workspace mounted from"
        Write-Host ""
        Write-Host "      $mounted"
        Write-Host ""
        Write-Host "  and you asked for"
        Write-Host ""
        Write-Host "      $Workspace"
        Write-Host ""
        Write-Host "  A mount cannot be changed on a container that already exists, so"
        Write-Host "  -Workspace has no effect here. To switch:"
        Write-Host ""
        Write-Host "      .\docker\desktop\autosdv.ps1 -Stop"
        Write-Host "      .\docker\desktop\autosdv.ps1 -Workspace `"$Workspace`""
        Write-Host ""
    }
}

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
#
# `cd /opt/AutoSDV` is conditional because :base carries no prebuilt workspace
# (only :desktop does). Sourcing an install/setup.bash that is not there prints
# "No such file or directory" on every shell the student opens, which reads as
# a broken image rather than as a different image.
#
# And on that path NOTHING under /workspace is sourced, deliberately. A
# checkout that was ever built on another machine has an install/setup.bash
# whose paths are that machine's, so sourcing it greets every shell with
#   not found: "/opt/ros/humble/local_setup.bash"
# from inside a container where ROS is installed and fine.
$Enter = @'
if [ -f /opt/AutoSDV/install/setup.bash ]; then
    cd /opt/AutoSDV
    [ -f /opt/autoware/1.5.0/setup.bash ] && source /opt/autoware/1.5.0/setup.bash
    source install/setup.bash
    _sourced="ROS 2, Autoware and the workspace are sourced."
else
    cd /workspace 2>/dev/null || cd
    [ -f /opt/autoware/1.5.0/setup.bash ] && source /opt/autoware/1.5.0/setup.bash
    _sourced="ROS 2 and Autoware are sourced; build your own workspace here."
fi
cat <<BANNER
  AutoSDV -- $_sourced

  /workspace is your own checkout, mounted from the host: what you write there
  is on your laptop, and is the only thing that survives -Stop.

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
    Show-WorkspaceMountNote
    Say "attaching another shell to '$Name'"
    Say "desktop: http://localhost:$Port/vnc.html?autoconnect=1&resize=remote"
    Write-Host ""
    $execUser = Get-ExecUserArgs
    docker exec -it @execUser $Name bash -c $Enter
    exit $LASTEXITCODE
}

docker inspect $Name 2>$null | Out-Null
if ($LASTEXITCODE -eq 0) {
    Say "starting the existing container '$Name'"
    docker start $Name | Out-Null
    if ($LASTEXITCODE -ne 0) { Show-PortHelp; exit 1 }
    Show-WorkspaceMountNote
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
    #
    # TWO mounts, and the data one is not redundant. /workspace is the
    # student's checkout, where their own work lives; the data mount stays
    # because :desktop's prebuilt workspace IS /opt/AutoSDV, and mounting the
    # checkout over it would shadow the prebuilt install/ and break the
    # `source install/setup.bash` every shell here does.
    #
    # HOST_UID/HOST_GID are hardcoded to 1000 because PowerShell has no `id`:
    # Windows accounts have SIDs, not POSIX uids. Nothing is lost -- Docker
    # Desktop's drvfs masks ownership on a Windows bind mount anyway, so the
    # value only has to be a real user inside the container, which 1000 is.
    docker run -dit `
        --name $Name `
        -p "${Port}:6080" `
        -v "${Repo}\data:/opt/AutoSDV/data" `
        -v "${Workspace}:/workspace" `
        -e HOST_UID=1000 `
        -e HOST_GID=1000 `
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
Say "workspace: $Workspace -> /workspace"
Say "another terminal: run this script again"
Say "stop: .\docker\desktop\autosdv.ps1 -Stop"
Write-Host ""

$execUser = Get-ExecUserArgs
docker exec -it @execUser $Name bash -c $Enter
exit $LASTEXITCODE
