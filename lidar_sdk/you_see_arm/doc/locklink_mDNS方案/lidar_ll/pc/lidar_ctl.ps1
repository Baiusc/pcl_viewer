<#
  lidar_ctl.ps1 - send a control command to one LiDAR

  Usage:
    .\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action status
    .\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action lock
    .\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action hold -Duration 30
    .\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action setip
    .\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action setip -Octet 77
    .\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action renew  -Token <token>
    .\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action unlock -Token <token>
    .\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action identify
    .\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action info
    .\lidar_ctl.ps1 -Sn CS96000C0126020001 -Action status -Ip 169.254.27.54

  "hold"  = lock once, then renew every 2.5s for -Duration seconds.
  "setip" = keep the first three octets, ask for the last one (default = current),
            the device ARP-probes it first and refuses if someone already owns it.

  If -Ip is omitted, the script listens on UDP 5354 for that SN and picks up its IP.
#>
param(
    [Parameter(Mandatory = $true)][string]$Sn,
    [ValidateSet('lock', 'renew', 'unlock', 'status', 'identify', 'info', 'hold', 'setip')]
    [string]$Action = 'status',
    [string]$Token = '',
    [string]$Ip = '',
    [int]$Port = 5354,
    [int]$CtrlPort = 8080,
    [int]$TimeoutSeconds = 8,
    [int]$Duration = 30,
    [int]$Octet = 0
)

function Find-DeviceIp {
    param($TargetSn, $ListenPort, $Seconds)

    $udp = New-Object System.Net.Sockets.UdpClient
    $udp.Client.SetSocketOption([System.Net.Sockets.SocketOptionLevel]::Socket,
                                [System.Net.Sockets.SocketOptionName]::ReuseAddress, $true)
    $udp.Client.Bind((New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, $ListenPort)))
    $udp.Client.ReceiveTimeout = 800

    $deadline = (Get-Date).AddSeconds($Seconds)
    $found = $null
    while ((Get-Date) -lt $deadline -and -not $found) {
        try {
            $remote = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)
            $bytes = $udp.Receive([ref]$remote)
            $obj = $null
            try { $obj = ([System.Text.Encoding]::UTF8.GetString($bytes)).Trim() | ConvertFrom-Json } catch { }
            if ($obj -and $obj.sn -eq $TargetSn) { $found = $obj.ip }
        }
        catch [System.Net.Sockets.SocketException] {
        }
        catch {
        }
    }
    $udp.Close()
    return $found
}

function Invoke-Cmd {
    param($TargetIp, $TargetPort, $Command)

    $client = New-Object System.Net.Sockets.TcpClient
    $client.Connect($TargetIp, $TargetPort)
    $stream = $client.GetStream()
    $stream.ReadTimeout = 3000
    $payload = [System.Text.Encoding]::ASCII.GetBytes($Command + "`n")
    $stream.Write($payload, 0, $payload.Length)
    $buf = New-Object byte[] 512
    $n = $stream.Read($buf, 0, $buf.Length)
    $resp = [System.Text.Encoding]::ASCII.GetString($buf, 0, $n).Trim()
    $client.Close()
    return $resp
}

if (-not $Ip) {
    Write-Host ("searching for SN {0} on udp/{1} ..." -f $Sn, $Port) -ForegroundColor DarkGray
    $Ip = Find-DeviceIp -TargetSn $Sn -ListenPort $Port -Seconds $TimeoutSeconds
    if (-not $Ip) {
        Write-Host ("device {0} not found (no broadcast in {1}s)" -f $Sn, $TimeoutSeconds) -ForegroundColor Red
        exit 1
    }
}

Write-Host ("device {0} at {1}" -f $Sn, $Ip) -ForegroundColor Cyan

if ($Action -eq 'hold') {
    try {
        $resp = Invoke-Cmd -TargetIp $Ip -TargetPort $CtrlPort -Command 'LOCK'
    }
    catch {
        Write-Host ("connect failed: {0}" -f $_.Exception.Message) -ForegroundColor Red
        exit 1
    }
    if (-not $resp.StartsWith('OK')) {
        Write-Host ("lock refused: {0}" -f $resp) -ForegroundColor Red
        exit 1
    }
    $tok = $resp.Split(' ')[1]
    Write-Host ("locked, token = {0}" -f $tok) -ForegroundColor Green
    Write-Host ("renewing every 2.5s for {0}s, press Ctrl+C to stop" -f $Duration) -ForegroundColor DarkGray

    $deadline = (Get-Date).AddSeconds($Duration)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Milliseconds 2500
        try {
            $r = Invoke-Cmd -TargetIp $Ip -TargetPort $CtrlPort -Command ("RENEW " + $tok)
        }
        catch {
            Write-Host ("renew failed: {0}" -f $_.Exception.Message) -ForegroundColor Red
            break
        }
        Write-Host ("  [{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $r)
        if ($r -eq 'GONE') { break }
    }

    Write-Host ""
    Write-Host "stopped renewing, lease expires in ~8s" -ForegroundColor DarkGray
    exit 0
}

if ($Action -eq 'setip') {
    $parts = $Ip.Split('.')
    if ($parts.Count -ne 4) {
        Write-Host ("unexpected address: {0}" -f $Ip) -ForegroundColor Red
        exit 1
    }
    $prefix = "{0}.{1}.{2}" -f $parts[0], $parts[1], $parts[2]
    $curOctet = [int]$parts[3]

    Write-Host ""
    Write-Host ("current address : {0}" -f $Ip) -ForegroundColor Cyan
    Write-Host ("prefix          : {0}.___" -f $prefix) -ForegroundColor Cyan
    Write-Host ""

    for ($attempt = 1; $attempt -le 5; $attempt++) {
        $oct = $curOctet
        if ($Octet -gt 0) {
            $oct = $Octet
        }
        else {
            $in = Read-Host ("new last octet [{0}]" -f $curOctet)
            if (-not [string]::IsNullOrWhiteSpace($in)) {
                $parsed = 0
                if (-not [int]::TryParse($in.Trim(), [ref]$parsed)) {
                    Write-Host "  please enter a number" -ForegroundColor Red
                    continue
                }
                $oct = $parsed
            }
        }

        if ($oct -lt 1 -or $oct -gt 254) {
            Write-Host "  octet must be 1..254" -ForegroundColor Red
            continue
        }

        $cand = "{0}.{1}" -f $prefix, $oct
        if ($cand -eq $Ip) {
            Write-Host "  same as current address, nothing to change" -ForegroundColor DarkGray
            exit 0
        }

        try {
            $lock = Invoke-Cmd -TargetIp $Ip -TargetPort $CtrlPort -Command 'LOCK'
        }
        catch {
            Write-Host ("  connect failed: {0}" -f $_.Exception.Message) -ForegroundColor Red
            exit 1
        }
        if (-not $lock.StartsWith('OK')) {
            Write-Host ("  cannot lock device: {0}" -f $lock) -ForegroundColor Red
            exit 1
        }
        $tok = $lock.Split(' ')[1]

        try {
            $resp = Invoke-Cmd -TargetIp $Ip -TargetPort $CtrlPort -Command ("SETIP {0} {1}" -f $cand, $tok)
        }
        catch {
            Write-Host ("  setip failed: {0}" -f $_.Exception.Message) -ForegroundColor Red
            exit 1
        }

        if ($resp.StartsWith('OK')) {
            Write-Host ("  accepted: {0}" -f $resp) -ForegroundColor Green
            Write-Host ""
            Write-Host ("device is switching to {0}, it will disappear from the old address in ~300ms" -f $cand) -ForegroundColor Yellow
            Write-Host ("re-discover with:  .\lidar_ctl.ps1 -Sn {0} -Action status" -f $Sn) -ForegroundColor DarkGray
            exit 0
        }
        elseif ($resp.StartsWith('BUSY')) {
            Write-Host ("  {0} is already in use -> {1}" -f $cand, $resp) -ForegroundColor Red
            Invoke-Cmd -TargetIp $Ip -TargetPort $CtrlPort -Command ("UNLOCK " + $tok) | Out-Null
            if ($Octet -gt 0) {
                Write-Host "  pick another octet with -Octet" -ForegroundColor Red
                exit 1
            }
            Write-Host "  please choose another one" -ForegroundColor Red
        }
        elseif ($resp.StartsWith('BADREQ')) {
            Write-Host ("  rejected: {0}" -f $resp) -ForegroundColor Red
            Invoke-Cmd -TargetIp $Ip -TargetPort $CtrlPort -Command ("UNLOCK " + $tok) | Out-Null
        }
        else {
            Write-Host ("  unexpected: {0}" -f $resp) -ForegroundColor Red
            Invoke-Cmd -TargetIp $Ip -TargetPort $CtrlPort -Command ("UNLOCK " + $tok) | Out-Null
        }
    }

    Write-Host "  too many attempts, giving up" -ForegroundColor Red
    exit 1
}

$cmd = switch ($Action) {
    'lock'     { 'LOCK' }
    'renew'    { "RENEW $Token" }
    'unlock'   { "UNLOCK $Token" }
    'status'   { 'STATUS' }
    'identify' { 'IDENTIFY' }
    'info'     { 'INFO' }
}

try {
    $resp = Invoke-Cmd -TargetIp $Ip -TargetPort $CtrlPort -Command $cmd
}
catch {
    Write-Host ("connect failed: {0}" -f $_.Exception.Message) -ForegroundColor Red
    if ($Ip -like '169.254.*') {
        Write-Host ""
        Write-Host "hint: this PC has no 169.254.x.x address on that NIC." -ForegroundColor Yellow
        Write-Host "      traffic to $Ip is sent to the default gateway and gets dropped." -ForegroundColor Yellow
        Write-Host '      fix (admin): netsh interface ipv4 add address name="Ethernet" address=169.254.1.100 mask=255.255.0.0' -ForegroundColor Yellow
        Write-Host "      check      : route print 169.254*" -ForegroundColor Yellow
    }
    exit 1
}

Write-Host ("<- {0}" -f $resp)

if ($Action -eq 'lock' -and $resp.StartsWith('OK')) {
    $tok = $resp.Split(' ')[1]
    Write-Host ""
    Write-Host ("token = {0}" -f $tok) -ForegroundColor Cyan
    Write-Host "lease = 8s. easiest way to keep it alive:"
    Write-Host ("  .\lidar_ctl.ps1 -Sn {0} -Action hold -Ip {1} -Duration 30" -f $Sn, $Ip) -ForegroundColor DarkGray
}
