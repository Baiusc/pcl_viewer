<#
  lidar_scan.ps1 - LiDAR discovery panel (UDP broadcast, port 5354)

  Usage:
    powershell -ExecutionPolicy Bypass -File .\lidar_scan.ps1
    powershell -ExecutionPolicy Bypass -File .\lidar_scan.ps1 -Port 5354 -StaleSeconds 30

  Notes:
    - listens on 0.0.0.0:5354, prints NEW / CHANGE / LOST events
    - prints a full table every 10s
    - do not run at the same time as lidar_ctl.ps1 -Ip auto (both bind 5354)
#>
param(
    [int]$Port = 5354,
    [int]$StaleSeconds = 30
)

$udp = New-Object System.Net.Sockets.UdpClient
$udp.Client.SetSocketOption([System.Net.Sockets.SocketOptionLevel]::Socket,
                            [System.Net.Sockets.SocketOptionName]::ReuseAddress, $true)
$udp.Client.Bind((New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, $Port)))
$udp.Client.ReceiveTimeout = 500

Write-Host ("Listening UDP {0} ... Ctrl+C to stop" -f $Port) -ForegroundColor Cyan
Write-Host ("{0,-22} {1,-16} {2,-7} {3}" -f "SN", "IP", "STATE", "ALT (link-local)") -ForegroundColor DarkGray

$devices = @{}
$lastTable = Get-Date

function Show-Table {
    if ($devices.Count -eq 0) { return }
    $now = Get-Date
    Write-Host ""
    Write-Host ("--- {0} device(s) ---" -f $devices.Count) -ForegroundColor DarkGray
    foreach ($k in ($devices.Keys | Sort-Object)) {
        $d = $devices[$k]
        $age = [int]($now - $d.Seen).TotalSeconds
        $color = if ($d.State -eq 'LOCKED') { 'Yellow' } else { 'Green' }
        Write-Host ("{0,-22} {1,-16} {2,-7} {3,3}s  alt={4}" -f $d.Sn, $d.Ip, $d.State, $age, $d.Alt) -ForegroundColor $color
    }
    Write-Host ""
}

while ($true) {
    try {
        $remote = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)
        $bytes = $udp.Receive([ref]$remote)
        $text = [System.Text.Encoding]::UTF8.GetString($bytes).Trim()
        $o = $null
        try { $o = $text | ConvertFrom-Json } catch { }

        if ($o -and $o.sn) {
            $now = Get-Date
            $isNew = -not $devices.ContainsKey($o.sn)
            $oldState = ''
            if (-not $isNew) { $oldState = $devices[$o.sn].State }

            $devices[$o.sn] = [pscustomobject]@{
                Sn    = $o.sn
                Ip    = $o.ip
                Alt   = $o.ip2
                Mac   = $o.mac
                State = $o.state
                Seen  = $now
                Src   = $remote.Address.ToString()
            }

            if ($isNew) {
                Write-Host ("NEW    {0,-22} {1,-16} {2,-7} alt={3}" -f $o.sn, $o.ip, $o.state, $o.ip2) -ForegroundColor Green
            } elseif ($oldState -ne $o.state) {
                Write-Host ("CHANGE {0,-22} {1,-16} {2} -> {3}" -f $o.sn, $o.ip, $oldState, $o.state) -ForegroundColor Yellow
            }
        }
    }
    catch [System.Net.Sockets.SocketException] {
    }
    catch {
    }

    $now = Get-Date
    foreach ($k in @($devices.Keys)) {
        if (($now - $devices[$k].Seen).TotalSeconds -gt $StaleSeconds) {
            Write-Host ("LOST   {0,-22} {1}" -f $k, $devices[$k].Ip) -ForegroundColor Red
            $devices.Remove($k)
        }
    }

    if (($now - $lastTable).TotalSeconds -ge 10) {
        $lastTable = $now
        Show-Table
    }
}
