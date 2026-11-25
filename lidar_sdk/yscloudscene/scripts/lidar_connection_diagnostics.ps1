param(
    [System.Net.IPAddress]$PeerIp = '10.42.0.218',
    [ValidateRange(0, 65535)]
    [int]$CommandPort = 0,
    [ValidateRange(1, 65535)]
    [int]$DataPort = 50027
)

$ErrorActionPreference = 'Stop'

Write-Output "LiDAR connection diagnostics: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
Write-Output "Peer: $PeerIp"
Write-Output 'Run this on the deployed computer while the application is waiting for the handshake.'

Write-Output "`nIPv4 addresses:"
Get-NetIPAddress -AddressFamily IPv4 |
    Select-Object InterfaceAlias, IPAddress, PrefixLength, AddressState |
    Format-Table -AutoSize

Write-Output "`nWindows route and source address for the LiDAR:"
try {
    Find-NetRoute -RemoteIPAddress $PeerIp.ToString() | Format-List *
}
catch {
    Write-Warning "Could not read the route: $_"
}

Write-Output "`nUDP listener owners:"
$lidarPorts = @(55000, $DataPort)
if ($CommandPort -gt 0) {
    $lidarPorts += $CommandPort
}
try {
    Get-NetUDPEndpoint |
        Where-Object { $_.LocalPort -in $lidarPorts } |
        Select-Object LocalAddress, LocalPort, OwningProcess,
            @{Name = 'ProcessName'; Expression = {
                (Get-Process -Id $_.OwningProcess -ErrorAction SilentlyContinue).ProcessName
            }},
            @{Name = 'ExecutablePath'; Expression = {
                (Get-Process -Id $_.OwningProcess -ErrorAction SilentlyContinue).Path
            }} |
        Format-Table -Wrap -AutoSize
}
catch {
    Write-Warning "Could not read UDP listeners: $_"
}

Write-Output "`nNetwork categories:"
try {
    Get-NetConnectionProfile |
        Select-Object InterfaceAlias, NetworkCategory, IPv4Connectivity |
        Format-Table -AutoSize
}
catch {
    Write-Warning "Could not read network categories: $_"
}

Write-Output "`nFirewall profile settings (individual application rules are not evaluated):"
try {
    Get-NetFirewallProfile |
        Select-Object Name, Enabled, DefaultInboundAction, DefaultOutboundAction |
        Format-Table -AutoSize
}
catch {
    Write-Warning "Could not read firewall profiles: $_"
}
