# Rolloff roof controller — field availability monitor
#
# Run this on the observatory PC and leave it going for days. It answers the one
# question the device cannot answer about itself: "were there moments when a
# fresh TCP connection could not be established?"
#
# READ-ONLY. It opens a TCP connection and closes it, and GETs /status.json.
# It never touches /cmd and never issues an Alpaca PUT, so it cannot move the roof.
#
# Deliberately probes once a MINUTE, not every few seconds. The device sustains
# only ~7 new connections/minute before lwIP's 16-PCB pool is under pressure, so
# a fast prober becomes a significant part of the load it is trying to measure.
#
#   .\monitor-rolloff.ps1
#   .\monitor-rolloff.ps1 -Ip 192.168.1.139 -IntervalSec 60 -Csv C:\temp\rolloff.csv
#
# The fault fingerprint, visible in the CSV:
#   reqAlpaca stops advancing while uptimeMin and rssi keep advancing,
#   and connect logs "fail" with ms at the timeout value (a drop, not a refusal).
# A refusal would come back in ~1 ms and would mean something quite different.

param(
    [string]$Ip          = "192.168.1.139",
    [int]   $IntervalSec = 60,
    [int]   $TimeoutMs   = 2000,
    [string]$Csv         = "$env:USERPROFILE\Documents\rolloff-monitor.csv",
    [int]   $MaxPolls    = 0        # 0 = run for ever; set a small number to smoke-test
)

if (-not (Test-Path $Csv)) {
    "ts,connect,connect_ms,uptimeMin,rssi,drops,reqInfo,reqAlpaca,rate,heap,maxBlock,listen,shutter" |
        Out-File $Csv -Encoding ascii
}

Write-Host "Monitoring $Ip every ${IntervalSec}s -> $Csv"
Write-Host "Ctrl+C to stop. Read-only: this cannot move the roof."

$lastAlpaca = -1
$flatFor    = 0
$polls      = 0

while ($true) {
    if ($MaxPolls -gt 0 -and $polls -ge $MaxPolls) { break }
    $polls++
    $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss"

    # 1. Can the Alpaca port serve a real request?
    #
    # This deliberately issues a full HTTP GET rather than a bare TCP connect.
    # A connect-then-close leaves a SILENT client, and WebServer holds that
    # accepted socket for HTTP_MAX_DATA_WAIT (5 s) waiting for a request that
    # never arrives — during which it accepts nobody else on that port. A bare
    # connect probe is therefore not a passive observer: it manufactures the
    # very starvation it claims to measure. A real request completes and lets
    # the server close cleanly, and it exercises accept -> parse -> handle ->
    # respond instead of just the SYN.
    $sw = [Diagnostics.Stopwatch]::StartNew()
    $code = curl.exe -s -o NUL -m ([int]($TimeoutMs / 1000) + 1) -w "%{http_code}" `
                "http://${Ip}:11111/api/v1/dome/0/interfaceversion" 2>$null
    $sw.Stop()
    $connect = if ($code -eq "200") { "ok" } else { "fail" }
    $ms = [int]$sw.Elapsed.TotalMilliseconds

    # 2. Snapshot the device's own counters.
    $d = $null
    try {
        $raw = curl.exe -s -m 5 "http://${Ip}/status.json" 2>$null
        if ($raw) { $d = $raw | ConvertFrom-Json }
    } catch { }

    if ($d) {
        "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}" -f `
            $ts, $connect, $ms, $d.uptimeMin, $d.rssi, $d.drops, $d.reqInfo,
            $d.reqAlpaca, $d.rate, $d.heap, $d.maxBlock, $d.listen, $d.shutter |
            Out-File $Csv -Append -Encoding ascii

        # Flag the fingerprint live: Alpaca requests flat while the clock runs on.
        if ($d.reqAlpaca -eq $lastAlpaca) { $flatFor++ } else { $flatFor = 0 }
        $lastAlpaca = $d.reqAlpaca

        if ($connect -ne "ok") {
            Write-Host "$ts  CONNECT $connect (${ms}ms)  uptime=$($d.uptimeMin)m listeners=$($d.listen)" -ForegroundColor Red
        } elseif ($flatFor -ge 3) {
            Write-Host "$ts  reqAlpaca flat at $($d.reqAlpaca) for $flatFor polls - is NINA connected?" -ForegroundColor Yellow
        }
    } else {
        "{0},{1},{2},,,,,,,,,," -f $ts, $connect, $ms | Out-File $Csv -Append -Encoding ascii
        Write-Host "$ts  CONNECT $connect (${ms}ms)  status.json unreachable" -ForegroundColor Red
    }

    Start-Sleep -Seconds $IntervalSec
}
