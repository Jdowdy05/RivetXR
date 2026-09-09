param(
    [Parameter(Mandatory = $true)]
    [string] $Apk,
    [string] $Aapt2Path,
    [string] $ReadElfPath,
    [string] $ApkSignerPath
)

$ErrorActionPreference = 'Stop'

function Fail([string] $Message) {
    throw "APK contract failure: $Message"
}

$apkPath = [System.IO.Path]::GetFullPath($Apk)
if (-not (Test-Path -LiteralPath $apkPath -PathType Leaf)) {
    Fail "APK does not exist: $apkPath"
}

$signer = $ApkSignerPath
if ([string]::IsNullOrWhiteSpace($signer)) { $signer = (Get-Command apksigner.bat -ErrorAction SilentlyContinue).Source }
if ([string]::IsNullOrWhiteSpace($signer) -or -not (Test-Path -LiteralPath $signer)) {
    Fail 'apksigner is required to verify APK Signature Scheme v2'
}
$signature = (& $signer verify --verbose $apkPath 2>&1 | Out-String)
if ($LASTEXITCODE -ne 0 -or $signature -notmatch 'Verified using v2 scheme \(APK Signature Scheme v2\): true') {
    Fail "APK is not v2-signed: $signature"
}

Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
$archive = [System.IO.Compression.ZipFile]::OpenRead($apkPath)
try {
    $entries = @{}
    foreach ($entry in $archive.Entries) {
        if ([string]::IsNullOrEmpty($entry.Name)) { continue }
        if ($entry.FullName -match '(^|/)\.\.?(/|$)' -or $entry.FullName.StartsWith('/')) {
            Fail "unsafe ZIP entry path: $($entry.FullName)"
        }
        if ($entries.ContainsKey($entry.FullName)) { Fail "duplicate ZIP entry: $($entry.FullName)" }
        $entries[$entry.FullName] = $entry
    }

    $manifestEntry = $entries['assets/newton/artifact_manifest.json']
    if ($null -eq $manifestEntry) { Fail 'missing canonical artifact manifest' }
    $manifestReader = [System.IO.StreamReader]::new($manifestEntry.Open())
    try { $manifest = $manifestReader.ReadToEnd() | ConvertFrom-Json }
    finally { $manifestReader.Dispose() }

    $abis = @($entries.Keys | Where-Object { $_ -match '^lib/([^/]+)/[^/]+\.so$' } |
        ForEach-Object { [regex]::Match($_, '^lib/([^/]+)/').Groups[1].Value } | Sort-Object -Unique)
    if ($abis.Count -eq 0) { Fail 'APK contains no native libraries' }
    $badAbis = @($abis | Where-Object { $_ -ne 'arm64-v8a' })
    if ($badAbis.Count -gt 0) { Fail "unsupported native ABI(s): $($badAbis -join ', ')" }

    $requiredEntries = @(
        'lib/arm64-v8a/libquest_newton.so',
        'lib/arm64-v8a/libwarp.so',
        'lib/arm64-v8a/libc++_shared.so',
        'assets/newton/franka_reset.wrp',
        'assets/newton/franka_step.wrp',
        'assets/newton/artifact_manifest.json'
    )
    foreach ($name in $requiredEntries) {
        if (-not $entries.ContainsKey($name)) { Fail "missing required APK entry: $name" }
    }

    $meshManifestEntry = $entries['assets/franka/franka_meshes.json']
    if ($null -eq $meshManifestEntry) { Fail 'missing Franka visual manifest' }
    $meshReader = [System.IO.StreamReader]::new($meshManifestEntry.Open())
    try { $meshManifest = $meshReader.ReadToEnd() | ConvertFrom-Json }
    finally { $meshReader.Dispose() }
    if ($meshManifest.schema_version -ne 1 -or $meshManifest.body_count -ne 12 -or
        $meshManifest.visual_count -ne 11 -or $meshManifest.urdf_sha256 -ne $manifest.urdf_sha256 -or
        @($meshManifest.bodies).Count -ne 12 -or @($meshManifest.assets).Count -ne 10) {
        Fail 'visual manifest does not match the twelve-body/eleven-visual Franka contract'
    }
    $meshNames = @($meshManifest.assets | ForEach-Object { [string]$_.name })
    if (@($meshNames | Sort-Object -Unique).Count -ne $meshNames.Count) { Fail 'duplicate mesh asset basename' }
    for ($bodyIndex = 0; $bodyIndex -lt 12; $bodyIndex++) {
        $body = $meshManifest.bodies[$bodyIndex]
        if ($body.name -ne $manifest.body_names[$bodyIndex]) { Fail 'visual/physics body order mismatch' }
        if ($bodyIndex -eq 8) {
            if ($null -ne $body.asset) { Fail 'empty panda_link8 must not invent visual geometry' }
        } elseif ($body.asset -notin $meshNames) { Fail 'visible body references missing geometry' }
        if (@($body.position).Count -ne 3 -or @($body.rotation).Count -ne 4 -or @($body.scale).Count -ne 3) {
            Fail 'visual pose or scale has incorrect dimensions'
        }
        foreach ($value in @($body.position) + @($body.rotation) + @($body.scale)) {
            if (-not [double]::IsFinite([double]$value)) { Fail 'non-finite visual placement' }
        }
        if (@($body.scale | Where-Object { $_ -le 0 }).Count) { Fail 'visual scale must be positive' }
        $quaternionNorm = 0.0
        foreach ($value in $body.rotation) { $quaternionNorm += [double]$value * [double]$value }
        if ([Math]::Abs($quaternionNorm - 1.0) -gt 0.0001) { Fail 'visual quaternion is not normalized' }
    }
    $meshHash = [System.Security.Cryptography.SHA256]::Create()
    [int64]$meshBytes = 0
    try {
        foreach ($mesh in $meshManifest.assets) {
            if ($mesh.name -notmatch '^[a-z0-9_]+\.qmsh$') { Fail 'unsafe visual asset basename' }
            $meshEntry = $entries["assets/franka/$($mesh.name)"]
            if ($null -eq $meshEntry -or $meshEntry.Length -ne [int64]$mesh.bytes -or
                $meshEntry.Length -lt 20 -or $meshEntry.Length -gt 12MB) { Fail 'mesh is missing or has invalid size' }
            $meshStream = $meshEntry.Open()
            try { $digest = [Convert]::ToHexString($meshHash.ComputeHash($meshStream)).ToLowerInvariant() }
            finally { $meshStream.Dispose() }
            if ($digest -ne $mesh.sha256) { Fail "mesh hash mismatch: $($mesh.name)" }
            $meshStream = $meshEntry.Open()
            $binary = [System.IO.BinaryReader]::new($meshStream)
            try {
                if ([System.Text.Encoding]::ASCII.GetString($binary.ReadBytes(4)) -ne 'QMSH' -or
                    $binary.ReadUInt32() -ne 1 -or $binary.ReadUInt32() -ne 0) { Fail 'invalid mesh header' }
                $vertices = $binary.ReadUInt32()
                $indices = $binary.ReadUInt32()
                if ($vertices -lt 3 -or $vertices -gt 65535 -or $indices -lt 3 -or $indices % 3 -ne 0 -or
                    $vertices -ne $mesh.vertex_count -or $indices -ne $mesh.index_count -or
                    (20L + 12L * $vertices + 2L * $indices) -ne $meshEntry.Length) { Fail 'invalid mesh counts' }
                for ($component = 0; $component -lt 3L * $vertices; $component++) {
                    if (-not [float]::IsFinite($binary.ReadSingle())) { Fail 'mesh has a non-finite vertex' }
                }
                for ($index = 0; $index -lt $indices; $index++) {
                    if ($binary.ReadUInt16() -ge $vertices) { Fail 'mesh index exceeds vertex count' }
                }
            } finally { $binary.Dispose() }
            $meshBytes += $meshEntry.Length
        }
    } finally { $meshHash.Dispose() }
    if ($meshBytes -ge 12MB -or $meshBytes -ne $meshManifest.total_geometry_bytes) { Fail 'mesh geometry budget mismatch' }

    $traceEntry = $entries['assets/verification/controller_trace.json']
    if ($null -eq $traceEntry) { Fail 'missing recorded controller trace' }
    $traceReader = [System.IO.StreamReader]::new($traceEntry.Open())
    try { $controllerTrace = $traceReader.ReadToEnd() | ConvertFrom-Json }
    finally { $traceReader.Dispose() }
    if ($controllerTrace.schema_version -ne 1 -or $controllerTrace.sample_count -ne 1000 -or
        @($controllerTrace.samples).Count -ne 1000 -or $controllerTrace.substeps_per_sample -ne 10 -or
        $controllerTrace.timestep_seconds -ne 0.001 -or $controllerTrace.urdf_sha256 -ne $manifest.urdf_sha256) {
        Fail 'recorded trace does not match the 1000-sample Franka verification contract'
    }
    for ($sampleIndex = 0; $sampleIndex -lt 1000; $sampleIndex++) {
        $sample = $controllerTrace.samples[$sampleIndex]
        if ($sample.index -ne $sampleIndex -or $sample.flags -lt 0 -or $sample.flags -gt 1023 -or
            @($sample.position).Count -ne 3 -or @($sample.rotation).Count -ne 4 -or
            $sample.trigger -lt 0 -or $sample.trigger -gt 1) { Fail 'invalid recorded sample shape or flags' }
        foreach ($value in @($sample.position) + @($sample.rotation) + @($sample.trigger)) {
            if ($null -eq $value -or $value -is [string] -or $value -is [bool] -or
                -not [double]::IsFinite([double]$value)) { Fail 'recorded sample contains a non-numeric or non-finite value' }
        }
        $norm = 0.0
        foreach ($value in $sample.rotation) { $norm += [double]$value * [double]$value }
        if ([Math]::Abs($norm - 1.0) -gt 0.0001) { Fail 'recorded quaternion is not normalized' }
    }
    $expectedAssets = @('assets/newton/artifact_manifest.json', 'assets/newton/franka_reset.wrp',
        'assets/newton/franka_step.wrp', 'assets/franka/franka_meshes.json',
        'assets/verification/controller_trace.json') +
        @($meshNames | ForEach-Object { "assets/franka/$_" })
    $actualAssets = @($entries.Keys | Where-Object { $_ -like 'assets/*' } | Sort-Object)
    if (@($actualAssets | Where-Object { $_ -notin $expectedAssets }).Count -gt 0 -or
        @($expectedAssets | Where-Object { $_ -notin $actualAssets }).Count -gt 0) {
        Fail "runtime/visual asset allowlist mismatch (actual: $($actualAssets -join ', '))"
    }

    $artifactNames = @($manifest.artifacts | ForEach-Object { [string]$_.name })
    if (-not ($artifactNames -contains 'franka_reset.wrp') -or -not ($artifactNames -contains 'franka_step.wrp')) {
        Fail 'artifact manifest does not list both Franka graphs'
    }
    $moduleNames = @($manifest.kernel_modules | ForEach-Object { [string]$_ })
    if ($moduleNames.Count -eq 0) { Fail 'artifact manifest has no kernel modules' }
    foreach ($module in $moduleNames) {
        $entryName = "lib/arm64-v8a/$module"
        if (-not $entries.ContainsKey($entryName)) { Fail "missing kernel module: $entryName" }
    }

    $expectedNative = @('libquest_newton.so', 'libwarp.so', 'libc++_shared.so',
        'libopenxr_loader.so', 'libktx.so') + $moduleNames | Sort-Object -Unique
    $allNativeEntries = @($entries.Keys | Where-Object { $_ -match '^lib/[^/]+/[^/]+$' })
    $actualNative = @($allNativeEntries | ForEach-Object { [System.IO.Path]::GetFileName($_) })
    if (@($actualNative | Where-Object { $_ -notin $expectedNative }).Count -gt 0 -or
        @($expectedNative | Where-Object { $_ -notin $actualNative }).Count -gt 0) {
        Fail "native library allowlist mismatch (actual: $($actualNative -join ', '))"
    }
    if (@($actualNative | Group-Object | Where-Object Count -gt 1).Count -gt 0) {
        Fail 'duplicate native-library basenames'
    }

    foreach ($entryName in $entries.Keys | Where-Object { $_ -like 'assets/newton/*' }) {
        $leaf = [System.IO.Path]::GetFileName($entryName)
        if ($leaf -match '\.(so|dll|o|dex|exe)$') { Fail "executable payload in assets: $entryName" }
    }

    $hash = [System.Security.Cryptography.SHA256]::Create()
    try {
        foreach ($artifact in $manifest.artifacts) {
            $name = [string]$artifact.name
            $candidate = if ($name -like '*.so') { "lib/arm64-v8a/$name" } else { "assets/newton/$name" }
            if (-not $entries.ContainsKey($candidate)) {
                # Native libraries are allowed to have a lib-prefixed APK basename
                # while retaining the artifact's canonical basename/SONAME.
                $candidate = "lib/arm64-v8a/lib$name"
            }
            if (-not $entries.ContainsKey($candidate)) { Fail "manifest artifact is not packaged: $name" }
            $stream = $entries[$candidate].Open()
            try { $digest = [BitConverter]::ToString($hash.ComputeHash($stream)).Replace('-', '').ToLowerInvariant() }
            finally { $stream.Dispose() }
            if ($digest -ne [string]$artifact.sha256) { Fail "hash mismatch for $name" }
            if ($entries[$candidate].Length -ne [int64]$artifact.bytes) { Fail "size mismatch for $name" }
        }
    }
    finally { $hash.Dispose() }

    $aapt2 = $Aapt2Path
    if ([string]::IsNullOrWhiteSpace($aapt2)) { $aapt2 = (Get-Command aapt2.exe -ErrorAction SilentlyContinue).Source }
    if ([string]::IsNullOrWhiteSpace($aapt2) -or -not (Test-Path -LiteralPath $aapt2)) {
        Fail 'aapt2 is required to inspect the merged binary AndroidManifest.xml'
    }
    $xml = (& $aapt2 dump xmltree $apkPath --file AndroidManifest.xml 2>&1 | Out-String)
    if ($LASTEXITCODE -ne 0) { Fail "aapt2 could not decode merged manifest: $xml" }
    foreach ($needle in @(
        'package="com.questnewton"',
        'android.hardware.vr.headtracking',
        'com.oculus.feature.PASSTHROUGH',
        'android.permission.INTERNET',
        'com.oculus.supportedDevices',
        'quest3|quest3s',
        'android.app.lib_name',
        'quest_newton',
        'org.khronos.openxr.intent.category.IMMERSIVE_HMD',
        'com.oculus.intent.category.VR'
    )) {
        if ($xml -notmatch [regex]::Escape($needle)) { Fail "merged manifest missing: $needle" }
    }
    foreach ($needle in @(
        'android:compileSdkVersion\([^\r\n]*\)=34',
        'android:minSdkVersion\([^\r\n]*\)=29',
        'android:targetSdkVersion\([^\r\n]*\)=34',
        'android:extractNativeLibs\([^\r\n]*\)=true',
        'android:glEsVersion\([^\r\n]*\)=0x00030001',
        'android:version\([^\r\n]*\)=1',
        'android:excludeFromRecents\([^\r\n]*\)=true',
        'android:exported\([^\r\n]*\)=true'
    )) {
        if ($xml -notmatch $needle) { Fail "merged manifest missing required value: $needle" }
    }
    $categories = @([regex]::Matches($xml, 'category[^\r\n]*\r?\n[^\r\n]*android:name[^=]*="([^"]+)"') |
        ForEach-Object { $_.Groups[1].Value } | Sort-Object -Unique)
    $expectedCategories = @('android.intent.category.LAUNCHER',
        'org.khronos.openxr.intent.category.IMMERSIVE_HMD', 'com.oculus.intent.category.VR')
    if (@($categories | Where-Object { $_ -notin $expectedCategories }).Count -gt 0 -or
        @($expectedCategories | Where-Object { $_ -notin $categories }).Count -gt 0) {
        Fail "intent category allowlist mismatch (actual: $($categories -join ', '))"
    }

    $readelf = $ReadElfPath
    if ([string]::IsNullOrWhiteSpace($readelf)) { $readelf = (Get-Command llvm-readelf.exe -ErrorAction SilentlyContinue).Source }
    if ([string]::IsNullOrWhiteSpace($readelf) -or -not (Test-Path -LiteralPath $readelf)) {
        Fail 'llvm-readelf is required to inspect native ELF closure'
    }
    $nativeEntries = @($entries.Keys | Where-Object { $_ -match '^lib/arm64-v8a/[^/]+\.so$' })
    $records = @{}
    $temp = Join-Path ([System.IO.Path]::GetTempPath()) ('quest-newton-apk-' + [guid]::NewGuid())
    New-Item -ItemType Directory -Path $temp | Out-Null
    try {
        foreach ($entryName in $nativeEntries) {
            $leaf = [System.IO.Path]::GetFileName($entryName)
            $path = Join-Path $temp $leaf
            [System.IO.Compression.ZipFileExtensions]::ExtractToFile($entries[$entryName], $path)
            $dump = (& $readelf -h -d --wide $path 2>&1 | Out-String)
            if ($LASTEXITCODE -ne 0) { Fail "llvm-readelf failed for $leaf" }
            if ($dump -notmatch 'Class:\s+ELF64' -or $dump -notmatch 'Machine:\s+AArch64') {
                Fail "native library is not ELF64/AArch64: $leaf"
            }
            $soname = [regex]::Match($dump, '\(SONAME\).*\[([^\]]+)\]').Groups[1].Value
            if ([string]::IsNullOrEmpty($soname)) { Fail "missing SONAME: $leaf" }
            if ($records.ContainsKey($soname)) { Fail "duplicate ELF SONAME: $soname" }
            if ($soname -ne $leaf) { Fail "ELF SONAME does not match package basename: $leaf -> $soname" }
            if ($leaf -eq 'libquest_newton.so') {
                $symbols = (& $readelf -Ws --wide $path 2>&1 | Out-String)
                if ($symbols -notmatch '(?m)^\s*\d+:\s+[0-9a-fA-F]+\s+\d+\s+FUNC\s+GLOBAL\s+DEFAULT\s+\d+\s+ANativeActivity_onCreate\s*$') {
                    Fail 'libquest_newton.so must define/export ANativeActivity_onCreate'
                }
                if ($symbols -match '(?m)^\s*\d+:\s+[0-9a-fA-F]+\s+\d+\s+FUNC\s+GLOBAL\s+DEFAULT\s+UND\s+ANativeActivity_onCreate\s*$') {
                    Fail 'ANativeActivity_onCreate is undefined in libquest_newton.so'
                }
            }
            $needed = @([regex]::Matches($dump, '\(NEEDED\).*\[([^\]]+)\]') | ForEach-Object { $_.Groups[1].Value })
            $records[$soname] = @{ Leaf = $leaf; Needed = $needed }
        }
        $system = @('libc.so', 'libm.so', 'libdl.so', 'liblog.so', 'libandroid.so', 'libGLESv3.so', 'libEGL.so', 'libz.so')
        foreach ($record in $records.Values) {
            foreach ($needed in $record.Needed) {
                if ($needed -notin $system -and -not $records.ContainsKey($needed)) {
                    Fail "unpackaged ELF dependency: $($record.Leaf) -> $needed"
                }
            }
        }
    }
    finally { Remove-Item -LiteralPath $temp -Recurse -Force -ErrorAction SilentlyContinue }
}
finally { $archive.Dispose() }

Write-Output "APK contract passed: $apkPath"
