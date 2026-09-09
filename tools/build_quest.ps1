param(
    [string] $ArtifactDir = $env:QUEST_NEWTON_ARTIFACT_DIR,
    [string] $MetaOpenXrSdkRoot = $env:META_OPENXR_SDK_ROOT,
    [string] $AndroidSdkRoot = $(if ($env:ANDROID_SDK_ROOT) { $env:ANDROID_SDK_ROOT } else { $env:ANDROID_HOME }),
    [string] $NdkRoot = $env:ANDROID_NDK_ROOT,
    [string] $JavaHome = $env:JAVA_HOME,
    [string] $FrankaDescriptionRoot = $env:FRANKA_DESCRIPTION_ROOT,
    [string] $HostPython = 'python',
    [string] $CMake = 'cmake',
    [string] $Ninja = 'ninja',
    [string] $FullRuntimeBundle,
    [switch] $StageOnly,
    [string] $BuildRoot = $(if ($env:QXR_BUILD_ROOT) { $env:QXR_BUILD_ROOT } else { Join-Path $PSScriptRoot '../out/quest-build' })
)

$ErrorActionPreference = 'Stop'

function Require-Path([string] $Path, [string] $Description) {
    if ([string]::IsNullOrWhiteSpace($Path) -or -not (Test-Path -LiteralPath $Path)) {
        throw "$Description is required and was not found: $Path"
    }
    return (Resolve-Path -LiteralPath $Path).Path
}

function Require-Command([string] $Command, [string] $Description) {
    $resolved = Get-Command $Command -ErrorAction SilentlyContinue
    if ($null -eq $resolved) { throw "$Description is required: $Command" }
    return $resolved.Source
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$artifactRoot = Require-Path $ArtifactDir 'QUEST_NEWTON_ARTIFACT_DIR'
$metaRoot = Require-Path $MetaOpenXrSdkRoot 'META_OPENXR_SDK_ROOT'
$sdkRoot = Require-Path $AndroidSdkRoot 'ANDROID_SDK_ROOT/ANDROID_HOME'
$ndk = Require-Path $NdkRoot 'ANDROID_NDK_ROOT'
$java = Require-Path $JavaHome 'JAVA_HOME'
$cmakeExe = Require-Command $CMake 'CMake'
$ninjaExe = Require-Command $Ninja 'Ninja'
$frankaRoot = Require-Path $FrankaDescriptionRoot 'FRANKA_DESCRIPTION_ROOT'
$pythonExe = Require-Command $HostPython 'Host Python for mesh conversion'

$metaCommit = (& git -C $metaRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $metaCommit -ne 'bbed2f20e38a5df7113630771c83cb8279e4fc26') {
    throw "Meta OpenXR SDK must be exact v85 commit bbed2f20e38a5df7113630771c83cb8279e4fc26; got $metaCommit"
}
$wrapperSources = @{
    'gradlew' = Join-Path $metaRoot 'Samples\XrSamples\XrInput\Projects\Android\gradlew'
    'gradlew.bat' = Join-Path $metaRoot 'Samples\XrSamples\XrInput\Projects\Android\gradlew.bat'
    'gradle-wrapper.jar' = Join-Path $metaRoot 'Samples\gradle\wrapper\gradle-wrapper.jar'
    'gradle-wrapper.properties' = Join-Path $metaRoot 'Samples\XrSamples\XrInput\Projects\Android\gradle\wrapper\gradle-wrapper.properties'
}
$wrapperHashes = @{
    'gradlew' = 'cc091938da61f618c5ead086e322d9ef54b48af2f64a27d861fc7f9e8e263c80'
    'gradlew.bat' = 'fbc50a5a441b0917ea1bfba260c0717a8416fd54505338f074ccd3f03241ec09'
    'gradle-wrapper.jar' = '33ad4583fd7ee156f533778736fa1b4940bd83b433934d1cc4e9f608e99a6a89'
    'gradle-wrapper.properties' = '144cece7ff8b67045fb90cf603fa4c2d3e0ca6d8ce8742c9b747f1bd8cd36bc3'
}
function Assert-MetaWrapperIntegrity([hashtable] $Files, [hashtable] $Hashes, [string] $MetaPath) {
    $status = ((& git -C $MetaPath status --porcelain --untracked-files=no) -join "`n").Trim()
    if ($LASTEXITCODE -ne 0 -or $status) { throw "Meta SDK tracked tree is dirty: $status" }
    foreach ($key in $Files.Keys) {
        $path = Require-Path $Files[$key] "Meta v85 wrapper source $key"
        $actual = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($actual -ne $Hashes[$key]) { throw "Meta v85 wrapper hash mismatch: $key" }
    }
}
Assert-MetaWrapperIntegrity $wrapperSources $wrapperHashes $metaRoot
$manifestPath = Join-Path $artifactRoot 'artifact_manifest.json'
$manifest = Get-Content -Raw -LiteralPath (Require-Path $manifestPath 'artifact manifest') | ConvertFrom-Json
$artifactNames = @($manifest.artifacts | ForEach-Object { [string]$_.name })
if ($artifactNames -contains 'warp.so' -or -not ($artifactNames -contains 'libwarp.so')) {
    throw 'Artifact bundle is stale: it must contain libwarp.so with matching ELF SONAME, not warp.so'
}

$javaVersion = (& (Join-Path $java 'bin\java.exe') -version 2>&1 | Out-String)
if ($javaVersion -notmatch 'version "17\.') { throw "JDK 17 is required; got $javaVersion" }
if (-not (Test-Path -LiteralPath (Join-Path $sdkRoot 'platforms\android-34'))) {
    throw 'Android SDK Platform 34 is required'
}
if (-not (Test-Path -LiteralPath (Join-Path $ndk 'source.properties'))) {
    throw "NDK source.properties missing: $ndk"
}
if ((Get-Content -Raw -LiteralPath (Join-Path $ndk 'source.properties')) -notmatch 'Pkg\.Revision\s*=\s*27\.0\.12077973') {
    throw 'NDK 27.0.12077973 is required'
}

$staging = Join-Path $BuildRoot 'artifacts'
$native = Join-Path $staging 'jniLibs\arm64-v8a'
$assets = Join-Path $staging 'assets\newton'
$wrapper = Join-Path $BuildRoot 'gradle-wrapper'
New-Item -ItemType Directory -Force -Path $native,$assets,(Join-Path $wrapper 'gradle\wrapper') | Out-Null
Get-ChildItem -LiteralPath $native -Force -ErrorAction SilentlyContinue | Remove-Item -Force
Get-ChildItem -LiteralPath $assets -Force -ErrorAction SilentlyContinue | Remove-Item -Force

foreach ($artifact in $manifest.artifacts) {
    $name = [string]$artifact.name
    $source = Join-Path $artifactRoot $name
    if (-not (Test-Path -LiteralPath $source -PathType Leaf)) { throw "Manifest artifact missing: $source" }
    $record = Get-Item -LiteralPath $source
    $digest = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($record.Length -ne [int64]$artifact.bytes -or $digest -ne [string]$artifact.sha256) {
        throw "Verified artifact hash/size mismatch: $name"
    }
    if ($name -like '*.wrp' -or $name -eq 'artifact_manifest.json') {
        Copy-Item -LiteralPath $source -Destination (Join-Path $assets $name)
    } elseif ($name -like '*.so' -and $name -ne 'libc++_shared.so') {
        if (-not $name.StartsWith('lib')) { throw "Native artifact must be lib-prefixed: $name" }
        Copy-Item -LiteralPath $source -Destination (Join-Path $native $name)
    } elseif ($name -eq 'libc++_shared.so') {
        # AGP/NDK supplies libc++_shared.so; deliberately do not duplicate it
        # in jniLibs. The APK contract compares the packaged NDK copy's hash.
        continue
    } else {
        throw "Artifact is not on the APK runtime allowlist: $name"
    }
}
Copy-Item -LiteralPath $manifestPath -Destination (Join-Path $assets 'artifact_manifest.json') -Force

# Meshes come from the same hash-verified URDF as the physics bundle. The
# converter preserves scene transforms and emits a generated native binding.
$meshOutput = Join-Path $BuildRoot 'franka-meshes'
Push-Location $repoRoot
try {
    & $pythonExe -m tools.assets.convert_franka_meshes `
        --franka-description-root $frankaRoot --artifact-manifest $manifestPath --output $meshOutput
    if ($LASTEXITCODE -ne 0) { throw 'Franka visual mesh conversion/verification failed' }
} finally { Pop-Location }
$meshManifestPath = Join-Path $meshOutput 'franka_meshes.json'
$meshManifest = Get-Content -Raw -LiteralPath $meshManifestPath | ConvertFrom-Json
if ($meshManifest.schema_version -ne 1 -or $meshManifest.body_count -ne 12 -or
    $meshManifest.visual_count -ne 11 -or $meshManifest.urdf_sha256 -ne $manifest.urdf_sha256) {
    throw 'Visual mesh manifest does not match the twelve-body Franka physics bundle'
}
$meshAssets = Join-Path $staging 'assets/franka'
$meshInclude = Join-Path $staging 'include'
New-Item -ItemType Directory -Force -Path $meshAssets,$meshInclude | Out-Null
if (@(Get-ChildItem -LiteralPath $meshAssets -Directory -Force).Count -ne 0) {
    throw 'Unexpected directory in the flat Franka mesh staging area'
}
foreach ($staleFile in Get-ChildItem -LiteralPath $meshAssets -File -Force) {
    Remove-Item -LiteralPath $staleFile.FullName -Force
}
foreach ($mesh in $meshManifest.assets) {
    if ($mesh.name -notmatch '^[a-z0-9_]+\.qmsh$') { throw 'Unsafe mesh asset basename' }
    $meshSource = Join-Path $meshOutput $mesh.name
    if ((Get-Item -LiteralPath $meshSource).Length -ne $mesh.bytes -or
        (Get-FileHash -LiteralPath $meshSource -Algorithm SHA256).Hash.ToLowerInvariant() -ne $mesh.sha256) {
        throw "Visual mesh hash/size mismatch: $($mesh.name)"
    }
    Copy-Item -LiteralPath $meshSource -Destination (Join-Path $meshAssets $mesh.name) -Force
}
Copy-Item -LiteralPath $meshManifestPath -Destination (Join-Path $meshAssets 'franka_meshes.json') -Force
Copy-Item -LiteralPath (Join-Path $meshOutput 'franka_meshes.h') -Destination $meshInclude -Force

$traceOutput = Join-Path $BuildRoot 'controller-trace'
Push-Location $repoRoot
try {
    & $pythonExe -m tools.verification.generate_controller_trace `
        --trace (Join-Path $repoRoot 'tests/device/controller_trace.json') `
        --artifact-manifest $manifestPath --output $traceOutput
    if ($LASTEXITCODE -ne 0) { throw 'Recorded controller trace validation/generation failed' }
} finally { Pop-Location }
$verificationAssets = Join-Path $staging 'assets/verification'
New-Item -ItemType Directory -Force -Path $verificationAssets | Out-Null
if (@(Get-ChildItem -LiteralPath $verificationAssets -Directory -Force).Count -ne 0) {
    throw 'Unexpected directory in verification asset staging'
}
foreach ($staleFile in Get-ChildItem -LiteralPath $verificationAssets -File -Force) {
    Remove-Item -LiteralPath $staleFile.FullName -Force
}
Copy-Item -LiteralPath (Join-Path $traceOutput 'controller_trace_data.h') -Destination $meshInclude -Force
Copy-Item -LiteralPath (Join-Path $traceOutput 'controller_trace.json') -Destination $verificationAssets -Force

if ($FullRuntimeBundle) {
    $fullBundle = Require-Path $FullRuntimeBundle 'full Newton runtime bundle'
    & $pythonExe (Join-Path $repoRoot 'tools/full_port/prepare_apk_runtime.py') --bundle $fullBundle --staging $staging
    if ($LASTEXITCODE -ne 0) { throw 'Full runtime bundle validation/staging failed' }
    Copy-Item -LiteralPath (Join-Path $metaRoot 'Samples/XrSamples/XrInput/assets/panel.ktx') -Destination (Join-Path $staging 'assets/panel.ktx') -Force
}

$sampleAndroid = Join-Path $metaRoot 'Samples\XrSamples\XrInput\Projects\Android'
foreach ($source in @('gradlew','gradlew.bat')) {
    Copy-Item -LiteralPath (Join-Path $sampleAndroid $source) -Destination (Join-Path $wrapper $source) -Force
}
Copy-Item -LiteralPath (Join-Path $metaRoot 'Samples\gradle\wrapper\gradle-wrapper.jar') -Destination (Join-Path $wrapper 'gradle\wrapper\gradle-wrapper.jar') -Force
Copy-Item -LiteralPath (Join-Path $sampleAndroid 'gradle\wrapper\gradle-wrapper.properties') -Destination (Join-Path $wrapper 'gradle\wrapper\gradle-wrapper.properties') -Force
$wrapperProperties = Get-Content -Raw -LiteralPath (Join-Path $wrapper 'gradle\wrapper\gradle-wrapper.properties')
if ($wrapperProperties -notmatch 'gradle-8\.5-bin\.zip') { throw 'Meta v85 Gradle wrapper is not pinned to Gradle 8.5' }
foreach ($key in $wrapperSources.Keys) {
    $target = if ($key -eq 'gradle-wrapper.jar' -or $key -eq 'gradle-wrapper.properties') {
        Join-Path $wrapper ("gradle\wrapper\$key")
    } else { Join-Path $wrapper $key }
    $actual = (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $wrapperHashes[$key]) { throw "Copied v85 wrapper hash mismatch: $key" }
}

$oldJava = $env:JAVA_HOME
$oldSdk = $env:ANDROID_SDK_ROOT
$oldNdk = $env:ANDROID_NDK_ROOT
if ($StageOnly) {
    Write-Output "Quest inputs staged: $staging"
    return
}
try {
    $env:JAVA_HOME = $java
    $env:ANDROID_SDK_ROOT = $sdkRoot
    $env:ANDROID_NDK_ROOT = $ndk
    Assert-MetaWrapperIntegrity $wrapperSources $wrapperHashes $metaRoot
    $gradle = Join-Path $wrapper 'gradlew.bat'
    $fullArguments = @()
    if ($FullRuntimeBundle) {
        $fullArguments = @('-PquestNewtonFullRuntime=true', "-PquestNewtonFullRuntimeDir=$fullBundle", "-PquestNewtonHostPython=$pythonExe")
    }
    & $gradle -p (Join-Path $repoRoot 'quest') --no-daemon :app:assembleDebug `
        "-PquestNewtonArtifactStaging=$staging" `
        "-PmetaOpenXrSdkRoot=$metaRoot" `
        "-PquestNewtonRepoRoot=$repoRoot" @fullArguments
    if ($LASTEXITCODE -ne 0) { throw "Gradle APK build failed with exit code $LASTEXITCODE" }
} finally {
    $env:JAVA_HOME = $oldJava
    $env:ANDROID_SDK_ROOT = $oldSdk
    $env:ANDROID_NDK_ROOT = $oldNdk
}

$apkName = if ($FullRuntimeBundle) { 'quest-newton-full-debug.apk' } else { 'quest-newton-debug.apk' }
$apk = Join-Path $repoRoot "quest/app/build/outputs/apk/debug/$apkName"
if (-not (Test-Path -LiteralPath $apk -PathType Leaf)) { throw "Gradle did not produce expected APK: $apk" }
$aapt2 = Get-ChildItem -LiteralPath (Join-Path $sdkRoot 'build-tools') -Directory | Sort-Object Name -Descending |
    ForEach-Object { Join-Path $_.FullName 'aapt2.exe' } | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
$readelf = Join-Path $ndk 'toolchains\llvm\prebuilt\windows-x86_64\bin\llvm-readelf.exe'
$apksigner = Get-ChildItem -LiteralPath (Join-Path $sdkRoot 'build-tools') -Directory | Sort-Object Name -Descending |
    ForEach-Object { Join-Path $_.FullName 'apksigner.bat' } | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not (Test-Path -LiteralPath $aapt2) -or -not (Test-Path -LiteralPath $readelf) -or -not (Test-Path -LiteralPath $apksigner)) { throw 'aapt2, llvm-readelf, and apksigner are required for APK verification' }
$oldJavaForSigner = $env:JAVA_HOME
$env:JAVA_HOME = $java
try {
    if ($FullRuntimeBundle) {
        & $pythonExe (Join-Path $repoRoot 'tools/full_port/verify_apk.py') --apk $apk --aapt2 $aapt2 --readelf $readelf --apksigner $apksigner --bundle $fullBundle
    } else {
        & pwsh -NoProfile -File (Join-Path $repoRoot 'tests\android_apk_contract.ps1') -Apk $apk -Aapt2Path $aapt2 -ReadElfPath $readelf -ApkSignerPath $apksigner
    }
} finally { $env:JAVA_HOME = $oldJavaForSigner }
if ($LASTEXITCODE -ne 0) { throw 'APK contract verification failed' }
Write-Output "Quest Newton APK complete: $apk"
