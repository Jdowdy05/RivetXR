<#
Build the pinned Warp CPU runtime and full in-process LLVM/Clang compiler for
Android ARM64. Requires Windows, VS C++ tools, CMake >=3.24, Ninja, NDK27 and a
host Python >=3.10 with NumPy. No device operation or global tool installation.
LLVM cross-build: https://www.llvm.org/docs/HowToCrossCompileLLVM.html
Warp build source: build_llvm.py and CMakeLists.txt at the pinned revision.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$WarpSource,
    [Parameter(Mandatory)][string]$HostPython,
    [Parameter(Mandatory)][string]$AndroidNdk,
    [Parameter(Mandatory)][string]$Ninja,
    [Parameter(Mandatory)][string]$BuildRoot,
    [string]$VsDevShell = 'C:/Program Files/Microsoft Visual Studio/18/Community/Common7/Tools/Launch-VsDevShell.ps1',
    [ValidateRange(1,16)][int]$Parallel = 4,
    [switch]$SkipHostTools
)
$ErrorActionPreference = 'Stop'
$WarpPin = 'd4de134b97b961f1a19bd76830e71ea7f9df2470'
$LlvmPin = 'ca7933e47d3a3451d81e72ac174dcb5aa28b59d1'
$BuildRoot = [IO.Path]::GetFullPath($BuildRoot)
$AndroidNdk = (Resolve-Path -LiteralPath $AndroidNdk).Path.Replace('\','/')
$Ninja = (Resolve-Path -LiteralPath $Ninja).Path.Replace('\','/')
$HostPython = (Resolve-Path -LiteralPath $HostPython).Path.Replace('\','/')
if ($BuildRoot.Length -gt 75) { throw 'Use a short external build root (at most 75 characters) for LLVM/Ninja on Windows.' }
if (-not (Select-String -LiteralPath "$AndroidNdk/source.properties" -Pattern 'Pkg.Revision = 27.0.12077973' -SimpleMatch)) {
    throw 'This port is pinned to Android NDK 27.0.12077973.'
}
New-Item -ItemType Directory -Force "$BuildRoot/logs" | Out-Null
$Root = $BuildRoot.Replace('\','/')
if ([IO.Path]::GetFullPath($WarpSource).TrimEnd('\','/') -eq [IO.Path]::GetFullPath("$Root/warp").TrimEnd('\','/')) {
    throw 'BuildRoot/warp must be an isolated checkout, not the supplied shared WarpSource.'
}
$LogStamp = Get-Date -Format 'yyyyMMdd-HHmmss'
function Invoke-Logged([string]$Name, [string]$Executable, [string[]]$Arguments) {
    $Log = "$Root/logs/$LogStamp-$Name.log"
    Write-Host "$Name -> $Log"
    @{ executable=$Executable; arguments=$Arguments } | ConvertTo-Json -Depth 4 | Set-Content "$Log.command.json"
    & $Executable @Arguments > $Log 2>&1
    if ($LASTEXITCODE -ne 0) {
        Get-Content -LiteralPath $Log -Tail 40 | Write-Host
        throw "$Name failed with exit code $LASTEXITCODE. Full log: $Log"
    }
}
if ((git -C $WarpSource rev-parse HEAD).Trim() -ne $WarpPin) { throw 'WarpSource has the wrong revision.' }
if (-not (Test-Path -LiteralPath "$Root/warp/.git")) {
    Invoke-Logged 'warp-clone' git @('clone','--no-hardlinks',$WarpSource,"$Root/warp")
    Invoke-Logged 'warp-checkout' git @('-C',"$Root/warp",'checkout','--detach',$WarpPin)
}
if ((git -C "$Root/warp" rev-parse HEAD).Trim() -ne $WarpPin) { throw 'Isolated Warp checkout has the wrong revision.' }
$Patch = Join-Path $PSScriptRoot 'patches/warp-android-jit.patch'
& git -C "$Root/warp" apply --reverse --check $Patch 2>$null
if ($LASTEXITCODE -ne 0) { Invoke-Logged 'warp-patch' git @('-C',"$Root/warp",'apply','--check',$Patch); Invoke-Logged 'warp-apply' git @('-C',"$Root/warp",'apply',$Patch) }
# Compare against an index containing exactly the documented patch; do not
# silently label an independently modified source tree as this pinned build.
$PreviousIndex = $env:GIT_INDEX_FILE
try {
    $env:GIT_INDEX_FILE = "$Root/warp-expected.index"
    Invoke-Logged 'warp-reference-index' git @('-C',"$Root/warp",'read-tree',$WarpPin)
    Invoke-Logged 'warp-reference-patch' git @('-C',"$Root/warp",'apply','--cached',$Patch)
    Invoke-Logged 'warp-source-check' git @('-C',"$Root/warp",'diff','--exit-code','--')
} finally {
    $env:GIT_INDEX_FILE = $PreviousIndex
}
if (-not (Test-Path -LiteralPath "$Root/llvm/.git")) {
    $Disk = Get-PSDrive -Name ([IO.Path]::GetPathRoot($BuildRoot).Substring(0,1))
    if ($Disk.Free -lt 20GB) { throw 'LLVM source and native builds require at least 20 GB free space.' }
    Invoke-Logged 'llvm-clone' git @('-c','core.longpaths=true','clone','--depth','1','--single-branch','--branch','llvmorg-22.1.8','https://github.com/llvm/llvm-project.git',"$Root/llvm")
}
if ((git -C "$Root/llvm" rev-parse HEAD).Trim() -ne $LlvmPin) { throw 'LLVM source does not match llvmorg-22.1.8.' }
if (git -C "$Root/llvm" status --porcelain --untracked-files=no) { throw 'LLVM has unexpected tracked source changes.' }
$Common = @('-G','Ninja',"-DCMAKE_MAKE_PROGRAM=$Ninja",'-DLLVM_ENABLE_PROJECTS=clang','-DLLVM_TARGETS_TO_BUILD=AArch64',
    '-DLLVM_INCLUDE_TESTS=OFF','-DLLVM_INCLUDE_BENCHMARKS=OFF','-DLLVM_INCLUDE_EXAMPLES=OFF',
    '-DLLVM_ENABLE_ZLIB=OFF','-DLLVM_ENABLE_ZSTD=OFF','-DLLVM_ENABLE_LIBXML2=OFF',
    '-DLLVM_ENABLE_ASSERTIONS=OFF','-DCLANG_ENABLE_STATIC_ANALYZER=OFF','-DCLANG_ENABLE_OBJC_REWRITER=OFF',
    "-DPython3_EXECUTABLE=$HostPython")
if (-not $SkipHostTools) {
    # Import the compiler environment only for the native table generators.
    & $VsDevShell -Arch amd64 -HostArch amd64 -SkipAutomaticLocation
    Invoke-Logged 'llvm-host-configure' cmake (@('-S',"$Root/llvm/llvm",'-B',"$Root/llvm-host",'-DCMAKE_BUILD_TYPE=Release','-DLLVM_ENABLE_DIA_SDK=OFF') + $Common)
    Invoke-Logged 'llvm-host-build' cmake @('--build',"$Root/llvm-host",'--target','llvm-tblgen','clang-tblgen','--parallel',"$Parallel")
}
foreach ($Tool in 'llvm-tblgen','clang-tblgen') {
    if (-not (Test-Path -LiteralPath "$Root/llvm-host/bin/$Tool.exe")) { throw "Missing native table generator: $Tool" }
}
$Android = @("-DCMAKE_TOOLCHAIN_FILE=$AndroidNdk/build/cmake/android.toolchain.cmake",'-DANDROID_ABI=arm64-v8a',
    '-DANDROID_PLATFORM=android-29','-DANDROID_STL=c++_shared')
Invoke-Logged 'llvm-android-configure' cmake (@('-S',"$Root/llvm/llvm",'-B',"$Root/llvm-android",'-DCMAKE_BUILD_TYPE=MinSizeRel',
    '-DLLVM_HOST_TRIPLE=aarch64-unknown-linux-android29','-DLLVM_DEFAULT_TARGET_TRIPLE=aarch64-unknown-linux-android29',
    "-DLLVM_TABLEGEN=$Root/llvm-host/bin/llvm-tblgen.exe","-DCLANG_TABLEGEN=$Root/llvm-host/bin/clang-tblgen.exe",
    '-DLLVM_BUILD_TOOLS=OFF','-DLLVM_BUILD_UTILS=OFF','-DCLANG_BUILD_TOOLS=OFF',
    '-DLLVM_ENABLE_LIBEDIT=OFF','-DLLVM_ENABLE_LIBPFM=OFF','-DLLVM_ENABLE_PLUGINS=OFF','-DCLANG_PLUGIN_SUPPORT=OFF',
    '-DLLVM_ENABLE_PIC=ON','-DLLVM_ENABLE_EH=OFF','-DLLVM_ENABLE_RTTI=OFF') + $Common + $Android)
# CMake follows dependencies for the compiler frontend, optimizer and AArch64 ORC JIT.
Invoke-Logged 'llvm-android-build' cmake @('--build',"$Root/llvm-android",'--target','clangCodeGen','clangFrontend',
    'LLVMOrcJIT','LLVMOrcDebugging','LLVMRuntimeDyld','LLVMAArch64CodeGen','LLVMAArch64AsmParser','LLVMPasses','LLVMIRReader','LLVMOrcTargetProcess',
    'clang-resource-headers','--parallel',"$Parallel")
$Stage = Join-Path $PSScriptRoot 'stage_warp_android.py'
Invoke-Logged 'stage-llvm-sdk' $HostPython @($Stage,'sdk','--root',$Root)
Invoke-Logged 'warp-configure' cmake (@('-S',"$Root/warp",'-B',"$Root/warp-build",'-G','Ninja',
    "-DCMAKE_MAKE_PROGRAM=$Ninja",'-DCMAKE_BUILD_TYPE=Release','-DWARP_ENABLE_CUDA=OFF','-DWARP_BUILD_CLANG=ON',
    "-DWARP_LLVM_PATH=$Root/llvm-android-sdk","-DPython3_EXECUTABLE=$HostPython") + $Android)
Invoke-Logged 'warp-build' cmake @('--build',"$Root/warp-build",'--parallel',"$Parallel")
Invoke-Logged 'stage-warp-bundle' $HostPython @($Stage,'bundle','--root',$Root,'--ndk',$AndroidNdk,'--output',"$Root/warp-android")
Write-Host "Verified Android ELF bundle: $Root/warp-android. Android JIT execution still requires a device smoke test."
