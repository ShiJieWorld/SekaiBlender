<div align="center">
  <img src="release/windows/msix/Assets/StoreLogo.scale-400.png" width="180" alt="SekaiBlender icon">
  <h1>SekaiBlender</h1>
  <p><strong>MMD-native workflows for Blender.</strong><br>
  PMX/VMD assets, animation, IK, physics and rendering in one Windows build.</p>
  <p>
    <a href="https://github.com/ShiJieWorld/SekaiBlender/releases/latest"><img src="https://img.shields.io/github/v/release/ShiJieWorld/SekaiBlender?style=flat-square&display_name=release&label=release&color=16a34a" alt="Latest release"></a>
    <a href="https://github.com/ShiJieWorld/SekaiBlender/blob/main/COPYING"><img src="https://img.shields.io/badge/license-GPL--2.0%2B-2563eb?style=flat-square" alt="GPL-2.0+"></a>
    <img src="https://img.shields.io/badge/platform-Windows_x64-f97316?style=flat-square" alt="Windows x64">
    <img src="https://img.shields.io/badge/Blender-5.3-2563eb?style=flat-square" alt="Blender 5.3">
  </p>
</div>

<p align="center">
  <a href="#download">Download</a> ·
  <a href="#feature-set">Features</a> ·
  <a href="#build-on-windows">Build</a> ·
  <a href="#known-limits">Known limits</a> ·
  <a href="#contributing">Contributing</a>
</p>

> SekaiBlender is a source-level Blender branch for MMD creators. It is a standalone application, not an add-on, and it does not replace an existing Blender installation.

## Download

A prebuilt Windows x64 package is published on the [Releases](https://github.com/ShiJieWorld/SekaiBlender/releases/latest) page, so building from source is optional.

1. Download `sekaiblender-1.0-git.<hash>-windows64.zip` (about 329 MB) from the latest release.
2. Extract it anywhere. No installer and no administrator rights are required.
3. Run `SekaiBlender-launcher.exe` from the extracted folder. Use `SekaiBlender.exe` directly if you want the console window attached.

Keep the extracted folder intact: the executables load the `5.3` runtime directory that sits next to them.

## At A Glance

| Workflow | Included |
| --- | --- |
| **PMX** | Native import and export, materials, UVs, morphs, bones, rigid bodies and joints |
| **VMD** | Bone, morph and camera animation with Bezier interpolation |
| **IK** | Native CCD V8 solver with PMX schema 1/2 support and angle limits |
| **Physics** | Realtime Bullet preview, multi-model scenes and deterministic Action Bake |
| **Viewport** | MMD toon outlines, EEVEE workflows and AMD FSR 1.0 upscaling |

## Feature Set

### PMX Asset Pipeline

- PMX 2.0 import for BDEF/SDEF skinning, IK, append transforms, axis limits, materials, UVs, morphs and display frames.
- PMX export for imported models, including source retention, vertex morph offsets, physics definitions and packaged textures.
- Round-trip validation against real PMX models with field-by-field comparison across all 11 major sections.

### Animation And Camera

- VMD bone and morph Actions with position, quaternion and Bezier interpolation.
- Native VMD camera import and export with perspective/orthographic support.
- Camera rig export preserves per-channel Bezier curves when the source uses the supported rig layout.

### Realtime Physics

- Frame-authoritative realtime preview with independent Bullet worlds per model.
- Multiple MMD models in the same Scene.
- F8 Action Bake with progress, cancellation recovery, NLA validation and provenance metadata.

### Rendering And Look Development

- MMD edge preview driven by PMX `edge_flag` and `edge_size`.
- AMD FSR 1.0 EASU/RCAS quality presets for the EEVEE viewport and renders.
- SekaiBlender branding, Chinese UI defaults and a 30 FPS startup timeline.

## Verified Workflows

| Check | Result |
| --- | --- |
| PMX / VMD / Blender kernel tests | 3/3 CTest suites passing |
| PMX, MMD and VMD direct tests | 170 passed, 11 environment-gated skips, 0 failed |
| PMX export round-trip | 2 real PMX models, all 11 sections matched |
| VMD camera round-trip | Real 27-frame sample, zero resampling error |

## Build On Windows

### Requirements

- Visual Studio 2022 with MSVC v143 or newer
- CMake 3.21 or newer
- Python 3.11 for the embedded runtime
- Git LFS and Git submodules
- CUDA Toolkit and OptiX SDK are optional GPU acceleration dependencies

### Clone

```powershell
git clone --recurse-submodules https://github.com/ShiJieWorld/SekaiBlender.git
cd SekaiBlender
git lfs install
git lfs pull
git submodule update --init --recursive
```

The repository uses Git LFS for Blender test assets and other binary resources. The initial LFS download is large.

### Configure, Build And Run

```powershell
cmake -S . -B ..\build -G "Visual Studio 17 2022" -A x64 `
  -DWITH_CYCLES_CUDA_BINARIES=ON `
  -DWITH_CYCLES_DEVICE_OPTIX=ON

cmake --build ..\build --config Release --target INSTALL
..\build\bin\Release\SekaiBlender.exe
```

For a CPU-only build, omit the CUDA and OptiX options. The `INSTALL` target is intentional: it refreshes the runtime `5.3/scripts` tree used by the packaged executable.

## Known Limits

- VMD light and self-shadow frames are not imported or exported.
- PMX Bone, UV, Material, Flip and Impulse Morph data do not all map to active Blender effects yet.
- PMX export currently accepts models imported by SekaiBlender with source-retention data; arbitrary native Blender meshes are rejected rather than exported with guessed semantics.
- Realtime physics sessions are independent per model within one Scene. Different Scenes cannot own simultaneous global realtime schedulers.

## Project Status

This GitHub `main` branch is an intentionally history-trimmed source snapshot. It contains the complete current source tree and SekaiBlender modifications, while the upstream Blender commit history is kept out of this public snapshot to keep the repository practical to clone and maintain.

## Contributing

Bug reports and focused pull requests are welcome. For changes to PMX, VMD, IK or physics, include the smallest reproducible asset or test case available and describe the expected Blender-side result.

## License And Upstream

SekaiBlender is a GPL-compatible Blender branch. It inherits the [GNU General Public License v2.0 or later](https://www.gnu.org/licenses/gpl-2.0.html).

- Upstream Blender: [blender/blender](https://projects.blender.org/blender/blender)
- Blender website: [blender.org](https://www.blender.org)
- PMX format reference: [PMX format notes](https://gist.github.com/felixjones/f8a06bd48f9da44a1cc9b71c14f0f3b5)

Copyright (C) 2024-2026 世界的歌 (ShiJieWorld) and Blender Foundation.

<p align="center"><sub>Built for creators who bring MMD worlds to life.</sub></p>
