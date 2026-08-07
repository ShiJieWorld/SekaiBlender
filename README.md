# SekaiBlender 1.0

**MMD 特化版 Blender** — 在 Blender 内原生承接 PMX/VMD 资产、角色动画与实时物理的独立分支。

基于 Blender 主线源码深度定制，面向 MMD（MikuMikuDance）创作者，提供从模型导入、动作编辑、物理模拟到最终渲染的完整工作流。

---

## 核心能力

### 资产管线
- **PMX 2.0 导入** — 完整解析骨骼（IK/追加/轴约束）、网格（BDEF/SDEF 蒙皮）、材质、UV、顶点 Morph（Shape Key + Driver）、Group Morph、Display Frame
- **PMX 2.0 导出** — 往返保真导出：模型信息、顶点、面、纹理、材质、骨骼、Morph、Display Frame、刚体、关节，已通过真实模型逐字段验证

### 动画系统
- **VMD 导入** — 骨骼 Action（位置/四元数/Bezier 插值）、Morph Action（顶点/Group/Bone/Flip/Impulse）、Camera Action
- **VMD 导出** — 骨骼 Action（线性/Bezier）、Camera Action（逐通道 Bezier 无损往返）
- **原生 CCD IK** — V8 求解器，兼容 PMX IK 定义 schema 1/2，角度限制

### 物理引擎
- **实时物理预览** — 帧权威重建、多模型同场景、Bullet 独立世界、120Hz 子步
- **离线物理烘焙** — 完整 Action Bake，modal 进度/取消恢复、NLA 兼容
- **物理诊断** — 快照采样、穿透检测、收敛追踪

### 视觉增强
- **AMD FSR 1.0** — EASU/RCAS 视口超分，四档质量预设
- **MMD 描边预览** — PMX edge_flag/edge_size 驱动的 toon 描边
- **EEVEE 渲染** — 自适应阴影池、中文界面、30 FPS 默认时间线

---

## 构建（Windows）

### 依赖
- Visual Studio 2022（MSVC v143+）
- CMake 3.21+
- Python 3.11（嵌入用）
- CUDA Toolkit（可选，GPU 渲染加速）
- OptiX SDK（可选）

### 步骤

```bash
# 1. 进入源码目录
cd SekaiBlender

# 2. 配置（Release + GPU）
cmake -B ../build -G "Visual Studio 17 2022" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DWITH_CYCLES_CUDA_BINARIES=ON ^
  -DWITH_CYCLES_DEVICE_OPTIX=ON

# 3. 构建
cmake --build ../build --config Release --target INSTALL

# 4. 运行
../build/bin/Release/SekaiBlender.exe
```

> **注意**：SekaiBlender 基于 Blender 主线深度修改，并非插件。构建产物为独立可执行程序，与官方 Blender 不冲突。

---

## 与官方 Blender 的关系

SekaiBlender 是 Blender 的 GPL 兼容分支：
- 上游同步自 [blender/blender](https://projects.blender.org/blender/blender) 主线
- 所有修改位于独立的 `main` 分支
- 遵循 GPL v2+ 许可证

---

## 许可证

本项目继承 Blender 的 [GNU General Public License v2.0 或更高版本](https://www.gnu.org/licenses/gpl-2.0.html)。

Copyright (C) 2024-2026 世界的歌 (ShiJieWorld) and Blender Foundation.

---

## 链接

- Blender 官方: https://www.blender.org
- PMX 格式规范（非官方）: https://gist.github.com/felixjones/f8a06bd48f9da44a1cc9b71c14f0f3b5
