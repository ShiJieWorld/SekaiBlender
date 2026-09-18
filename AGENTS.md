# SekaiBlender — AI 操控规范

给任意会读本仓库的 AI：怎么驱动**已经安装、正在运行**的 SekaiBlender。不是编译手册，不含某台机器的路径。

出厂不弹窗、不自动听端口。用户在偏好设置里点过「启动 Local AI Bridge」之后才有控制面。AI 不替用户开端口。

详细配方与客户端：`docs/agent-control.md`、`docs/agent-control/bridge_client.py`、`docs/agent-control/recipes/`。

## 选哪只手

1. 对着已经打开的 SekaiBlender 窗口改场景 / 截视口 → TCP `127.0.0.1:9876`（Local AI Bridge，官方 Lab 协议）。宿主若已用 MCP 连上同一端口，用 MCP 写，不要再叠 TCP 写入。
2. 批处理、不要 GUI → `SekaiBlender.exe --background --factory-startup --python job.py`。无头可导入 PMX/VMD；不可视口截图、不可 `check_is_finished`。无头是另一个进程，不改开着的窗。
3. 先确认连的是 SekaiBlender，不要把配方发到名叫 Blender 的官方安装。

## 活窗口流程

1. **连。** 对 9876 发 NUL JSON：`{"type":"execute","code":"...","strict_json":true}`。代码必须给 `result` 赋 dict。路径用 `json.dumps` 再插进 code。operator 用 `EXEC_DEFAULT`（`INVOKE` 会弹文件框）。导入超时 ≥180s，截图 ≥60s。超时当 unknown，禁止自动重发同一请求。
2. **连不上 → 停下告诉用户**（不要扫端口、不要改偏好设置）：打开 SekaiBlender → 编辑 → 偏好设置 → 系统 → 勾选「允许在线访问」→ 扩展里打开 **Local AI Bridge** → 点「启动 Local AI Bridge」（可勾 Auto Start）。用户说开好了再试一次。
3. **identify（必须先做）。**

```python
import bpy, os
result = {
    "pid": os.getpid(),
    "binary_path": os.path.normcase(os.path.realpath(bpy.app.binary_path)),
    "version": str(bpy.app.version_string),
    "filepath": str(bpy.data.filepath),
    "scene": str(bpy.context.scene.name),
    "dirty": bool(bpy.data.is_dirty),
    "background": bool(bpy.app.background),
}
```

仅当 `os.path.basename(binary_path).lower() == "sekaiblender.exe"` 才继续。对不上就停。不擅自存盘。
4. **读。** 摘要：物体计数、类型、骨架名、前若干名字。不要 dump 全场景。不要用 `bpy.data.is_dirty` 当「没改过」的证据。
5. **写。** 只动用户点名的对象。`bpy.ops.wm.pmx_import("EXEC_DEFAULT", filepath=..., global_scale=0.08)`（缩放冻结）。VMD：先把目标 Armature 设为 active。
6. **截图。** 先藏默认 Cube（0.08 角色会被挡住）。只要活窗口 VIEW_3D OpenGL。
7. **收工。** 未允许不 `save_mainfile()`，不 `quit_blender()`。

## execute 脚本格式

发进 Bridge 的 `code` 是普通 Python，在 **Sekai 进程里**跑，不是你这边的脚本。固定骨架：

```python
import bpy
# ... 只做这一件事 ...
result = {"ok": True, "key": value}  # 必须是 dict；不要 print 当协议
```

硬规则：

- 必须赋值 `result = { ... }`。漏了或不是 dict → 协议失败。
- 不要 `sys.exit()` / `quit_blender()` / `save_mainfile()`（除非用户明确要存）。
- 路径先在 **客户端** `json.dumps(path)` 再插进 `code`，不要手写 Windows 反斜杠。
- operator：`"EXEC_DEFAULT"`，不要 `"INVOKE_DEFAULT"`。
- 一次请求只做一件可描述的事；读和写分开。返回计数、名字、错误字符串，不要把整份 bpy 数据 dump 进对话。
- 截图路径由**调用方**决定（用户给的目录或 agent 自己的工作目录）。不要做 PNG 路径沙箱，不要把文件强制写进某个宿主的 artifacts 目录。

客户端：从 `docs/agent-control/bridge_client.py` 调用 `execute_result(code)`（该文件没有命令行入口）。完整说明：`docs/agent-control.md`。

## 固定配方（skills）

优先跑现成脚本，不要从零发明导入/截图。路径相对本仓库（或安装目录里同一份 `docs/`）：

| skill | 何时用 | 怎么跑 |
|---|---|---|
| identify | 连上后第一件事 | `python docs/agent-control/recipes/identify.py` |
| scene_info | 认当前工程 | `python docs/agent-control/recipes/scene_info.py` |
| import_pmx | 用户给了 .pmx | `python docs/agent-control/recipes/import_pmx.py --filepath <pmx>` |
| import_vmd | 用户给了 .vmd | `python docs/agent-control/recipes/import_vmd.py --filepath <vmd> [--armature 名]` |
| capture | 活窗口要图 | `python docs/agent-control/recipes/capture.py --output <png>` |
| headless_job | 无 GUI 导入 | `SekaiBlender.exe --background --factory-startup --python docs/agent-control/recipes/headless_job.py -- --filepath <pmx>` |

环境变量（可选）：`SEKAIBLENDER_BRIDGE_HOST`（默认 127.0.0.1）、`SEKAIBLENDER_BRIDGE_PORT`（默认 9876）、`SEKAIBLENDER_IMPORT_TIMEOUT`、`SEKAIBLENDER_CAPTURE_TIMEOUT`。

节点 / 集合 / RNA：没有单独 exe，按 `docs/agent-control.md` 里的 execute 格式自己写一小段，规则同上。

## 与 MCP

9876 是 SekaiBlender 在听。MCP 和 TCP 都是客户端：不抢端口，抢场景。同一窗口同一时刻一只手写。MCP 正在跑或 unknown 时不要用 TCP 写入。
