# SekaiBlender — AI 操控规范

给**任意**会读文件夹的 AI：怎么驱动已经安装好的 SekaiBlender。不是编译手册，不含任何一台机器的绝对路径。

出厂 **不** 启动弹窗、**不** 自动听端口。用户在偏好设置里点过「启动 Local AI Bridge」之后，本机才出现控制面。AI **不替用户开端口**，只指路。

## 两只手

| | 何时 | 怎么做 |
|---|---|---|
| A | 批处理 / 不要 GUI | `SekaiBlender.exe --background --factory-startup --python job.py` |
| B | 对着**已经打开**的 Sekai 窗口 | 标准库 TCP → `127.0.0.1:9876`（Local AI Bridge，官方 Lab 协议） |

同一套 `bpy` 配方。客户端：`docs/agent-control/bridge_client.py`（仅 Python 标准库）。配方：`docs/agent-control/recipes/`。

若宿主已经通过 MCP 连上**同一** Bridge，把它当作第三只手：与手 B 不要叠着写。MCP **不是**使用前提。

未允许不要保存 `.blend`。Bridge 代码里不要 `bpy.ops.wm.quit_blender()` / `sys.exit()`。

## 手 B 完整流程

### 1. 连

对 `127.0.0.1:9876` 发一条 NUL 结尾的 JSON：

```text
{"type":"execute","code":"<python>","strict_json":true}\0
```

回包：

```text
{"status":"ok","result":{...}}\0
{"status":"error","message":"..."}\0
```

执行的 Python **必须**给 `result` 赋一个 `dict`。若命名空间里有可调用的 `check_is_finished`，Bridge 会卡住这条连接直到它返回非 `None`（仅交互式窗口；内部大约一小时上限）。无头模式没有这条 deferred。

`code` 里的文件路径必须先 `json.dumps`，再插进脚本（Windows 反斜杠、括号目录会弄碎字符串）。

超时：identify / scene_info 30s；导入 ≥180s；截图 ≥60s。`receive_timeout` 或断连 → 当作 **unknown**（对面可能还在跑）。**禁止自动把同一请求再发一遍。**

调用 operator 必须 `EXEC_DEFAULT`。`INVOKE_DEFAULT` 会弹出文件选择器，TCP 会挂死。

### 2. 连不上 → 停下，告诉用户

`connection_refused` / `connect_timeout`：**不要扫别的端口，不要自己改偏好设置。** 对用户说：

1. 打开 **SekaiBlender**（窗口标题 / 进程名是 SekaiBlender，不要打开名叫 Blender 的官方安装）。
2. 编辑 → 偏好设置 → **系统**：勾选 **允许在线访问（Allow Online Access）**（官方 Bridge 需要）。
3. 编辑 → 偏好设置 → 扩展：启用 **Local AI Bridge**，点 **启动 Local AI Bridge**。想下次免点，再勾 Auto Start。
4. 等用户说开好了，再重试 **一次**。

### 3. identify（连上后第一件事）

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

仅当 `os.path.basename(binary_path).lower() == "sekaiblender.exe"` 才继续。对不上就停：连到别的程序了。报告 pid / version / filepath，**不擅自存盘**。不要把某台机器的安装路径写进规范或脚本默认值。

### 4. 读

场景摘要：物体数量、类型、骨架名、前若干个名字。不要把全场景 dump 进对话。

`bpy.data.is_dirty` **不能**证明「没改过」。导入过几十个物体后它仍可能是 `False`。看计数、骨架名、Action 名。

### 5. 写

只动用户点名的对象。导入走产品 operator，不要手搓网格冒充 PMX。

| 做什么 | 调用 | 注意 |
|---|---|---|
| 导入 PMX | `bpy.ops.wm.pmx_import("EXEC_DEFAULT", filepath=..., global_scale=0.08)` | 缩放冻结 `0.08`。无头 `--python` 可以（仍有 window 指针） |
| 导入 VMD | `bpy.ops.wm.vmd_import("EXEC_DEFAULT", filepath=...)` | 先把目标 Armature 设为 active；多骨架必须显式指定 |
| 视口 PNG | 对 `VIEW_3D` 的 window region 调 `bpy.ops.render.opengl` | 先藏默认 Cube（0.08 的角色会被它挡住）。只要活窗口 |
| 物理烘焙 | `bpy.ops.wm.mmd_physics_bake` | 本客户端路径 **未验证** |

### 6. 截图

只在活窗口。先 `Cube.hide_set(True)`（或隐藏用户没要的默认立方体）。无头没有可用 3D 视口，不要做 OpenGL 截图。

### 7. 收工

未允许不 `save_mainfile()`，不退出 Sekai。

## 无头（手 A）

- **可以：** `--background --factory-startup --python` 做 `pmx_import` / `vmd_import`。
- **不可以：** `check_is_finished` 长任务；VIEW_3D OpenGL 截图。
- 无头是**另一个进程**，不会改用户正开着的窗口。

## 与已有 MCP

手 B 和「宿主里已经连上的 Blender MCP」都是 `127.0.0.1:9876` 的**客户端**。Sekai 自己在听。不抢端口，**抢场景**。

- 同一窗口同一时刻 **一只手写**。已有 MCP 工具就用 MCP 写，不要再叠 TCP 写入。MCP 空闲时，TCP 只读 identify / scene_info 可以。
- MCP 正在跑、或状态是 unknown / timed_out 时，不要用 TCP 做导入 / 烘焙 / 截图。MCP 的锁管不住 TCP。
- 确认连的是 **SekaiBlender**（identify 基名）。不要把 Sekai 配方发到别的 Blender 进程。
- 手 A 不碰 GUI 进程。TCP 写完之后，MCP 侧应重新读场景，不要沿用旧物体列表。

## 控制台编码

中日文物体名在 Windows 控制台可能乱码，UTF-8 JSON 文件仍是对的。以 JSON 为准，不要去「修编码」。

## execute 脚本格式

`code` 在 Sekai 进程内执行。最小合法脚本：

```python
import bpy
result = {"ok": True, "scene": bpy.context.scene.name}
```

模板（改属性 / 节点 / 集合都套这一层）：

```python
import bpy

error = None
try:
    obj = bpy.data.objects.get(NAME)  # NAME 由客户端 json.dumps 进来
    if obj is None:
        error = "object not found"
    else:
        # 只改用户点名的那一项
        pass
except Exception as ex:
    error = type(ex).__name__ + ": " + str(ex)

result = {
    "ok": error is None,
    "error": error,
    # 再加少量验收字段：计数、名字、改前/改后
}
```

| 要 | 不要 |
|---|---|
| `result = dict(...)` | 只 `print`、返回 list/str、不赋值 |
| 客户端 `json.dumps` 插路径和物体名 | 在 code 里手写 `C:\foo\bar.pmx` |
| `"EXEC_DEFAULT"` | `"INVOKE_DEFAULT"` |
| 一次一件事 | 导入+烘焙+截图塞一条 |
| 截图 `--output` 由调用方给 | 给 agent 宿主做 PNG 沙箱 / 强制 artifacts 目录 |
| UTF-8 JSON 当真值 | 用 Windows 控制台判断中文名 |

节点：`mat.node_tree.nodes` / `.links`。集合：`bpy.data.collections.new`、`col.objects.link`、`layer_collection.exclude`。属性面板：改 RNA（`light.energy`、`obj.location`、`cam.data.lens`）。这些没有单独菜谱，用上面的模板。

## 固定配方（skills）

| 配方 | 手 | 脚本 |
|---|---|---|
| identify | B | `docs/agent-control/recipes/identify.py` |
| scene_info | B（也可当 A 的 bpy） | `docs/agent-control/recipes/scene_info.py` |
| import_pmx | A 或 B | `docs/agent-control/recipes/import_pmx.py --filepath ...` |
| import_vmd | A 或 B | `docs/agent-control/recipes/import_vmd.py --filepath ... [--armature 名]` |
| capture | B | `docs/agent-control/recipes/capture.py --output ...` |
| headless job | A | `docs/agent-control/recipes/headless_job.py` |

路径用命令行参数或环境变量传入。本目录不要写死某台机器的盘符路径。截图输出路径由调用方决定，不做路径沙箱。
