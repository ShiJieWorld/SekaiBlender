# Hand B (also usable as bpy inside hand A): scene summary, truncated object list.
# Usage: python scene_info.py
# Env: SEKAIBLENDER_BRIDGE_HOST, SEKAIBLENDER_BRIDGE_PORT

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge_client import BridgeError, execute_result

HOST = os.environ.get("SEKAIBLENDER_BRIDGE_HOST", "127.0.0.1")
PORT = int(os.environ.get("SEKAIBLENDER_BRIDGE_PORT", "9876"))
NAME_LIMIT = 20

SCENE_INFO_CODE = f"""
import bpy

scene = bpy.context.scene
objects = []
types = {{}}
armatures = []
for obj in scene.objects:
    types[obj.type] = types.get(obj.type, 0) + 1
    entry = {{"name": obj.name, "type": obj.type}}
    data = obj.data
    if obj.type == "MESH" and data is not None:
        entry["vertices"] = len(data.vertices)
        entry["polygons"] = len(data.polygons)
    elif obj.type == "ARMATURE" and data is not None:
        entry["bones"] = len(data.bones)
        armatures.append(obj.name)
    objects.append(entry)

result = {{
    "scene": scene.name,
    "filepath": str(bpy.data.filepath),
    "object_count": len(objects),
    "types": types,
    "armatures": armatures,
    "objects_preview": objects[:{NAME_LIMIT}],
    "materials_preview": [mat.name for mat in bpy.data.materials[:{NAME_LIMIT}]],
    "frame": scene.frame_current,
    "frame_range": [int(scene.frame_start), int(scene.frame_end)],
}}
""".strip()


def main() -> int:
    try:
        scene = execute_result(SCENE_INFO_CODE, host=HOST, port=PORT)
    except BridgeError as ex:
        print(json.dumps({"ok": False, "error_code": ex.code, "message": ex.message}, ensure_ascii=False, indent=2))
        return 1
    print(json.dumps({"ok": True, "host": HOST, "port": PORT, "scene": scene}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
