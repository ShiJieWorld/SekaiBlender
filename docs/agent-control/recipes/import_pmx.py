# Hand A or B: import a PMX with frozen global_scale=0.08. Does not save.
# Usage: python import_pmx.py --filepath path/to/model.pmx
# Env: SEKAIBLENDER_BRIDGE_HOST, SEKAIBLENDER_BRIDGE_PORT, SEKAIBLENDER_IMPORT_TIMEOUT
# Hand A: run the inner bpy block via SekaiBlender.exe --background --python ...

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge_client import BridgeError, execute_result

HOST = os.environ.get("SEKAIBLENDER_BRIDGE_HOST", "127.0.0.1")
PORT = int(os.environ.get("SEKAIBLENDER_BRIDGE_PORT", "9876"))
IMPORT_TIMEOUT_S = float(os.environ.get("SEKAIBLENDER_IMPORT_TIMEOUT", "180"))


def build_code(filepath: str) -> str:
    return f"""
import bpy

pmx = {json.dumps(filepath)}
poll_ok = bool(bpy.ops.wm.pmx_import.poll())
op_result = None
error = None
if not poll_ok:
    error = "pmx_import poll returned False"
else:
    try:
        op_result = list(bpy.ops.wm.pmx_import(
            "EXEC_DEFAULT",
            filepath=pmx,
            global_scale=0.08,
            split_by_material=True,
        ))
    except Exception as ex:
        error = type(ex).__name__ + ": " + str(ex)

armatures = [obj.name for obj in bpy.context.scene.objects if obj.type == "ARMATURE"]
result = {{
    "pmx": pmx,
    "poll_ok": poll_ok,
    "op_result": op_result,
    "error": error,
    "object_count": len(bpy.context.scene.objects),
    "mesh_count": sum(1 for obj in bpy.context.scene.objects if obj.type == "MESH"),
    "armatures": armatures,
}}
""".strip()


def main() -> int:
    parser = argparse.ArgumentParser(description="Import PMX over Bridge TCP")
    parser.add_argument("--filepath", required=True, help="Path to a .pmx file")
    args = parser.parse_args()
    pmx = Path(args.filepath)
    if not pmx.is_file():
        print(json.dumps({"ok": False, "error_code": "pmx_missing", "path": str(pmx)}, ensure_ascii=False, indent=2))
        return 1
    try:
        imported = execute_result(
            build_code(str(pmx)),
            host=HOST,
            port=PORT,
            recv_timeout=IMPORT_TIMEOUT_S,
        )
    except BridgeError as ex:
        print(json.dumps({"ok": False, "error_code": ex.code, "message": ex.message}, ensure_ascii=False, indent=2))
        return 1
    finished = imported.get("error") is None and "FINISHED" in (imported.get("op_result") or [])
    print(json.dumps({"ok": bool(finished), "import": imported, "saved": False}, ensure_ascii=False, indent=2))
    return 0 if finished else 1


if __name__ == "__main__":
    sys.exit(main())
