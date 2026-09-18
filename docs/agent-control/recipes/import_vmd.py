# Hand A or B: import a VMD onto the active armature. Does not save.
# Usage: python import_vmd.py --filepath path/to/motion.vmd [--armature NAME]
# Env: SEKAIBLENDER_BRIDGE_HOST, SEKAIBLENDER_BRIDGE_PORT, SEKAIBLENDER_IMPORT_TIMEOUT

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


def build_code(filepath: str, armature: str | None) -> str:
    return f"""
import bpy

vmd = {json.dumps(filepath)}
wanted = {json.dumps(armature)}
arm = None
error = None
if wanted:
    arm = bpy.data.objects.get(wanted)
    if arm is None or arm.type != "ARMATURE":
        error = "armature not found: " + str(wanted)
    else:
        bpy.ops.object.select_all(action="DESELECT")
        arm.select_set(True)
        bpy.context.view_layer.objects.active = arm
else:
    arm = next((obj for obj in bpy.context.scene.objects if obj.type == "ARMATURE"), None)
    if arm is not None:
        bpy.ops.object.select_all(action="DESELECT")
        arm.select_set(True)
        bpy.context.view_layer.objects.active = arm

poll_ok = bool(bpy.ops.wm.vmd_import.poll())
op_result = None
if error is None and arm is None:
    error = "no armature in the scene; import a PMX first or pass --armature"
if error is None and not poll_ok:
    error = "vmd_import poll returned False"
if error is None:
    try:
        op_result = list(bpy.ops.wm.vmd_import(
            "EXEC_DEFAULT",
            filepath=vmd,
            frame_offset=0,
            replace_existing_action=True,
            import_morphs=True,
        ))
    except Exception as ex:
        error = type(ex).__name__ + ": " + str(ex)

action_name = None
if arm is not None and arm.animation_data and arm.animation_data.action:
    action_name = arm.animation_data.action.name
result = {{
    "vmd": vmd,
    "armature": None if arm is None else arm.name,
    "poll_ok": poll_ok,
    "op_result": op_result,
    "error": error,
    "action": action_name,
}}
""".strip()


def main() -> int:
    parser = argparse.ArgumentParser(description="Import VMD over Bridge TCP")
    parser.add_argument("--filepath", required=True, help="Path to a .vmd file")
    parser.add_argument("--armature", default=None, help="Target armature object name")
    args = parser.parse_args()
    vmd = Path(args.filepath)
    if not vmd.is_file():
        print(json.dumps({"ok": False, "error_code": "vmd_missing", "path": str(vmd)}, ensure_ascii=False, indent=2))
        return 1
    try:
        imported = execute_result(
            build_code(str(vmd), args.armature),
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
