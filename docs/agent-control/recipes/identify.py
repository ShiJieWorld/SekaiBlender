# Hand B: identify the process behind 127.0.0.1:9876.
# Usage: python identify.py
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
EXPECTED_BASENAME = "sekaiblender.exe"

IDENTIFY_CODE = """
import bpy
import os
result = {
    "pid": os.getpid(),
    "binary_path": os.path.normcase(os.path.realpath(bpy.app.binary_path)),
    "version": str(bpy.app.version_string),
    "filepath": str(bpy.data.filepath),
    "scene": str(bpy.context.scene.name),
    "dirty": bool(bpy.data.is_dirty),
    "background": bool(bpy.app.background),
}
""".strip()


def main() -> int:
    try:
        identity = execute_result(IDENTIFY_CODE, host=HOST, port=PORT)
    except BridgeError as ex:
        print(json.dumps({"ok": False, "error_code": ex.code, "message": ex.message}, ensure_ascii=False, indent=2))
        return 1
    basename = os.path.basename(str(identity.get("binary_path", ""))).lower()
    ok = basename == EXPECTED_BASENAME
    report = {"ok": ok, "host": HOST, "port": PORT, "identity": identity}
    if not ok:
        report["error_code"] = "wrong_binary"
        report["message"] = f"binary basename {basename!r} is not {EXPECTED_BASENAME}"
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
