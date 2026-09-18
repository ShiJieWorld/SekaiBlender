# Hand A: run inside SekaiBlender.exe --background --factory-startup --python this_file.py -- --filepath model.pmx
# Identify + optional PMX import. No OpenGL capture. Does not save.

from __future__ import annotations

import argparse
import json
import os
import sys

import bpy


def _parse_args() -> argparse.Namespace:
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    parser = argparse.ArgumentParser(description="Headless identify + optional PMX import")
    parser.add_argument("--filepath", default=None, help="Optional .pmx to import")
    return parser.parse_args(argv)


def main() -> int:
    args = _parse_args()
    identity = {
        "pid": os.getpid(),
        "binary_path": os.path.normcase(os.path.realpath(bpy.app.binary_path)),
        "version": str(bpy.app.version_string),
        "background": bool(bpy.app.background),
    }
    basename = os.path.basename(identity["binary_path"]).lower()
    if basename != "sekaiblender.exe":
        print(json.dumps({"ok": False, "error_code": "wrong_binary", "identity": identity}, ensure_ascii=False))
        return 1
    imported = None
    if args.filepath:
        pmx = args.filepath
        poll_ok = bool(bpy.ops.wm.pmx_import.poll())
        op_result = None
        error = None
        if not poll_ok:
            error = "pmx_import poll returned False"
        else:
            try:
                op_result = list(
                    bpy.ops.wm.pmx_import(
                        "EXEC_DEFAULT",
                        filepath=pmx,
                        global_scale=0.08,
                        split_by_material=True,
                    )
                )
            except Exception as ex:
                error = type(ex).__name__ + ": " + str(ex)
        imported = {
            "pmx": pmx,
            "poll_ok": poll_ok,
            "op_result": op_result,
            "error": error,
            "object_count": len(bpy.context.scene.objects),
            "armatures": [obj.name for obj in bpy.context.scene.objects if obj.type == "ARMATURE"],
        }
        ok = error is None and "FINISHED" in (op_result or [])
    else:
        ok = True
    print(
        json.dumps(
            {"ok": ok, "identity": identity, "import": imported, "saved": False},
            ensure_ascii=False,
        )
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
