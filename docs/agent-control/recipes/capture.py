# Hand B only: hide default Cube, VIEW_3D OpenGL capture, camera fallback.
# Usage: python capture.py --output capture.png
# Env: SEKAIBLENDER_BRIDGE_HOST, SEKAIBLENDER_BRIDGE_PORT, SEKAIBLENDER_CAPTURE_TIMEOUT
# Restores frame / camera / render filepath / format / resolution. Does not save the blend.

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
CAPTURE_TIMEOUT_S = float(os.environ.get("SEKAIBLENDER_CAPTURE_TIMEOUT", "120"))


def build_code(output: str) -> str:
    return f"""
import bpy
from pathlib import Path

_OUTPUT_PATH = {json.dumps(output)}
scene = bpy.context.scene
render = scene.render
old_frame = scene.frame_current
old_subframe = getattr(scene, "frame_subframe", 0.0)
old_camera = scene.camera
old_filepath = render.filepath
old_format = render.image_settings.file_format
old_resolution = (render.resolution_x, render.resolution_y, render.resolution_percentage)
actual_mode = None
fallback_reason = None
viewport_info = None
hidden = []
try:
    cube = bpy.data.objects.get("Cube")
    if cube is not None and not cube.hide_get():
        cube.hide_set(True)
        hidden.append("Cube")
    output = Path(_OUTPUT_PATH)
    output.parent.mkdir(parents=True, exist_ok=True)
    viewport = None
    window_manager = getattr(bpy.context, "window_manager", None)
    if window_manager is not None:
        for window_index, window in enumerate(window_manager.windows):
            screen = getattr(window, "screen", None)
            if screen is None:
                continue
            for area_index, area in enumerate(screen.areas):
                if area.type != "VIEW_3D":
                    continue
                region = next((item for item in area.regions if item.type == "WINDOW"), None)
                if region is not None:
                    viewport = (window, area, region, window_index, area_index)
                    break
            if viewport is not None:
                break
    if viewport is not None:
        window, area, region, window_index, area_index = viewport
        try:
            with bpy.context.temp_override(window=window, area=area, region=region):
                if not bpy.ops.render.opengl.poll():
                    raise RuntimeError("VIEW_3D cannot run OpenGL capture")
                op_result = bpy.ops.render.opengl(
                    "EXEC_DEFAULT",
                    animation=False,
                    sequencer=False,
                    view_context=True,
                    write_still=False,
                )
            if "FINISHED" not in op_result:
                raise RuntimeError("Viewport OpenGL capture did not finish")
            actual_mode = "viewport_opengl"
            viewport_info = {{"window_index": window_index, "area_index": area_index}}
        except Exception as ex:
            fallback_reason = "viewport_opengl_failed: " + str(ex)
    else:
        fallback_reason = "no_usable_view3d_window_region"
    if actual_mode is None:
        camera = scene.camera
        if camera is None or camera.type != "CAMERA":
            raise RuntimeError("No usable camera is available for camera render")
        scene.camera = camera
        render.image_settings.file_format = "PNG"
        op_result = bpy.ops.render.render(
            "EXEC_DEFAULT",
            animation=False,
            write_still=False,
            use_viewport=False,
        )
        if "FINISHED" not in op_result:
            raise RuntimeError("Camera render did not finish")
        actual_mode = "camera_render"
    image = bpy.data.images.get("Render Result")
    if image is None:
        raise RuntimeError("No Render Result image is available")
    render.image_settings.file_format = "PNG"
    image.save_render(str(output), scene=scene)
    with output.open("rb") as stream:
        header = stream.read(24)
    if len(header) != 24 or header[:16] != b"\\x89PNG\\r\\n\\x1a\\n\\x00\\x00\\x00\\x0dIHDR":
        raise RuntimeError("Did not write a PNG with an IHDR header")
    result = {{
        "output": str(output),
        "bytes": output.stat().st_size,
        "mode": actual_mode,
        "fallback_reason": fallback_reason,
        "viewport": viewport_info,
        "hidden": hidden,
        "scene": scene.name,
        "frame": scene.frame_current,
        "camera": scene.camera.name if scene.camera else None,
        "resolution": {{
            "width": int.from_bytes(header[16:20], "big"),
            "height": int.from_bytes(header[20:24], "big"),
        }},
    }}
except Exception as ex:
    result = {{
        "error": type(ex).__name__ + ": " + str(ex),
        "mode": actual_mode,
        "fallback_reason": fallback_reason,
        "hidden": hidden,
    }}
finally:
    scene.camera = old_camera
    render.filepath = old_filepath
    render.image_settings.file_format = old_format
    render.resolution_x, render.resolution_y, render.resolution_percentage = old_resolution
    try:
        scene.frame_set(old_frame, subframe=old_subframe)
    except TypeError:
        scene.frame_set(old_frame)
    if "Cube" in hidden:
        cube = bpy.data.objects.get("Cube")
        if cube is not None:
            cube.hide_set(False)
""".strip()


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture a PNG over Bridge TCP")
    parser.add_argument("--output", required=True, help="Destination .png path")
    args = parser.parse_args()
    output = Path(args.output)
    try:
        captured = execute_result(
            build_code(str(output)),
            host=HOST,
            port=PORT,
            recv_timeout=CAPTURE_TIMEOUT_S,
        )
    except BridgeError as ex:
        print(json.dumps({"ok": False, "error_code": ex.code, "message": ex.message}, ensure_ascii=False, indent=2))
        return 1
    ok = "error" not in captured and Path(captured.get("output", "")).is_file()
    print(json.dumps({"ok": ok, "capture": captured, "saved_blend": False}, ensure_ascii=False, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
