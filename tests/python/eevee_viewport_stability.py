#!/usr/bin/env python3

import os
import time
import traceback

import bpy


TEST_SECONDS = float(os.environ.get('BLENDER_TEST_SECONDS', '45'))
FSR_QUALITY = os.environ.get('BLENDER_FSR_QUALITY')
start_time = time.monotonic()


def finish(result):
    print(f'EEVEE_VIEWPORT_STABILITY result={result} seconds={time.monotonic() - start_time:.2f}')
    bpy.ops.wm.quit_blender()


def redraw():
    try:
        elapsed = time.monotonic() - start_time
        if elapsed >= TEST_SECONDS:
            finish('PASS')
            return None
        for window in bpy.context.window_manager.windows:
            for area in window.screen.areas:
                if area.type == 'VIEW_3D':
                    area.tag_redraw()
        return 0.05
    except Exception:
        traceback.print_exc()
        finish('FAIL')
        return None


scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
if hasattr(scene.eevee, 'use_fsr_viewport'):
    scene.eevee.use_fsr_viewport = FSR_QUALITY is not None
    if FSR_QUALITY is not None:
        scene.eevee.fsr_quality = FSR_QUALITY
print(f'EEVEE_VIEWPORT_STABILITY quality={FSR_QUALITY or "NATIVE"} seconds={TEST_SECONDS}')
for area in bpy.context.screen.areas:
    if area.type == 'VIEW_3D':
        area.spaces.active.shading.type = 'RENDERED'
bpy.app.timers.register(redraw, first_interval=0.1)
