# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import math
import os
import tempfile
import traceback

import bpy


def render_viewport(directory, name, use_fsr, sharpness=0.2, quality=None):
    scene = bpy.context.scene
    scene.eevee.use_fsr_viewport = use_fsr
    scene.eevee.fsr_sharpness = sharpness
    if quality is not None:
        scene.eevee.fsr_quality = quality
    scene.render.filepath = os.path.join(directory, name + '.exr')

    result = bpy.ops.render.opengl(write_still=True, view_context=True)
    if result != {'FINISHED'}:
        raise RuntimeError(f'Viewport render failed: {result}')

    image = bpy.data.images.load(scene.render.filepath, check_existing=False)
    try:
        pixels = list(image.pixels)
        if tuple(image.size) != (257, 193):
            raise AssertionError(f'Unexpected viewport size: {tuple(image.size)}')
        if not pixels or not all(math.isfinite(value) for value in pixels):
            raise AssertionError('Viewport output contains invalid pixels')
        return pixels
    finally:
        bpy.data.images.remove(image)


def run():
    try:
        scene = bpy.context.scene
        scene.render.engine = 'BLENDER_EEVEE'
        scene.render.resolution_x = 257
        scene.render.resolution_y = 193
        scene.render.resolution_percentage = 100
        scene.render.image_settings.file_format = 'OPEN_EXR'
        scene.render.image_settings.color_mode = 'RGBA'
        scene.eevee.taa_samples = 1

        for area in bpy.context.screen.areas:
            if area.type == 'VIEW_3D':
                area.spaces.active.shading.type = 'RENDERED'
                area.spaces.active.overlay.show_overlays = True

        with tempfile.TemporaryDirectory() as directory:
            native = render_viewport(directory, 'viewport_native', False)
            fsr = render_viewport(directory, 'viewport_fsr', True, 0.2, 'PERFORMANCE')
            quality = render_viewport(directory, 'viewport_quality', True, 0.2, 'QUALITY')
            unsharpened = render_viewport(directory, 'viewport_unsharpened', True, 0.0)
            sharpened = render_viewport(directory, 'viewport_sharpened', True, 1.0)

        error = sum(abs(a - b) for a, b in zip(native, fsr)) / len(fsr)
        if not 1.0e-6 < error < 0.3:
            raise AssertionError(f'Unexpected viewport mean absolute error: {error}')
        quality_error = sum(abs(a - b) for a, b in zip(quality, fsr)) / len(fsr)
        if quality_error <= 1.0e-7:
            raise AssertionError('Viewport quality preset did not affect the output')
        sharpness_error = sum(
            abs(a - b) for a, b in zip(unsharpened, sharpened)
        ) / len(sharpened)
        if sharpness_error <= 1.0e-6:
            raise AssertionError('Viewport sharpness change did not affect the output')
        print(f'FSR_VIEWPORT_TEST result=PASS size=257x193 mean_absolute_error={error:.8f}')
    except Exception:
        traceback.print_exc()
        print('FSR_VIEWPORT_TEST result=FAIL')
    finally:
        bpy.ops.wm.quit_blender()


bpy.app.timers.register(run, first_interval=1.0)
