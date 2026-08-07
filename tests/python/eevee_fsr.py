# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import math
import os
import tempfile
import unittest

import bpy


class EeveeFsrTest(unittest.TestCase):
    def setUp(self):
        bpy.ops.wm.read_homefile(use_empty=False, use_factory_startup=True)
        self.scene = bpy.context.scene
        self.scene.render.engine = 'BLENDER_EEVEE'
        self.scene.render.resolution_percentage = 100
        self.scene.render.image_settings.file_format = 'OPEN_EXR'
        self.scene.render.image_settings.color_mode = 'RGBA'
        self.scene.eevee.taa_render_samples = 2

    def render_pixels(
            self, directory, name, width, height, use_fsr, sharpness=0.2, quality=None):
        scene = self.scene
        scene.render.resolution_x = width
        scene.render.resolution_y = height
        scene.render.filepath = os.path.join(directory, name + '.exr')
        scene.eevee.use_fsr_render = use_fsr
        scene.eevee.fsr_sharpness = sharpness
        if quality is not None:
            scene.eevee.fsr_quality = quality

        self.assertEqual(bpy.ops.render.render(write_still=True), {'FINISHED'})
        image = bpy.data.images.load(scene.render.filepath, check_existing=False)
        try:
            pixels = list(image.pixels)
            self.assertEqual(tuple(image.size), (width, height))
            self.assertEqual(len(pixels), width * height * 4)
            self.assertTrue(all(math.isfinite(value) for value in pixels))
            return pixels
        finally:
            bpy.data.images.remove(image)

    def test_settings_save_load(self):
        props = self.scene.eevee
        self.assertFalse(props.use_fsr_viewport)
        self.assertFalse(props.use_fsr_render)
        self.assertAlmostEqual(props.fsr_sharpness, 0.2)
        self.assertEqual(props.fsr_quality, 'PERFORMANCE')

        props.use_fsr_viewport = True
        props.use_fsr_render = True
        props.fsr_sharpness = 0.75
        props.fsr_quality = 'QUALITY'

        with tempfile.TemporaryDirectory() as directory:
            filepath = os.path.join(directory, 'eevee_fsr_settings.blend')
            bpy.ops.wm.save_as_mainfile(filepath=filepath, check_existing=False)
            bpy.ops.wm.open_mainfile(filepath=filepath, load_ui=False)

        props = bpy.context.scene.eevee
        self.assertTrue(props.use_fsr_viewport)
        self.assertTrue(props.use_fsr_render)
        self.assertAlmostEqual(props.fsr_sharpness, 0.75)
        self.assertEqual(props.fsr_quality, 'QUALITY')

    def test_render_output(self):
        self.scene.render.film_transparent = True
        self.scene.eevee.use_overscan = True
        self.scene.eevee.overscan_size = 5.0

        with tempfile.TemporaryDirectory() as directory:
            native = self.render_pixels(directory, 'native', 65, 49, False)
            fsr = self.render_pixels(directory, 'fsr', 65, 49, True)

            alpha = fsr[3::4]
            self.assertLess(min(alpha), 0.01)
            self.assertGreater(max(alpha), 0.99)

            mean_absolute_error = sum(abs(a - b) for a, b in zip(native, fsr)) / len(fsr)
            self.assertGreater(mean_absolute_error, 1.0e-5)
            self.assertLess(mean_absolute_error, 0.2)

            view_layer = self.scene.view_layers[0]
            view_layer.use_pass_z = True
            view_layer.use_pass_normal = True
            view_layer.use_pass_vector = True
            quality_results = {}
            for quality in ('ULTRA_QUALITY', 'QUALITY', 'BALANCED', 'PERFORMANCE'):
                quality_results[quality] = self.render_pixels(
                    directory,
                    'fsr_' + quality.lower(),
                    67,
                    51,
                    True,
                    quality=quality,
                )
            for quality_a, quality_b in zip(
                    ('ULTRA_QUALITY', 'QUALITY', 'BALANCED'),
                    ('QUALITY', 'BALANCED', 'PERFORMANCE')):
                difference = sum(
                    abs(a - b)
                    for a, b in zip(quality_results[quality_a], quality_results[quality_b])
                ) / len(quality_results[quality_a])
                self.assertGreater(difference, 1.0e-7)

            unsharpened = self.render_pixels(directory, 'fsr_unsharpened', 65, 49, True, 0.0)
            sharpened = self.render_pixels(directory, 'fsr_sharpened', 65, 49, True, 1.0)
            sharpness_error = sum(
                abs(a - b) for a, b in zip(unsharpened, sharpened)
            ) / len(sharpened)
            self.assertGreater(sharpness_error, 1.0e-6)

            resized = self.render_pixels(directory, 'fsr_resized', 97, 55, True)
            self.assertGreater(max(resized), min(resized))

            self.scene.render.use_border = True
            self.scene.render.use_crop_to_border = False
            self.scene.render.border_min_x = 0.17
            self.scene.render.border_max_x = 0.83
            self.scene.render.border_min_y = 0.11
            self.scene.render.border_max_y = 0.89
            bordered = self.render_pixels(directory, 'fsr_border', 83, 61, True)
            self.assertGreater(max(bordered), min(bordered))


if __name__ == '__main__':
    unittest.main(argv=[__file__])
