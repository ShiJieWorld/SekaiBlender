# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import statistics
import time

import bpy
from mathutils import Vector


def build_scene():
    bpy.ops.wm.read_homefile(use_empty=True, use_factory_startup=True)
    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_EEVEE'
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.resolution_percentage = 100
    scene.eevee.taa_render_samples = 16

    material = bpy.data.materials.new('FSR Benchmark Material')
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    principled = nodes.get('Principled BSDF')
    noise = nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 7.0
    noise.inputs['Detail'].default_value = 8.0
    noise.inputs['Roughness'].default_value = 0.7
    links.new(noise.outputs['Fac'], principled.inputs['Base Color'])
    principled.inputs['Metallic'].default_value = 0.35
    principled.inputs['Roughness'].default_value = 0.28

    for y in range(-3, 4):
        for x in range(-6, 7):
            bpy.ops.mesh.primitive_ico_sphere_add(
                subdivisions=2,
                radius=0.42,
                location=(x * 0.9, y * 0.9, 0.45 + 0.08 * ((x + y) & 1)),
            )
            bpy.context.object.data.materials.append(material)

    bpy.ops.mesh.primitive_plane_add(size=30, location=(0, 0, 0))
    bpy.context.object.data.materials.append(material)

    bpy.ops.object.camera_add(location=(9.5, -12.0, 9.0))
    camera = bpy.context.object
    camera.rotation_euler = (Vector((0, 0, 1.0)) - camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera.data.lens = 52
    scene.camera = camera

    for location, energy, size in [((-5, -4, 10), 1600, 5), ((6, -1, 7), 1200, 4), ((0, 7, 5), 900, 3)]:
        bpy.ops.object.light_add(type='AREA', location=location)
        light = bpy.context.object
        light.data.energy = energy
        light.data.shape = 'DISK'
        light.data.size = size
        light.rotation_euler = (Vector((0, 0, 0)) - light.location).to_track_quat('-Z', 'Y').to_euler()

    return scene


def render_times(scene, use_fsr, count=3):
    scene.eevee.use_fsr_render = use_fsr
    bpy.ops.render.render()
    times = []
    for _ in range(count):
        start = time.perf_counter()
        bpy.ops.render.render()
        times.append(time.perf_counter() - start)
    return times


scene = build_scene()
native = render_times(scene, False)
fsr = render_times(scene, True)
native_median = statistics.median(native)
fsr_median = statistics.median(fsr)
print(
    'FSR_BENCHMARK '
    f'native={native_median:.6f}s fsr={fsr_median:.6f}s '
    f'speedup={native_median / fsr_median:.3f}x '
    f'native_samples={native} fsr_samples={fsr}'
)
