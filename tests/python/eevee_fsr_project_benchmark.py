# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import argparse
import statistics
import sys
import time

import bpy


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    return parser.parse_args(argv)


def render(scene, use_fsr, label):
    scene.eevee.use_fsr_render = use_fsr
    print(f'FSR_PROJECT_BENCHMARK phase={label} status=START', flush=True)
    start = time.perf_counter()
    result = bpy.ops.render.render()
    elapsed = time.perf_counter() - start
    if result != {'FINISHED'}:
        raise RuntimeError(f'Render failed during {label}: {result}')
    print(f'FSR_PROJECT_BENCHMARK phase={label} status=FINISH time={elapsed:.6f}s', flush=True)
    return elapsed


args = parse_args()
scene = bpy.context.scene
if scene.render.engine != 'BLENDER_EEVEE':
    raise RuntimeError(f'Expected EEVEE scene, got {scene.render.engine}')

print(
    'FSR_PROJECT_BENCHMARK '
    f'scene={scene.name!r} resolution={scene.render.resolution_x}x{scene.render.resolution_y} '
    f'percentage={scene.render.resolution_percentage} samples={scene.eevee.taa_render_samples} '
    f'objects={len(scene.objects)}',
    flush=True,
)

render(scene, False, 'native_warmup')
render(scene, True, 'fsr_warmup')
native_times = []
fsr_times = []
for index in range(4):
    if index % 2 == 0:
        native_times.append(render(scene, False, f'native_timed_{index}'))
        fsr_times.append(render(scene, True, f'fsr_timed_{index}'))
    else:
        fsr_times.append(render(scene, True, f'fsr_timed_{index}'))
        native_times.append(render(scene, False, f'native_timed_{index}'))
native_time = statistics.median(native_times)
fsr_time = statistics.median(fsr_times)
print(
    'FSR_PROJECT_BENCHMARK result=PASS '
    f'native={native_time:.6f}s fsr={fsr_time:.6f}s '
    f'speedup={native_time / fsr_time:.3f}x '
    f'native_samples={native_times} fsr_samples={fsr_times}',
    flush=True,
)
