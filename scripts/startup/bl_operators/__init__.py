# SPDX-FileCopyrightText: 2009-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

from __future__ import annotations

# support reloading sub-modules
if "bpy" in locals():
    from importlib import reload
    _modules_loaded[:] = [reload(val) for val in _modules_loaded]
    del reload

_modules = [
    "add_mesh_torus",
    "anim",
    "assets",
    "bone_selection_sets",
    "clip",
    "connect_to_output",
    "console",
    "constraint",
    "copy_global_transform",
    "file",
    "geometry_nodes",
    "grease_pencil",
    "image",
    "image_as_planes",
    "mesh",
    "node",
    "object",
    "object_align",
    "object_quick_effects",
    "object_randomize_transform",
    "presets",
    "render",
    "rigidbody",
    "screen_play_rendered_anim",
    "sequencer",
    "spreadsheet",
    "userpref",
    "uvcalc_follow_active",
    "uvcalc_lightmap",
    "uvcalc_transform",
    "vertexpaint_dirt",
    "view3d",
    "world",
    "wm",
]

import bpy

if bpy.app.build_options.freestyle:
    _modules.append("freestyle")

__import__(name=__name__, fromlist=_modules)
_namespace = globals()
_modules_loaded = [_namespace[name] for name in _modules]
del _namespace


def register():
    from bpy.utils import register_class
    from . import (
        bone_selection_sets,
        copy_global_transform,
    )

    for mod in _modules_loaded:
        for cls in mod.classes:
            register_class(cls)

    from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty, StringProperty
    from sekaiblender_sponsors import SORT_ITEMS

    def _splash_sponsor_sort_update(self, _context):
        self.splash_sponsor_page = 0

    def _splash_sponsor_desc_update(self, _context):
        self.splash_sponsor_page = 0

    def _splash_sponsor_query_update(self, _context):
        self.splash_sponsor_page = 0

    bpy.types.WindowManager.splash_sponsor_page = IntProperty(
        name="Sponsor Page",
        description="Current page of the splash sponsor list",
        default=0,
        min=0,
        options={'SKIP_SAVE'},
    )
    bpy.types.WindowManager.splash_sponsor_sort = EnumProperty(
        name="Sponsor Sort",
        description="How the splash sponsor list is ordered",
        items=SORT_ITEMS,
        default='TIME',
        options={'SKIP_SAVE'},
        update=_splash_sponsor_sort_update,
    )
    bpy.types.WindowManager.splash_sponsor_cell_scale = FloatProperty(
        name="Sponsor Cell Scale",
        description="Vertical scale of splash sponsor grid cells so the list fills the right column",
        default=1.15,
        min=0.8,
        max=1.6,
        options={'SKIP_SAVE'},
    )
    bpy.types.WindowManager.splash_recent_scale = FloatProperty(
        name="Recent File Scale",
        description="Vertical scale of splash recent-file rows so they fill the left column",
        default=1.0,
        min=0.9,
        max=2.6,
        options={'SKIP_SAVE'},
    )
    bpy.types.WindowManager.splash_recent_rows = IntProperty(
        name="Recent File Rows",
        description="How many recent files the splash lists",
        default=6,
        min=1,
        max=12,
        options={'SKIP_SAVE'},
    )
    bpy.types.WindowManager.splash_sponsor_desc = BoolProperty(
        name="Sponsor Descending",
        description="When true, newest / highest amount first",
        default=True,
        options={'SKIP_SAVE'},
        update=_splash_sponsor_desc_update,
    )
    bpy.types.WindowManager.splash_sponsor_query = StringProperty(
        name="Sponsor Search",
        description="Filter splash sponsors by name",
        default="",
        options={'SKIP_SAVE', 'TEXTEDIT_UPDATE'},
        update=_splash_sponsor_query_update,
    )

    bone_selection_sets.register()
    copy_global_transform.register()


def unregister():
    from bpy.utils import unregister_class
    from . import (
        bone_selection_sets,
        copy_global_transform,
    )

    bone_selection_sets.unregister()
    copy_global_transform.unregister()

    if hasattr(bpy.types.WindowManager, "splash_sponsor_page"):
        del bpy.types.WindowManager.splash_sponsor_page
    if hasattr(bpy.types.WindowManager, "splash_sponsor_sort"):
        del bpy.types.WindowManager.splash_sponsor_sort
    if hasattr(bpy.types.WindowManager, "splash_sponsor_cell_scale"):
        del bpy.types.WindowManager.splash_sponsor_cell_scale
    if hasattr(bpy.types.WindowManager, "splash_recent_scale"):
        del bpy.types.WindowManager.splash_recent_scale
    if hasattr(bpy.types.WindowManager, "splash_recent_rows"):
        del bpy.types.WindowManager.splash_recent_rows
    if hasattr(bpy.types.WindowManager, "splash_sponsor_query"):
        del bpy.types.WindowManager.splash_sponsor_query
    if hasattr(bpy.types.WindowManager, "splash_sponsor_desc"):
        del bpy.types.WindowManager.splash_sponsor_desc

    for mod in reversed(_modules_loaded):
        for cls in reversed(mod.classes):
            if cls.is_registered:
                unregister_class(cls)
