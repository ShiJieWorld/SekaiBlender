# SPDX-FileCopyrightText: 2026 SekaiBlender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy


# Start from Blender Dark so editor backgrounds remain neutral for production work.
bpy.ops.preferences.reset_default_theme()

theme = bpy.context.preferences.themes[0]
theme.name = "SekaiBlender"

# Restrict branding to subdued interaction feedback rather than workspace surfaces.
accent = (0.45, 0.39, 0.62, 1.0)
accent_light = (0.60, 0.53, 0.80, 1.0)
ui = theme.user_interface

for widget_name in (
    "wcol_regular",
    "wcol_tool",
    "wcol_toolbar_item",
    "wcol_radio",
    "wcol_option",
    "wcol_toggle",
    "wcol_num",
    "wcol_numslider",
    "wcol_menu",
    "wcol_menu_back",
    "wcol_menu_item",
    "wcol_tooltip",
    "wcol_progress",
    "wcol_list_item",
):
    getattr(ui, widget_name).inner_sel = accent

ui.wcol_num.item = accent_light
ui.wcol_numslider.item = accent_light
ui.wcol_progress.item = accent_light
ui.panel_active = accent
theme.common.anim.playhead = accent_light[:3]
