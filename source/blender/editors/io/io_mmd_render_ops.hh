/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors/io
 */

#pragma once

struct wmOperatorType;
struct ARegionType;

namespace blender {

void WM_OT_mmd_edge_preview_setup(wmOperatorType *ot);
void WM_OT_mmd_render_set_panel_language(wmOperatorType *ot);

/**
 * Register the "MMD Render" panel in the 3D View sidebar (N-panel) by
 * appending a #PanelType to `art->paneltypes`. Call once during
 * `view3d_buttons_register(art)`. The category is "MMD Render" so render
 * features get their own sidebar tab, separate from the "MMD" physics tab.
 */
void ED_mmd_render_panel_register(ARegionType *art);

}  // namespace blender
