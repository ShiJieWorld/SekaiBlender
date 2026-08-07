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

void WM_OT_mmd_physics_start(wmOperatorType *ot);
void WM_OT_mmd_physics_use_bake_source(wmOperatorType *ot);
void WM_OT_mmd_physics_set_realtime_hz(wmOperatorType *ot);
void WM_OT_mmd_physics_set_dynamic_constraint_iterations(wmOperatorType *ot);
void WM_OT_mmd_physics_set_panel_language(wmOperatorType *ot);
void WM_OT_mmd_physics_set_bake_quality(wmOperatorType *ot);
void WM_OT_mmd_physics_step(wmOperatorType *ot);
void WM_OT_mmd_physics_reset(wmOperatorType *ot);
void WM_OT_mmd_physics_bake(wmOperatorType *ot);
void WM_OT_mmd_physics_bake_cancel(wmOperatorType *ot);
void WM_OT_mmd_physics_capture_diagnostics(wmOperatorType *ot);
void WM_OT_mmd_physics_snapshot_diagnostics(wmOperatorType *ot);
void WM_OT_mmd_physics_export_definition(wmOperatorType *ot);
void WM_OT_mmd_physics_stop(wmOperatorType *ot);

/**
 * Register the "MMD Physics" panel in the 3D View sidebar (N-panel) by
 * appending a #PanelType to `art->paneltypes`. Call once during
 * `view3d_buttons_register(art)`. Category is "MMD" so the panel appears
 * under its own tab in the N-panel.
 */
void ED_mmd_physics_panel_register(ARegionType *art);

}  // namespace blender
