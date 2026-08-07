/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "io_ops.hh" /* own include */

#include "WM_api.hh"

#ifdef WITH_ALEMBIC
#  include "io_alembic.hh"
#endif

#ifdef WITH_USD
#  include "io_usd.hh"
#endif

#ifdef WITH_IO_FBX
#  include "io_fbx_ops.hh"
#endif

#include "io_cache.hh"
#include "io_drop_import_file.hh"
#include "io_grease_pencil.hh"
#include "io_obj.hh"
#include "io_ply_ops.hh"
#include "io_stl_ops.hh"
#include "io_pmx_ops.hh"
#include "io_vmd_ops.hh"
#ifdef WITH_IO_PMX
#  include "io_mmd_physics_ops.hh"
#  include "io_mmd_render_ops.hh"
#endif

namespace blender {

void ED_operatortypes_io()
{
#ifdef WITH_ALEMBIC
  WM_operatortype_append(WM_OT_alembic_import);
  WM_operatortype_append(WM_OT_alembic_export);
  ed::io::alembic_file_handler_add();
#endif
#ifdef WITH_USD
  WM_operatortype_append(WM_OT_usd_import);
  WM_operatortype_append(WM_OT_usd_export);
  ed::io::usd_file_handler_add();
#endif

#ifdef WITH_IO_GREASE_PENCIL
  WM_operatortype_append(WM_OT_grease_pencil_import_svg);
  ed::io::grease_pencil_file_handler_add();
#  ifdef WITH_PUGIXML
  WM_operatortype_append(WM_OT_grease_pencil_export_svg);
#  endif
#  ifdef WITH_HARU
  WM_operatortype_append(WM_OT_grease_pencil_export_pdf);
#  endif
#endif

  WM_operatortype_append(CACHEFILE_OT_open);
  WM_operatortype_append(CACHEFILE_OT_reload);

  WM_operatortype_append(CACHEFILE_OT_layer_add);
  WM_operatortype_append(CACHEFILE_OT_layer_remove);
  WM_operatortype_append(CACHEFILE_OT_layer_move);
#ifdef WITH_IO_WAVEFRONT_OBJ
  WM_operatortype_append(WM_OT_obj_export);
  WM_operatortype_append(WM_OT_obj_import);
  ed::io::obj_file_handler_add();
#endif

#ifdef WITH_IO_PLY
  WM_operatortype_append(WM_OT_ply_export);
  WM_operatortype_append(WM_OT_ply_import);
  ed::io::ply_file_handler_add();
#endif

#ifdef WITH_IO_STL
  WM_operatortype_append(WM_OT_stl_import);
  WM_operatortype_append(WM_OT_stl_export);
  ed::io::stl_file_handler_add();
#endif

#ifdef WITH_IO_PMX
  WM_operatortype_append(WM_OT_pmx_import);
  WM_operatortype_append(WM_OT_pmx_export);
  WM_operatortype_append(WM_OT_pmx_apply_ik);
  WM_operatortype_append(WM_OT_pmx_apply_append_transform);
  WM_operatortype_append(WM_OT_pmx_apply_fixed_axis);
  WM_operatortype_append(WM_OT_pmx_apply_local_axis);
  WM_operatortype_append(WM_OT_pmx_capture_pose_snapshot);
  ed::io::pmx_file_handler_add();
#endif

  WM_operatortype_append(WM_OT_vmd_import);
  WM_operatortype_append(WM_OT_vmd_camera_import);
  WM_operatortype_append(WM_OT_vmd_export);
  WM_operatortype_append(WM_OT_vmd_camera_export);
  ed::io::vmd_file_handler_add();

#ifdef WITH_IO_PMX
  /* MMD real-time physics operators. Start is modal (TIMER-driven);
   * Stop / Step / Reset / Capture Diagnostics are exec operators. */
  WM_operatortype_append(WM_OT_mmd_physics_start);
  WM_operatortype_append(WM_OT_mmd_physics_use_bake_source);
  WM_operatortype_append(WM_OT_mmd_physics_set_realtime_hz);
  WM_operatortype_append(WM_OT_mmd_physics_set_dynamic_constraint_iterations);
  WM_operatortype_append(WM_OT_mmd_physics_set_panel_language);
  WM_operatortype_append(WM_OT_mmd_physics_set_bake_quality);
  WM_operatortype_append(WM_OT_mmd_physics_step);
  WM_operatortype_append(WM_OT_mmd_physics_reset);
  WM_operatortype_append(WM_OT_mmd_physics_bake);
  WM_operatortype_append(WM_OT_mmd_physics_bake_cancel);
  WM_operatortype_append(WM_OT_mmd_physics_capture_diagnostics);
  WM_operatortype_append(WM_OT_mmd_physics_export_definition);
  WM_operatortype_append(WM_OT_mmd_physics_snapshot_diagnostics);
  WM_operatortype_append(WM_OT_mmd_physics_stop);

  /* MMD render operators (own "MMD Render" sidebar tab). */
  WM_operatortype_append(WM_OT_mmd_render_set_panel_language);
  WM_operatortype_append(WM_OT_mmd_edge_preview_setup);
#endif

#ifdef WITH_IO_FBX
  WM_operatortype_append(WM_OT_fbx_import);
  ed::io::fbx_file_handler_add();
#endif

  WM_operatortype_append(WM_OT_drop_import_file);
  ED_dropbox_drop_import_file();
}

}  // namespace blender
