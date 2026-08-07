/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editor/io
 */

#pragma once

#include "WM_types.hh"

namespace blender {

struct wmOperatorType;

void WM_OT_pmx_import(wmOperatorType *ot);
void WM_OT_pmx_export(wmOperatorType *ot);
void WM_OT_pmx_apply_ik(wmOperatorType *ot);
void WM_OT_pmx_apply_append_transform(wmOperatorType *ot);
void WM_OT_pmx_apply_fixed_axis(wmOperatorType *ot);
void WM_OT_pmx_apply_local_axis(wmOperatorType *ot);
void WM_OT_pmx_capture_pose_snapshot(wmOperatorType *ot);

namespace ed::io {
void pmx_file_handler_add();
}

}  // namespace blender
