/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editor/io
 */

#pragma once

struct wmOperatorType;

namespace blender {

void WM_OT_vmd_import(wmOperatorType *ot);
void WM_OT_vmd_camera_import(wmOperatorType *ot);
void WM_OT_vmd_export(wmOperatorType *ot);
void WM_OT_vmd_camera_export(wmOperatorType *ot);

namespace ed::io {
void vmd_file_handler_add();
}

}  // namespace blender
