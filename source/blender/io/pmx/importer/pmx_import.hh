/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include "IO_pmx.hh"

namespace blender::io::pmx {

void importer_main(bContext *C, PMXImportParams &params);

}  // namespace blender::io::pmx
