/* SPDX-FileCopyrightText: 2026 SekaiBlender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup pmx_parser
 *
 * Minimal stand-in for the single blenlib symbol the authoritative PMX reader
 * (source/blender/io/pmx/intern/pmx_reader.cc) needs: `blender::BLI_fopen`.
 * This lets the standalone dump tool compile the production reader verbatim
 * instead of carrying a drifted second reader. On Windows the stub opens the
 * file through the wide API so UTF-8 command-line paths work.
 */

#pragma once

#include <cstdio>

#ifdef _WIN32
#  include <string>
#  include <windows.h>

namespace blender {

inline FILE *BLI_fopen(const char *filepath, const char *mode)
{
  /* argv on Windows arrives in the ANSI code page while the formal reader's
   * contract is UTF-8 paths. Decode as strict UTF-8 first (MB_ERR_INVALID_
   * CHARS) and fall back to the ANSI code page for console-provided paths. */
  const UINT codepages[2] = {CP_UTF8, CP_ACP};
  for (const UINT cp : codepages) {
    const UINT flags = (cp == CP_UTF8) ? MB_ERR_INVALID_CHARS : 0;
    const int wlen = MultiByteToWideChar(cp, flags, filepath, -1, nullptr, 0);
    const int mlen = MultiByteToWideChar(cp, flags, mode, -1, nullptr, 0);
    if (wlen <= 0 || mlen <= 0) {
      continue;
    }
    std::wstring wpath(wlen - 1, L'\0');
    MultiByteToWideChar(cp, flags, filepath, -1, &wpath[0], wlen);
    std::wstring wmode(mlen - 1, L'\0');
    MultiByteToWideChar(cp, flags, mode, -1, &wmode[0], mlen);
    FILE *f = _wfopen(wpath.c_str(), wmode.c_str());
    if (f != nullptr) {
      return f;
    }
  }
  return nullptr;
}

}  // namespace blender

#else

namespace blender {
inline FILE *BLI_fopen(const char *filepath, const char *mode)
{
  return std::fopen(filepath, mode);
}
}  // namespace blender

#endif
