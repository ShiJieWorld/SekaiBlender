# SPDX-FileCopyrightText: 2026 MuAnChen
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  .
  ../../animrig
  ../../blenkernel
  ../../blenlib
  ../../depsgraph
  ../../makesdna
  ../../makesrna
)

set(INC_SYS
)

set(SRC
  IO_vmd.cc
  vmd_action.cc
  vmd_camera_action.cc
  vmd_mapping.cc
  vmd_morph_mapping.cc
  vmd_morph_action.cc
  vmd_import.cc
  vmd_export.cc

  IO_vmd.hh
  vmd_action.hh
  vmd_camera_action.hh
  vmd_mapping.hh
  vmd_morph_mapping.hh
  vmd_morph_action.hh
  vmd_import.hh
  vmd_export.hh
)

set(LIB
  PRIVATE bf::animrig
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::intern::guardedalloc
)

blender_add_lib(bf_io_vmd "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

if(WITH_GTESTS)
  set(TEST_SRC
    tests/vmd_action_test.cc
    tests/vmd_camera_action_test.cc
    tests/vmd_morph_mapping_test.cc
    tests/vmd_morph_action_test.cc
    tests/vmd_import_morph_test.cc
  )
  set(TEST_INC
    ../../../../tests/gtests
  )
  set(TEST_LIB
    bf_io_vmd
  )
  blender_add_test_suite_lib(
    io_vmd
    "${TEST_SRC}"
    "${INC};${TEST_INC}"
    "${INC_SYS}"
    "${LIB};${TEST_LIB}"
  )
endif()
