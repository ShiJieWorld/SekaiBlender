# SPDX-FileCopyrightText: 2026 MuAnChen
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Match extern/bullet2: btScalar = double so symbol mangling aligns with
# extern_bullet.lib (otherwise LNK2001 on btCollisionShape / btCapsuleShapeZ
# / btGeneric6DofSpringConstraint methods).
add_definitions(-DBT_USE_DOUBLE_PRECISION)

set(INC
  .
  exporter
  importer
  intern
  ../../mmd_physics
  ../../mmd_ccd_ik
  ../../editors/include
  ../../makesrna
  ../common
  ../../../../extern/bullet2/src
)

set(INC_SYS
)

set(SRC
  ../../mmd_physics/mmd_physics_definition.cc
  ../../mmd_physics/mmd_physics_diagnostics.cc
  ../../mmd_physics/mmd_physics_world.cc

  ../../mmd_physics/mmd_physics_definition.hh
  ../../mmd_physics/mmd_physics_diagnostics.hh
  ../../mmd_physics/mmd_physics_world.hh

  ../../mmd_ccd_ik/mmd_ccd_ik.hh

  IO_pmx.cc
  exporter/pmx_export.cc
  importer/pmx_import.cc
  importer/pmx_import_armature.cc
  importer/pmx_import_bone_ik.cc
  importer/pmx_import_bone_append.cc
  importer/pmx_import_bone_axis.cc
  importer/pmx_import_pose_snapshot.cc
  importer/pmx_import_material.cc
  importer/pmx_import_mesh.cc
  importer/pmx_import_morph.cc
  importer/pmx_import_morph_controller.cc
  importer/pmx_group_morph.cc
  importer/pmx_import_weights.cc
  importer/pmx_source_data.cc
  intern/pmx_model_diff.cc
  intern/pmx_reader.cc
  intern/pmx_writer.cc

  IO_pmx.hh
  exporter/pmx_export.hh
  importer/pmx_import.hh
  importer/pmx_import_armature.hh
  importer/pmx_import_bone_ik.hh
  importer/pmx_import_bone_append.hh
  importer/pmx_import_bone_axis.hh
  importer/pmx_import_pose_snapshot.hh
  importer/pmx_import_material.hh
  importer/pmx_import_mesh.hh
  importer/pmx_import_morph.hh
  importer/pmx_import_morph_controller.hh
  importer/pmx_group_morph.hh
  importer/pmx_import_weights.hh
  importer/pmx_source_data.hh
  intern/pmx_model_diff.h
  intern/pmx_reader.h
  intern/pmx_types.h
  intern/pmx_writer.h
)

set(LIB
  PRIVATE bf::animrig
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::bmesh
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::imbuf
  bf_editor_animation
  bf_editor_armature
  PRIVATE bf::intern::clog
  PRIVATE bf::intern::guardedalloc
  bf_io_common
  PRIVATE bf::nodes
  PRIVATE bf::windowmanager
  extern_bullet
)

blender_add_lib(bf_io_pmx "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# MSVC defaults to the system code page for non-ASCII string literals.
# /utf-8 keeps diagnostic names aligned with Blender runtime strings.
if(MSVC)
  target_compile_options(bf_io_pmx PRIVATE /utf-8)
endif()

if(WITH_GTESTS)
  set(TEST_SRC
    tests/pmx_group_morph_test.cc
    tests/pmx_bone_ik_test.cc
    tests/pmx_bone_append_test.cc
    tests/pmx_bone_axis_test.cc
    tests/pmx_pose_snapshot_test.cc
    tests/pmx_import_weights_test.cc
    tests/pmx_material_overlap_test.cc
    tests/pmx_model_diff_test.cc
    tests/pmx_normal_fidelity_test.cc
    tests/pmx_round_trip_test.cc
    tests/pmx_source_data_test.cc
    tests/pmx_writer_test.cc
    tests/mmd_ccd_ik_test.cc
  )
  set(TEST_INC
    ../../../../tests/gtests
  )
  set(TEST_LIB
    bf_io_pmx
  )
  blender_add_test_suite_lib(
    io_pmx
    "${TEST_SRC}"
    "${INC};${TEST_INC}"
    "${INC_SYS}"
    "${LIB};${TEST_LIB}"
  )
endif()
