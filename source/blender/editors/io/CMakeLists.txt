# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Match extern/bullet2: btScalar = double so mmd_physics_world.hh (included
# transitively via io_mmd_physics_ops.cc) sees the same btScalar type as
# extern_bullet.lib and bf_io_pmx.
add_definitions(-DBT_USE_DOUBLE_PRECISION)

set(INC
  ../include
  ../../io/alembic
  ../../io/common
  ../../io/fbx
  ../../io/grease_pencil
  ../../io/ply
  ../../io/stl
  ../../io/pmx
  ../../io/usd
  ../../io/vmd
  ../../io/wavefront_obj
  ../../mmd_ccd_ik
  ../../mmd_physics
  ../../makesrna
  ../../animrig
  ../../../../extern/bullet2/src
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  io_alembic.cc
  io_cache.cc
  io_drop_import_file.cc
  io_fbx_ops.cc
  io_grease_pencil.cc
  io_obj.cc
  io_ops.cc
  io_ply_ops.cc
  io_stl_ops.cc
  io_pmx_ops.cc
  io_usd.cc
  io_utils.cc
  io_vmd_ops.cc

  io_alembic.hh
  io_cache.hh
  io_drop_import_file.hh
  io_fbx_ops.hh
  io_grease_pencil.hh
  io_obj.hh
  io_ops.hh
  io_ply_ops.hh
  io_stl_ops.hh
  io_pmx_ops.hh
  io_usd.hh
  io_utils.hh
  io_vmd_ops.hh
)

set(LIB
  PRIVATE bf::animrig
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::blentranslation
  PRIVATE bf::bmesh
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::intern::clog
  PRIVATE bf::windowmanager
  PRIVATE bf_io_vmd
)

if(WITH_IO_WAVEFRONT_OBJ)
  list(APPEND LIB
    bf_io_wavefront_obj
  )
  add_definitions(-DWITH_IO_WAVEFRONT_OBJ)
endif()

if(WITH_IO_PLY)
  list(APPEND LIB
    bf_io_ply
  )
  add_definitions(-DWITH_IO_PLY)
endif()

if(WITH_IO_STL)
  list(APPEND LIB
    bf_io_stl
  )
  add_definitions(-DWITH_IO_STL)
endif()

if(WITH_IO_PMX)
  list(APPEND SRC
    io_mmd_physics_ops.cc
    io_mmd_render_ops.cc
    io_mmd_physics_ops.hh
    io_mmd_render_ops.hh
  )
  list(APPEND LIB
    bf_io_pmx
  )
  add_definitions(-DWITH_IO_PMX)
endif()

if(WITH_IO_FBX)
  list(APPEND LIB
    bf_io_fbx
  )
  add_definitions(-DWITH_IO_FBX)
endif()

if(WITH_IO_GREASE_PENCIL)
  list(APPEND LIB
    bf_io_grease_pencil
  )
  add_definitions(-DWITH_IO_GREASE_PENCIL)
endif()

if(WITH_ALEMBIC)
  list(APPEND LIB
    bf_io_alembic
  )
  add_definitions(-DWITH_ALEMBIC)
endif()

if(WITH_USD)
  list(APPEND LIB
    bf_io_usd
    PRIVATE bf::dependencies::optional::usd
  )
  add_definitions(-DWITH_USD)
endif()

if(WITH_PUGIXML)
  add_definitions(-DWITH_PUGIXML)
endif()

if(WITH_HARU)
  add_definitions(-DWITH_HARU)
endif()

blender_add_lib(bf_editor_io "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# MSVC defaults to the system code page for non-ASCII string literals.
# /utf-8 keeps diagnostic names aligned with Blender runtime strings.
if(MSVC)
  target_compile_options(bf_editor_io PRIVATE /utf-8)
endif()
