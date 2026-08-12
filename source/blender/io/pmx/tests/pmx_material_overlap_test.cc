/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "testing/testing.h"

#include "intern/pmx_types.h"
#include "pmx_import_mesh.hh"

namespace blender::io::pmx::tests {
namespace {

static PMXVertex vertex(const float x, const float y, const float z)
{
  PMXVertex value{};
  value.pos[0] = x;
  value.pos[1] = y;
  value.pos[2] = z;
  return value;
}

static PMXMaterial material()
{
  PMXMaterial value{};
  value.face_vertex_count = 3;
  return value;
}

TEST(PMXMaterialOverlap, MatchesCyclicTriangleRotation)
{
  PMXModel model;
  model.vertices = {vertex(0.0f, 0.0f, 0.0f),
                    vertex(1.0f, 0.0f, 0.0f),
                    vertex(0.0f, 1.0f, 0.0f)};
  model.face_indices = {0, 1, 2, 1, 2, 0};
  model.materials = {material(), material()};

  const Vector<bool> overlaps = find_overlapping_materials(model, 0.08f);

  ASSERT_EQ(overlaps.size(), 2);
  EXPECT_FALSE(overlaps[0]);
  EXPECT_TRUE(overlaps[1]);
}

TEST(PMXMaterialOverlap, MatchesOppositeTriangleWinding)
{
  PMXModel model;
  model.vertices = {vertex(0.0f, 0.0f, 0.0f),
                    vertex(1.0f, 0.0f, 0.0f),
                    vertex(0.0f, 1.0f, 0.0f)};
  model.face_indices = {0, 1, 2, 0, 2, 1};
  model.materials = {material(), material()};

  const Vector<bool> overlaps = find_overlapping_materials(model, 0.08f);

  ASSERT_EQ(overlaps.size(), 2);
  EXPECT_FALSE(overlaps[0]);
  EXPECT_TRUE(overlaps[1]);
}

TEST(PMXMaterialOverlap, RejectsDifferentTriangle)
{
  PMXModel model;
  model.vertices = {vertex(0.0f, 0.0f, 0.0f),
                    vertex(1.0f, 0.0f, 0.0f),
                    vertex(0.0f, 1.0f, 0.0f),
                    vertex(0.0f, 0.0f, 1.0f)};
  model.face_indices = {0, 1, 2, 0, 1, 3};
  model.materials = {material(), material()};

  const Vector<bool> overlaps = find_overlapping_materials(model, 0.08f);

  ASSERT_EQ(overlaps.size(), 2);
  EXPECT_FALSE(overlaps[0]);
  EXPECT_FALSE(overlaps[1]);
}

}  // namespace
}  // namespace blender::io::pmx::tests
