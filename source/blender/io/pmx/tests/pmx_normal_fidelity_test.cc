/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 *
 * Measures how much precision a per-vertex normal loses on a Blender round-trip.
 *
 * PMX stores one normal per vertex. The importer widens that to per-corner and
 * hands it to `bke::mesh_set_custom_normals`, which does not keep float3: it
 * projects each normal into the corner's fan space and stores a compressed
 * `short2`. Reading `Mesh::corner_normals()` back decodes that.
 *
 * So a PMX export cannot reproduce the source normal bit-exactly even when
 * nothing was edited, and `PMXModelDiffOptions::normal_tolerance` has to be
 * loose enough to allow the encoding loss. Picking that number by guesswork
 * would either produce thousands of false differences (too tight) or hide a
 * genuine inversion error (too loose), so it is measured here instead.
 *
 * These tests exist to keep that constant honest: if Blender's encoding ever
 * changes, the measured bound moves and this fails.
 */

#include "testing/testing.h"

#include "BKE_gtest_base.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"

#include "DNA_mesh_types.h"

#include "intern/pmx_model_diff.h"

#include <algorithm>
#include <cmath>

namespace blender::io::pmx::tests {
namespace {

/** Deterministic near-uniform directions on the unit sphere (Fibonacci spiral). */
Vector<float3> sample_directions(const int count)
{
  Vector<float3> directions;
  directions.reserve(count);
  const float golden_angle = 2.399963229728653f; /* pi * (3 - sqrt(5)) */
  for (const int i : IndexRange(count)) {
    const float z = 1.0f - 2.0f * (float(i) + 0.5f) / float(count);
    const float radius = std::sqrt(std::max(0.0f, 1.0f - z * z));
    const float theta = golden_angle * float(i);
    directions.append(float3(radius * std::cos(theta), radius * std::sin(theta), z));
  }
  return directions;
}

struct NormalError {
  float max_component = 0.0f;
  float max_angle_rad = 0.0f;
};

void accumulate(NormalError &error, const float3 &expected, const float3 &actual)
{
  for (const int axis : IndexRange(3)) {
    error.max_component = std::max(error.max_component,
                                   std::abs(expected[axis] - actual[axis]));
  }
  const float dot = std::clamp(math::dot(math::normalize(expected), math::normalize(actual)),
                               -1.0f,
                               1.0f);
  error.max_angle_rad = std::max(error.max_angle_rad, std::acos(dot));
}

/**
 * One independent triangle per direction, with no shared vertices.
 *
 * Isolates the `short2` quantization: every corner fan holds a single corner, so
 * no fan-space averaging can contribute.
 */
Mesh *make_isolated_triangles(const Span<float3> directions)
{
  const int triangle_num = int(directions.size());
  Mesh *mesh = BKE_mesh_new_nomain(triangle_num * 3, 0, triangle_num, triangle_num * 3);

  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
  for (const int triangle : IndexRange(triangle_num)) {
    /* Spread the triangles apart so none of them share a position. */
    const float3 origin = float3(float(triangle) * 4.0f, 0.0f, 0.0f);
    positions[triangle * 3 + 0] = origin;
    positions[triangle * 3 + 1] = origin + float3(1.0f, 0.0f, 0.0f);
    positions[triangle * 3 + 2] = origin + float3(0.0f, 1.0f, 0.0f);
    for (const int corner : IndexRange(3)) {
      corner_verts[triangle * 3 + corner] = triangle * 3 + corner;
    }
  }
  offset_indices::fill_constant_group_size(3, 0, mesh->face_offsets_for_write());
  bke::mesh_calc_edges(*mesh, false, false);
  bke::mesh_smooth_set(*mesh, true);
  return mesh;
}

/**
 * A triangle fan around one shared centre vertex.
 *
 * Every corner at the centre belongs to the same fan, which is the arrangement
 * that makes fan-space projection contribute on top of quantization.
 */
Mesh *make_shared_vertex_fan(const int segments)
{
  const int vert_num = segments + 1;
  Mesh *mesh = BKE_mesh_new_nomain(vert_num, 0, segments, segments * 3);

  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  positions[0] = float3(0.0f);
  for (const int segment : IndexRange(segments)) {
    const float angle = 2.0f * float(M_PI) * float(segment) / float(segments);
    positions[segment + 1] = float3(std::cos(angle), std::sin(angle), 0.0f);
  }

  MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
  for (const int segment : IndexRange(segments)) {
    corner_verts[segment * 3 + 0] = 0;
    corner_verts[segment * 3 + 1] = segment + 1;
    corner_verts[segment * 3 + 2] = (segment + 1) % segments + 1;
  }
  offset_indices::fill_constant_group_size(3, 0, mesh->face_offsets_for_write());
  bke::mesh_calc_edges(*mesh, false, false);
  bke::mesh_smooth_set(*mesh, true);
  return mesh;
}

/** Set `normals` as custom corner normals, read them back, and measure the loss. */
NormalError measure_round_trip(Mesh &mesh, const Span<float3> normals)
{
  Array<float3> to_write(normals.size());
  to_write.as_mutable_span().copy_from(normals);
  bke::mesh_set_custom_normals(mesh, to_write);

  const Span<float3> read_back = mesh.corner_normals();
  NormalError error;
  for (const int corner : normals.index_range()) {
    accumulate(error, normals[corner], read_back[corner]);
  }
  return error;
}

class PMXNormalFidelityTest : public bke::BlenderGTestBase {};

TEST_F(PMXNormalFidelityTest, quantization_bound_justifies_normal_tolerance)
{
  const Vector<float3> directions = sample_directions(64);
  Mesh *mesh = make_isolated_triangles(directions);

  /* Every corner of a triangle carries that triangle's sampled direction, which
   * is what PMX import does: one PMX vertex normal, written to each corner. */
  Vector<float3> normals;
  normals.reserve(directions.size() * 3);
  for (const float3 &direction : directions) {
    normals.append(direction);
    normals.append(direction);
    normals.append(direction);
  }

  const NormalError error = measure_round_trip(*mesh, normals);
  std::cout << "  isolated triangles: max component error " << error.max_component
            << ", max angle error " << error.max_angle_rad << " rad\n";

  /* The loss is real, so a zero tolerance would be wrong. */
  EXPECT_GT(error.max_component, 0.0f);
  /* Independent upper guard, deliberately not equal to `normal_tolerance`:
   * this one fails if Blender's normal encoding ever gets meaningfully
   * coarser, which would mean the tolerance below needs re-deriving rather
   * than silently absorbing the change. */
  EXPECT_LT(error.max_component, 2.0e-2f);

  /* The whole point: the configured tolerance must actually cover the loss. */
  const PMXModelDiffOptions options;
  EXPECT_LT(error.max_component, options.normal_tolerance)
      << "normal_tolerance is too tight for Blender's normal encoding";

  BKE_id_free(nullptr, mesh);
}

TEST_F(PMXNormalFidelityTest, shared_vertex_fan_stays_within_normal_tolerance)
{
  const int segments = 32;
  Mesh *mesh = make_shared_vertex_fan(segments);

  /* One direction per ring vertex, reused at the centre corner, so the centre
   * fan sees many different normals -- the worst case for fan-space projection. */
  const Vector<float3> directions = sample_directions(segments);
  Vector<float3> normals;
  normals.reserve(segments * 3);
  for (const int segment : IndexRange(segments)) {
    normals.append(directions[segment]);
    normals.append(directions[segment]);
    normals.append(directions[(segment + 1) % segments]);
  }

  const NormalError error = measure_round_trip(*mesh, normals);
  std::cout << "  shared-vertex fan: max component error " << error.max_component
            << ", max angle error " << error.max_angle_rad << " rad\n";

  const PMXModelDiffOptions options;
  EXPECT_LT(error.max_component, options.normal_tolerance)
      << "normal_tolerance is too tight once fan-space projection is involved";

  BKE_id_free(nullptr, mesh);
}

}  // namespace
}  // namespace blender::io::pmx::tests
