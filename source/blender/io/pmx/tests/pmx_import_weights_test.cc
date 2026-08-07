/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "testing/testing.h"

#include "BKE_attribute.hh"
#include "BKE_gtest_base.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "BLI_math_vector_types.hh"

#include "intern/pmx_types.h"
#include "pmx_import_weights.hh"

namespace blender::io::pmx::tests {
namespace {

static PMXVertex make_vertex(const BoneWeightType weight_type)
{
  PMXVertex vertex{};
  vertex.weight_type = weight_type;
  return vertex;
}

static PMXModel make_weight_model()
{
  PMXModel model;
  model.vertices.push_back(make_vertex(BoneWeightType::BDEF1));
  model.vertices.push_back(make_vertex(BoneWeightType::BDEF2));
  model.vertices.push_back(make_vertex(BoneWeightType::BDEF4));

  PMXVertex sdef = make_vertex(BoneWeightType::SDEF);
  sdef.sdef_c[0] = 1.0f;
  sdef.sdef_c[1] = 2.0f;
  sdef.sdef_c[2] = 3.0f;
  sdef.sdef_r0[0] = 4.0f;
  sdef.sdef_r0[1] = 5.0f;
  sdef.sdef_r0[2] = 6.0f;
  sdef.sdef_r1[0] = 7.0f;
  sdef.sdef_r1[1] = 8.0f;
  sdef.sdef_r1[2] = 9.0f;
  sdef.bone_indices = {0, 1};
  sdef.bone_weights = {0.25f, 0.75f};
  model.vertices.push_back(sdef);

  model.vertices.push_back(make_vertex(BoneWeightType::QDEF));
  return model;
}

class PMXImportWeightsTest : public bke::BlenderGTestBase {};

TEST_F(PMXImportWeightsTest, PreservesWeightTypesAndSdefCoordinates)
{
  PMXModel model = make_weight_model();
  Mesh *mesh = BKE_mesh_new_nomain(int(model.vertices.size()), 0, 0, 0);

  write_pmx_vertex_skinning_attributes(*mesh, model, nullptr, 0.25f);

  const bke::AttributeAccessor attributes = mesh->attributes();
  const bke::AttributeReader<int8_t> weight_types = attributes.lookup<int8_t>(
      "pmx_weight_type", bke::AttrDomain::Point);
  const bke::AttributeReader<float3> sdef_c = attributes.lookup<float3>("pmx_sdef_c",
                                                                          bke::AttrDomain::Point);
  const bke::AttributeReader<float3> sdef_r0 = attributes.lookup<float3>("pmx_sdef_r0",
                                                                           bke::AttrDomain::Point);
  const bke::AttributeReader<float3> sdef_r1 = attributes.lookup<float3>("pmx_sdef_r1",
                                                                           bke::AttrDomain::Point);
  const bke::AttributeReader<int> sdef_bone0 = attributes.lookup<int>("pmx_sdef_bone0",
                                                                        bke::AttrDomain::Point);
  const bke::AttributeReader<int> sdef_bone1 = attributes.lookup<int>("pmx_sdef_bone1",
                                                                        bke::AttrDomain::Point);

  ASSERT_TRUE(weight_types);
  ASSERT_TRUE(sdef_c);
  ASSERT_TRUE(sdef_r0);
  ASSERT_TRUE(sdef_r1);
  ASSERT_TRUE(sdef_bone0);
  ASSERT_TRUE(sdef_bone1);
  ASSERT_EQ(weight_types.varray.size(), 5);

  EXPECT_EQ(weight_types.varray[0], static_cast<int8_t>(BoneWeightType::BDEF1));
  EXPECT_EQ(weight_types.varray[1], static_cast<int8_t>(BoneWeightType::BDEF2));
  EXPECT_EQ(weight_types.varray[2], static_cast<int8_t>(BoneWeightType::BDEF4));
  EXPECT_EQ(weight_types.varray[3], static_cast<int8_t>(BoneWeightType::SDEF));
  EXPECT_EQ(weight_types.varray[4], static_cast<int8_t>(BoneWeightType::QDEF));

  EXPECT_EQ(sdef_c.varray[0], float3(0.0f));
  EXPECT_EQ(sdef_r0.varray[1], float3(0.0f));
  EXPECT_EQ(sdef_r1.varray[2], float3(0.0f));
  EXPECT_EQ(sdef_c.varray[3], float3(0.25f, 0.75f, 0.5f));
  EXPECT_EQ(sdef_r0.varray[3], float3(1.0f, 1.5f, 1.25f));
  EXPECT_EQ(sdef_r1.varray[3], float3(1.75f, 2.25f, 2.0f));
  EXPECT_EQ(sdef_bone0.varray[3], 0);
  EXPECT_EQ(sdef_bone1.varray[3], 1);
  EXPECT_EQ(sdef_bone0.varray[0], -1);
  EXPECT_EQ(sdef_bone1.varray[4], -1);

  BKE_id_free(nullptr, mesh);
}

TEST_F(PMXImportWeightsTest, RemapsSplitMeshSkinningAttributes)
{
  PMXModel model = make_weight_model();
  Mesh *mesh = BKE_mesh_new_nomain(3, 0, 0, 0);
  const Vector<int> new_to_old = {3, 0, 4};

  write_pmx_vertex_skinning_attributes(*mesh, model, &new_to_old, 1.0f);

  const bke::AttributeAccessor attributes = mesh->attributes();
  const bke::AttributeReader<int8_t> weight_types = attributes.lookup<int8_t>(
      "pmx_weight_type", bke::AttrDomain::Point);
  const bke::AttributeReader<float3> sdef_c = attributes.lookup<float3>("pmx_sdef_c",
                                                                          bke::AttrDomain::Point);

  ASSERT_TRUE(weight_types);
  ASSERT_TRUE(sdef_c);
  EXPECT_EQ(weight_types.varray[0], static_cast<int8_t>(BoneWeightType::SDEF));
  EXPECT_EQ(weight_types.varray[1], static_cast<int8_t>(BoneWeightType::BDEF1));
  EXPECT_EQ(weight_types.varray[2], static_cast<int8_t>(BoneWeightType::QDEF));
  EXPECT_EQ(sdef_c.varray[0], float3(1.0f, 3.0f, 2.0f));
  EXPECT_EQ(sdef_c.varray[1], float3(0.0f));
  EXPECT_EQ(sdef_c.varray[2], float3(0.0f));

  BKE_id_free(nullptr, mesh);
}

}  // namespace
}  // namespace blender::io::pmx::tests
