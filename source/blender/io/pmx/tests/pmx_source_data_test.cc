/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "testing/testing.h"

#include "BKE_attribute.hh"
#include "BKE_collection.hh"
#include "BKE_gtest_base.hh"
#include "BKE_idprop.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_material.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"

#include "BLI_math_vector_types.hh"

#include "DNA_collection_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"

#include "intern/pmx_types.h"
#include "pmx_import_mesh.hh"
#include "pmx_source_data.hh"

#include "MEM_guardedalloc.h"

#include <cstdio>
#include <string>

namespace blender::io::pmx::tests {
namespace {

/* UTF-8 spelled out so the test does not depend on the execution charset. */
constexpr const char *kModelNameLocal = "\xE3\x83\x86\xE3\x82\xB9\xE3\x83\x88"; /* test */

/**
 * A model that exercises every retained section and every Morph kind, so the
 * round-trip test fails if any field is dropped from either direction.
 */
PMXModel make_source_model()
{
  PMXModel model;

  model.header.version = 2.1f;
  model.header.encoding = uint8_t(PMXEncoding::UTF16LE);
  model.header.add_uv_cnt = 2;
  model.name_local = kModelNameLocal;
  model.name_universal = "Source Model";
  model.comment_local = "comment local";
  model.comment_universal = "comment universal";

  /* --- Vertices: additional UV must survive as mesh attributes. --- */
  for (const int i : IndexRange(3)) {
    PMXVertex vertex{};
    vertex.additional_uv_count = 2;
    vertex.pos[0] = float(i);
    vertex.weight_type = BoneWeightType::BDEF1;
    vertex.bone_indices = {0};
    vertex.bone_weights = {1.0f};
    vertex.edge_factor = 1.0f;
    vertex.additional_uv[0] = {float(i) + 0.1f, float(i) + 0.2f, float(i) + 0.3f, float(i) + 0.4f};
    vertex.additional_uv[1] = {float(i) + 0.5f, float(i) + 0.6f, float(i) + 0.7f, float(i) + 0.8f};
    model.vertices.push_back(vertex);
  }
  model.face_indices = {0, 1, 2};

  /* --- Textures --- */
  model.textures.push_back({"tex\\base.png"});
  model.textures.push_back({"tex\\toon.png"});

  /* --- Materials: one internal toon, one external toon. --- */
  PMXMaterial mat0{};
  mat0.name_local = "Mat0";
  mat0.name_universal = "Mat0 EN";
  mat0.diffuse[0] = 0.1f;
  mat0.diffuse[1] = 0.2f;
  mat0.diffuse[2] = 0.3f;
  mat0.diffuse[3] = 1.0f;
  mat0.specular[0] = 0.4f;
  mat0.specular[1] = 0.5f;
  mat0.specular[2] = 0.6f;
  mat0.specular_power = 12.5f;
  mat0.ambient[0] = 0.7f;
  mat0.ambient[1] = 0.8f;
  mat0.ambient[2] = 0.9f;
  mat0.flag = PMX_MATERIAL_FLAG_DOUBLE_SIDED | PMX_MATERIAL_FLAG_EDGE;
  mat0.edge_color[0] = 0.05f;
  mat0.edge_color[3] = 1.0f;
  mat0.edge_size = 1.25f;
  mat0.texture_idx = 0;
  mat0.sphere_texture_idx = 1;
  mat0.sphere_mode = SphereMode::Sphere;
  mat0.toon_flag = 0;
  mat0.toon_internal_value = 3;
  mat0.memo = "memo 0";
  mat0.face_vertex_count = 3;
  model.materials.push_back(mat0);

  PMXMaterial mat1{};
  mat1.name_local = "Mat1";
  mat1.diffuse[3] = 1.0f;
  mat1.flag = PMX_MATERIAL_FLAG_SELF_SHADOW;
  mat1.edge_size = 0.5f;
  mat1.texture_idx = -1;
  mat1.sphere_texture_idx = -1;
  mat1.sphere_mode = SphereMode::None;
  mat1.toon_flag = 1;
  mat1.toon_texture_idx = 1;
  mat1.face_vertex_count = 0;
  model.materials.push_back(mat1);

  /* --- Bones: plain, tail-offset + fixed/local axis + append, and IK. --- */
  PMXBone bone0{};
  bone0.name_local = "Root";
  bone0.name_universal = "Root EN";
  bone0.parent_index = -1;
  bone0.transform_order = 0;
  bone0.flag = BONE_FLAG_ROTATABLE | BONE_FLAG_VISIBLE | BONE_FLAG_TAIL_POS;
  bone0.tail_pos_bone = 1;
  model.bones.push_back(bone0);

  PMXBone bone1{};
  bone1.name_local = "Twist";
  bone1.parent_index = 0;
  bone1.transform_order = 7;
  bone1.flag = BONE_FLAG_APPEND_ROTATION | BONE_FLAG_FIXED_AXIS | BONE_FLAG_LOCAL_AXIS |
               BONE_FLAG_PHYSICS_AFTER_DEF | BONE_FLAG_EXTERNAL_PARENT;
  bone1.tail_pos_bone = -2;
  bone1.tail_pos_offset[0] = 0.5f;
  bone1.tail_pos_offset[1] = 1.5f;
  bone1.tail_pos_offset[2] = 2.5f;
  bone1.inherit_parent_index = 0;
  bone1.inherit_parent_ratio = -0.75f;
  bone1.fixed_axis[0] = 1.0f;
  bone1.local_x[0] = 1.0f;
  bone1.local_z[2] = 1.0f;
  bone1.external_parent_index = 42;
  model.bones.push_back(bone1);

  PMXBone bone2{};
  bone2.name_local = "IK";
  bone2.parent_index = 0;
  bone2.flag = BONE_FLAG_IK;
  bone2.tail_pos_bone = -2;
  bone2.ik_target_index = 1;
  bone2.ik_loop_count = 40;
  bone2.ik_angle_limit = 0.5f;
  PMXIKLink link0{};
  link0.bone_index = 1;
  link0.limit_angle = true;
  link0.limit_min[0] = -1.0f;
  link0.limit_max[0] = 1.0f;
  bone2.ik_links.push_back(link0);
  PMXIKLink link1{};
  link1.bone_index = 0;
  link1.limit_angle = false;
  bone2.ik_links.push_back(link1);
  model.bones.push_back(bone2);

  /* --- Morphs: index 0 is Vertex so Group/Flip can reference it. --- */
  PMXMorph vertex_morph{};
  vertex_morph.name_local = "VertexMorph";
  vertex_morph.name_universal = "VertexMorph EN";
  vertex_morph.panel = 1;
  vertex_morph.type = MorphType::Vertex;
  vertex_morph.vertex_offsets.push_back({0, {1.0f, 2.0f, 3.0f}});
  vertex_morph.vertex_offsets.push_back({2, {4.0f, 5.0f, 6.0f}});
  model.morphs.push_back(vertex_morph);

  PMXMorph group_morph{};
  group_morph.name_local = "GroupMorph";
  group_morph.panel = 2;
  group_morph.type = MorphType::Group;
  group_morph.group_offsets.push_back({0, 0.5f});
  model.morphs.push_back(group_morph);

  PMXMorph bone_morph{};
  bone_morph.name_local = "BoneMorph";
  bone_morph.panel = 3;
  bone_morph.type = MorphType::Bone;
  bone_morph.bone_offsets.push_back({1, {0.1f, 0.2f, 0.3f}, {0.4f, 0.5f, 0.6f, 0.7f}});
  model.morphs.push_back(bone_morph);

  PMXMorph uv_morph{};
  uv_morph.name_local = "UVMorph";
  uv_morph.panel = 4;
  uv_morph.type = MorphType::UV_2nd;
  uv_morph.uv_offsets.push_back({1, {0.11f, 0.22f, 0.33f, 0.44f}});
  model.morphs.push_back(uv_morph);

  PMXMorph material_morph{};
  material_morph.name_local = "MaterialMorph";
  material_morph.panel = 4;
  material_morph.type = MorphType::Material;
  PMXMaterialMorphOffset material_offset{};
  material_offset.material_index = -1;
  material_offset.calc_mode = 1;
  material_offset.diffuse[0] = 0.9f;
  material_offset.specular[1] = 0.8f;
  material_offset.specular_power = 4.5f;
  material_offset.ambient[2] = 0.7f;
  material_offset.edge_color[3] = 0.6f;
  material_offset.edge_size = 2.5f;
  material_offset.texture_factor[0] = 0.5f;
  material_offset.sphere_texture_factor[1] = 0.4f;
  material_offset.toon_texture_factor[2] = 0.3f;
  material_morph.material_offsets.push_back(material_offset);
  model.morphs.push_back(material_morph);

  PMXMorph flip_morph{};
  flip_morph.name_local = "FlipMorph";
  flip_morph.panel = 4;
  flip_morph.type = MorphType::Flip;
  flip_morph.group_offsets.push_back({0, 1.0f});
  model.morphs.push_back(flip_morph);

  PMXMorph impulse_morph{};
  impulse_morph.name_local = "ImpulseMorph";
  impulse_morph.panel = 4;
  impulse_morph.type = MorphType::Impulse;
  PMXImpulseMorphOffset impulse_offset{};
  impulse_offset.rigid_index = 0;
  impulse_offset.local_flag = 1;
  impulse_offset.velocity[0] = 1.5f;
  impulse_offset.torque[2] = 2.5f;
  impulse_morph.impulse_offsets.push_back(impulse_offset);
  model.morphs.push_back(impulse_morph);

  /* --- Display frames --- */
  PMXDisplayFrame bone_frame{};
  bone_frame.name_local = "BoneFrame";
  bone_frame.name_universal = "BoneFrame EN";
  bone_frame.flag = 0;
  bone_frame.items.push_back({0, 0});
  bone_frame.items.push_back({0, 2});
  model.display_frames.push_back(bone_frame);

  PMXDisplayFrame morph_frame{};
  morph_frame.name_local = "MorphFrame";
  morph_frame.flag = 1;
  morph_frame.items.push_back({1, 1});
  model.display_frames.push_back(morph_frame);

  /* --- Physics counts only; bodies live in mmd_physics_definition. --- */
  PMXRigidBody rigid{};
  rigid.name_local = "Rigid";
  rigid.bone_index = 0;
  model.rigid_bodies.push_back(rigid);

  return model;
}

PMXImportParams make_params()
{
  PMXImportParams params;
  std::snprintf(params.filepath, sizeof(params.filepath), "%s", "D:\\models\\source.pmx");
  params.global_scale = 0.08f;
  params.split_by_material = true;
  return params;
}

void expect_source_data_equal(const PMXSourceData &a, const PMXSourceData &b)
{
  EXPECT_EQ(a.schema_version, b.schema_version);
  EXPECT_FLOAT_EQ(a.pmx_version, b.pmx_version);
  EXPECT_EQ(a.encoding, b.encoding);
  EXPECT_EQ(a.additional_uv_count, b.additional_uv_count);

  EXPECT_EQ(a.name_local, b.name_local);
  EXPECT_EQ(a.name_universal, b.name_universal);
  EXPECT_EQ(a.comment_local, b.comment_local);
  EXPECT_EQ(a.comment_universal, b.comment_universal);

  EXPECT_EQ(a.source_filepath, b.source_filepath);
  EXPECT_FLOAT_EQ(a.global_scale, b.global_scale);
  EXPECT_EQ(a.split_by_material, b.split_by_material);

  EXPECT_EQ(a.vertex_count, b.vertex_count);
  EXPECT_EQ(a.face_index_count, b.face_index_count);
  EXPECT_EQ(a.rigid_body_count, b.rigid_body_count);
  EXPECT_EQ(a.joint_count, b.joint_count);

  ASSERT_EQ(a.textures.size(), b.textures.size());
  for (const int i : IndexRange(a.textures.size())) {
    EXPECT_EQ(a.textures[i].path, b.textures[i].path);
  }

  ASSERT_EQ(a.materials.size(), b.materials.size());
  for (const int i : IndexRange(a.materials.size())) {
    const PMXSourceMaterial &x = a.materials[i];
    const PMXSourceMaterial &y = b.materials[i];
    EXPECT_EQ(x.pmx_index, y.pmx_index);
    EXPECT_EQ(x.name_local, y.name_local);
    EXPECT_EQ(x.name_universal, y.name_universal);
    EXPECT_EQ(x.diffuse, y.diffuse);
    EXPECT_EQ(x.specular, y.specular);
    EXPECT_FLOAT_EQ(x.specular_power, y.specular_power);
    EXPECT_EQ(x.ambient, y.ambient);
    EXPECT_EQ(x.flag, y.flag);
    EXPECT_EQ(x.edge_color, y.edge_color);
    EXPECT_FLOAT_EQ(x.edge_size, y.edge_size);
    EXPECT_EQ(x.texture_index, y.texture_index);
    EXPECT_EQ(x.sphere_texture_index, y.sphere_texture_index);
    EXPECT_EQ(x.sphere_mode, y.sphere_mode);
    EXPECT_EQ(x.toon_flag, y.toon_flag);
    EXPECT_EQ(x.toon_texture_index, y.toon_texture_index);
    EXPECT_EQ(x.toon_internal_value, y.toon_internal_value);
    EXPECT_EQ(x.memo, y.memo);
    EXPECT_EQ(x.face_vertex_count, y.face_vertex_count);
    EXPECT_EQ(x.blender_material_name, y.blender_material_name);
  }

  ASSERT_EQ(a.bones.size(), b.bones.size());
  for (const int i : IndexRange(a.bones.size())) {
    const PMXSourceBone &x = a.bones[i];
    const PMXSourceBone &y = b.bones[i];
    EXPECT_EQ(x.pmx_index, y.pmx_index);
    EXPECT_EQ(x.name_local, y.name_local);
    EXPECT_EQ(x.name_universal, y.name_universal);
    EXPECT_EQ(x.parent_index, y.parent_index);
    EXPECT_EQ(x.transform_order, y.transform_order);
    EXPECT_EQ(x.flag, y.flag);
    EXPECT_EQ(x.tail_pos_bone, y.tail_pos_bone);
    EXPECT_EQ(x.tail_pos_offset, y.tail_pos_offset);
    EXPECT_EQ(x.inherit_parent_index, y.inherit_parent_index);
    EXPECT_FLOAT_EQ(x.inherit_parent_ratio, y.inherit_parent_ratio);
    EXPECT_EQ(x.fixed_axis, y.fixed_axis);
    EXPECT_EQ(x.local_x, y.local_x);
    EXPECT_EQ(x.local_z, y.local_z);
    EXPECT_EQ(x.ik_target_index, y.ik_target_index);
    EXPECT_EQ(x.ik_loop_count, y.ik_loop_count);
    EXPECT_FLOAT_EQ(x.ik_angle_limit, y.ik_angle_limit);
    EXPECT_EQ(x.external_parent_index, y.external_parent_index);
    EXPECT_EQ(x.blender_bone_name, y.blender_bone_name);
    ASSERT_EQ(x.ik_links.size(), y.ik_links.size());
    for (const int j : IndexRange(x.ik_links.size())) {
      EXPECT_EQ(x.ik_links[j].bone_index, y.ik_links[j].bone_index);
      EXPECT_EQ(x.ik_links[j].limit_angle, y.ik_links[j].limit_angle);
      EXPECT_EQ(x.ik_links[j].limit_min, y.ik_links[j].limit_min);
      EXPECT_EQ(x.ik_links[j].limit_max, y.ik_links[j].limit_max);
    }
  }

  ASSERT_EQ(a.morphs.size(), b.morphs.size());
  for (const int i : IndexRange(a.morphs.size())) {
    const PMXSourceMorph &x = a.morphs[i];
    const PMXSourceMorph &y = b.morphs[i];
    EXPECT_EQ(x.pmx_index, y.pmx_index);
    EXPECT_EQ(x.name_local, y.name_local);
    EXPECT_EQ(x.name_universal, y.name_universal);
    EXPECT_EQ(x.panel, y.panel);
    EXPECT_EQ(x.type, y.type);
    EXPECT_EQ(x.vertex_offset_count, y.vertex_offset_count);
    EXPECT_EQ(x.blender_shape_key_name, y.blender_shape_key_name);

    ASSERT_EQ(x.group_offsets.size(), y.group_offsets.size());
    for (const int j : IndexRange(x.group_offsets.size())) {
      EXPECT_EQ(x.group_offsets[j].morph_index, y.group_offsets[j].morph_index);
      EXPECT_FLOAT_EQ(x.group_offsets[j].influence, y.group_offsets[j].influence);
    }
    ASSERT_EQ(x.bone_offsets.size(), y.bone_offsets.size());
    for (const int j : IndexRange(x.bone_offsets.size())) {
      EXPECT_EQ(x.bone_offsets[j].bone_index, y.bone_offsets[j].bone_index);
      EXPECT_EQ(x.bone_offsets[j].pos, y.bone_offsets[j].pos);
      EXPECT_EQ(x.bone_offsets[j].rot, y.bone_offsets[j].rot);
    }
    ASSERT_EQ(x.uv_offsets.size(), y.uv_offsets.size());
    for (const int j : IndexRange(x.uv_offsets.size())) {
      EXPECT_EQ(x.uv_offsets[j].vertex_index, y.uv_offsets[j].vertex_index);
      EXPECT_EQ(x.uv_offsets[j].offset, y.uv_offsets[j].offset);
    }
    ASSERT_EQ(x.material_offsets.size(), y.material_offsets.size());
    for (const int j : IndexRange(x.material_offsets.size())) {
      const PMXSourceMaterialMorphOffset &p = x.material_offsets[j];
      const PMXSourceMaterialMorphOffset &q = y.material_offsets[j];
      EXPECT_EQ(p.material_index, q.material_index);
      EXPECT_EQ(p.calc_mode, q.calc_mode);
      EXPECT_EQ(p.diffuse, q.diffuse);
      EXPECT_EQ(p.specular, q.specular);
      EXPECT_FLOAT_EQ(p.specular_power, q.specular_power);
      EXPECT_EQ(p.ambient, q.ambient);
      EXPECT_EQ(p.edge_color, q.edge_color);
      EXPECT_FLOAT_EQ(p.edge_size, q.edge_size);
      EXPECT_EQ(p.texture_factor, q.texture_factor);
      EXPECT_EQ(p.sphere_texture_factor, q.sphere_texture_factor);
      EXPECT_EQ(p.toon_texture_factor, q.toon_texture_factor);
    }
    ASSERT_EQ(x.impulse_offsets.size(), y.impulse_offsets.size());
    for (const int j : IndexRange(x.impulse_offsets.size())) {
      EXPECT_EQ(x.impulse_offsets[j].rigid_index, y.impulse_offsets[j].rigid_index);
      EXPECT_EQ(x.impulse_offsets[j].local_flag, y.impulse_offsets[j].local_flag);
      EXPECT_EQ(x.impulse_offsets[j].velocity, y.impulse_offsets[j].velocity);
      EXPECT_EQ(x.impulse_offsets[j].torque, y.impulse_offsets[j].torque);
    }
  }

  ASSERT_EQ(a.display_frames.size(), b.display_frames.size());
  for (const int i : IndexRange(a.display_frames.size())) {
    const PMXSourceDisplayFrame &x = a.display_frames[i];
    const PMXSourceDisplayFrame &y = b.display_frames[i];
    EXPECT_EQ(x.name_local, y.name_local);
    EXPECT_EQ(x.name_universal, y.name_universal);
    EXPECT_EQ(x.flag, y.flag);
    ASSERT_EQ(x.items.size(), y.items.size());
    for (const int j : IndexRange(x.items.size())) {
      EXPECT_EQ(x.items[j].type, y.items[j].type);
      EXPECT_EQ(x.items[j].index, y.items[j].index);
    }
  }
}

class PMXSourceDataTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Collection *model_collection = nullptr;
  ReportList reports;

  void SetUp() override
  {
    bmain = BKE_main_new();
    model_collection = BKE_collection_add(bmain, nullptr, "SourceModel");
    BKE_reports_init(&reports, RPT_STORE);
  }

  void TearDown() override
  {
    BKE_reports_free(&reports);
    BKE_main_free(bmain);
  }

  Object *add_mesh_object(const char *name, const int vert_num)
  {
    Mesh *nomain = BKE_mesh_new_nomain(vert_num, 0, 0, 0);
    Mesh *mesh = BKE_mesh_add(bmain, name);
    BKE_mesh_nomain_to_mesh(nomain, mesh, nullptr);
    Object *obj = BKE_object_add_only_object(bmain, OB_MESH, name);
    obj->data = &mesh->id;
    return obj;
  }

  IDProperty *source_root()
  {
    IDProperty *system = model_collection->id.system_properties;
    return system ? IDP_GetPropertyTypeFromGroup(system, "mmd_pmx_source_data", IDP_GROUP) :
                    nullptr;
  }
};

TEST_F(PMXSourceDataTest, round_trips_every_retained_section)
{
  const PMXModel model = make_source_model();
  const PMXImportParams params = make_params();
  PMXImportContext ctx{};

  const PMXSourceData built = build_pmx_source_data(model, params, {}, {}, {}, ctx);
  ASSERT_TRUE(serialize_pmx_source_data(*model_collection, built, &reports));

  PMXSourceData read;
  ASSERT_TRUE(deserialize_pmx_source_data(*model_collection, read, &reports));
  expect_source_data_equal(built, read);
}

TEST_F(PMXSourceDataTest, retains_header_and_model_info)
{
  const PMXModel model = make_source_model();
  const PMXImportParams params = make_params();
  PMXImportContext ctx{};

  const PMXSourceData data = build_pmx_source_data(model, params, {}, {}, {}, ctx);

  EXPECT_FLOAT_EQ(data.pmx_version, 2.1f);
  EXPECT_EQ(data.encoding, int(PMXEncoding::UTF16LE));
  EXPECT_EQ(data.additional_uv_count, 2);
  EXPECT_EQ(data.name_local, kModelNameLocal);
  EXPECT_EQ(data.name_universal, "Source Model");
  EXPECT_EQ(data.comment_local, "comment local");
  EXPECT_EQ(data.comment_universal, "comment universal");
  EXPECT_EQ(data.source_filepath, "D:\\models\\source.pmx");
  EXPECT_FLOAT_EQ(data.global_scale, 0.08f);
  EXPECT_TRUE(data.split_by_material);
  EXPECT_EQ(data.vertex_count, 3);
  EXPECT_EQ(data.face_index_count, 3);
  EXPECT_EQ(data.rigid_body_count, 1);
  EXPECT_EQ(data.joint_count, 0);
}

TEST_F(PMXSourceDataTest, retains_material_fields_lost_by_import)
{
  const PMXModel model = make_source_model();
  const PMXImportParams params = make_params();
  PMXImportContext ctx{};

  const PMXSourceData data = build_pmx_source_data(model, params, {}, {}, {}, ctx);

  ASSERT_EQ(data.materials.size(), 2);
  const PMXSourceMaterial &mat0 = data.materials[0];
  EXPECT_EQ(mat0.pmx_index, 0);
  EXPECT_FLOAT_EQ(mat0.specular_power, 12.5f);
  EXPECT_FLOAT_EQ(mat0.specular[0], 0.4f);
  EXPECT_FLOAT_EQ(mat0.ambient[2], 0.9f);
  EXPECT_EQ(mat0.flag, PMX_MATERIAL_FLAG_DOUBLE_SIDED | PMX_MATERIAL_FLAG_EDGE);
  EXPECT_EQ(mat0.sphere_mode, int(SphereMode::Sphere));
  EXPECT_EQ(mat0.sphere_texture_index, 1);
  EXPECT_EQ(mat0.toon_flag, 0);
  EXPECT_EQ(mat0.toon_internal_value, 3);
  EXPECT_EQ(mat0.memo, "memo 0");
  EXPECT_EQ(mat0.face_vertex_count, 3);

  const PMXSourceMaterial &mat1 = data.materials[1];
  EXPECT_EQ(mat1.toon_flag, 1);
  EXPECT_EQ(mat1.toon_texture_index, 1);
  EXPECT_EQ(mat1.texture_index, -1);
  EXPECT_EQ(mat1.sphere_mode, int(SphereMode::None));
}

TEST_F(PMXSourceDataTest, retains_bone_metadata_lost_by_import)
{
  const PMXModel model = make_source_model();
  const PMXImportParams params = make_params();
  PMXImportContext ctx{};

  const PMXSourceData data = build_pmx_source_data(model, params, {}, {}, {}, ctx);

  ASSERT_EQ(data.bones.size(), 3);

  /* Tail as a bone index. */
  EXPECT_EQ(data.bones[0].tail_pos_bone, 1);
  EXPECT_EQ(data.bones[0].flag & BONE_FLAG_TAIL_POS, BONE_FLAG_TAIL_POS);

  /* Tail as an offset, plus every axis / append / deform field. */
  const PMXSourceBone &twist = data.bones[1];
  EXPECT_EQ(twist.transform_order, 7);
  EXPECT_EQ(twist.tail_pos_bone, -2);
  EXPECT_FLOAT_EQ(twist.tail_pos_offset[1], 1.5f);
  EXPECT_EQ(twist.inherit_parent_index, 0);
  EXPECT_FLOAT_EQ(twist.inherit_parent_ratio, -0.75f);
  EXPECT_FLOAT_EQ(twist.fixed_axis[0], 1.0f);
  EXPECT_FLOAT_EQ(twist.local_x[0], 1.0f);
  EXPECT_FLOAT_EQ(twist.local_z[2], 1.0f);
  EXPECT_EQ(twist.external_parent_index, 42);
  EXPECT_EQ(twist.flag & BONE_FLAG_PHYSICS_AFTER_DEF, BONE_FLAG_PHYSICS_AFTER_DEF);

  /* IK, including a link with and a link without angle limits. */
  const PMXSourceBone &ik = data.bones[2];
  EXPECT_EQ(ik.ik_target_index, 1);
  EXPECT_EQ(ik.ik_loop_count, 40);
  EXPECT_FLOAT_EQ(ik.ik_angle_limit, 0.5f);
  ASSERT_EQ(ik.ik_links.size(), 2);
  EXPECT_TRUE(ik.ik_links[0].limit_angle);
  EXPECT_FLOAT_EQ(ik.ik_links[0].limit_min[0], -1.0f);
  EXPECT_FALSE(ik.ik_links[1].limit_angle);
}

TEST_F(PMXSourceDataTest, retains_non_vertex_morphs_but_not_vertex_offsets)
{
  const PMXModel model = make_source_model();
  const PMXImportParams params = make_params();
  PMXImportContext ctx{};

  const PMXSourceData data = build_pmx_source_data(model, params, {}, {}, {}, ctx);

  ASSERT_EQ(data.morphs.size(), 7);

  /* Vertex Morph: offsets are owned by Shape Keys, only the count is recorded. */
  EXPECT_EQ(data.morphs[0].type, int(MorphType::Vertex));
  EXPECT_EQ(data.morphs[0].vertex_offset_count, 2);
  EXPECT_EQ(data.morphs[0].panel, 1);

  /* Group and Flip both use the group offset list. */
  EXPECT_EQ(data.morphs[1].type, int(MorphType::Group));
  ASSERT_EQ(data.morphs[1].group_offsets.size(), 1);
  EXPECT_FLOAT_EQ(data.morphs[1].group_offsets[0].influence, 0.5f);
  EXPECT_EQ(data.morphs[5].type, int(MorphType::Flip));
  ASSERT_EQ(data.morphs[5].group_offsets.size(), 1);

  /* Bone / UV / Material / Impulse produce no Blender data at all. */
  ASSERT_EQ(data.morphs[2].bone_offsets.size(), 1);
  EXPECT_FLOAT_EQ(data.morphs[2].bone_offsets[0].rot[3], 0.7f);
  EXPECT_EQ(data.morphs[3].type, int(MorphType::UV_2nd));
  ASSERT_EQ(data.morphs[3].uv_offsets.size(), 1);
  EXPECT_FLOAT_EQ(data.morphs[3].uv_offsets[0].offset[3], 0.44f);
  ASSERT_EQ(data.morphs[4].material_offsets.size(), 1);
  EXPECT_EQ(data.morphs[4].material_offsets[0].material_index, -1);
  EXPECT_EQ(data.morphs[4].material_offsets[0].calc_mode, 1);
  ASSERT_EQ(data.morphs[6].impulse_offsets.size(), 1);
  EXPECT_EQ(data.morphs[6].impulse_offsets[0].local_flag, 1);
  EXPECT_FLOAT_EQ(data.morphs[6].impulse_offsets[0].torque[2], 2.5f);
}

TEST_F(PMXSourceDataTest, retains_display_frames)
{
  const PMXModel model = make_source_model();
  const PMXImportParams params = make_params();
  PMXImportContext ctx{};

  const PMXSourceData data = build_pmx_source_data(model, params, {}, {}, {}, ctx);

  ASSERT_EQ(data.display_frames.size(), 2);
  EXPECT_EQ(data.display_frames[0].name_local, "BoneFrame");
  EXPECT_EQ(data.display_frames[0].flag, 0);
  ASSERT_EQ(data.display_frames[0].items.size(), 2);
  EXPECT_EQ(data.display_frames[0].items[1].type, 0);
  EXPECT_EQ(data.display_frames[0].items[1].index, 2);
  EXPECT_EQ(data.display_frames[1].flag, 1);
  ASSERT_EQ(data.display_frames[1].items.size(), 1);
  EXPECT_EQ(data.display_frames[1].items[0].type, 1);
}

TEST_F(PMXSourceDataTest, records_blender_names_for_correlation)
{
  const PMXModel model = make_source_model();
  const PMXImportParams params = make_params();

  PMXImportContext ctx{};
  Material *material = BKE_material_add(bmain, "Mat0Blender");
  ctx.material_cache.add(0, material);

  const Vector<std::string> bone_names = {"Root", "Twist", "IK"};
  const Vector<int> morph_indices = {0};
  const Vector<std::string> morph_names = {"VertexMorphKey"};

  const PMXSourceData data = build_pmx_source_data(
      model, params, bone_names.as_span(), morph_indices.as_span(), morph_names.as_span(), ctx);

  EXPECT_EQ(data.materials[0].blender_material_name, "Mat0Blender");
  EXPECT_TRUE(data.materials[1].blender_material_name.empty());
  EXPECT_EQ(data.bones[0].blender_bone_name, "Root");
  EXPECT_EQ(data.bones[2].blender_bone_name, "IK");
  EXPECT_EQ(data.morphs[0].blender_shape_key_name, "VertexMorphKey");
  EXPECT_TRUE(data.morphs[1].blender_shape_key_name.empty());
}

TEST_F(PMXSourceDataTest, rejects_missing_property)
{
  PMXSourceData read;
  EXPECT_FALSE(deserialize_pmx_source_data(*model_collection, read, &reports));
}

TEST_F(PMXSourceDataTest, rejects_unsupported_schema_version)
{
  const PMXModel model = make_source_model();
  PMXImportContext ctx{};
  const PMXSourceData data = build_pmx_source_data(model, make_params(), {}, {}, {}, ctx);
  ASSERT_TRUE(serialize_pmx_source_data(*model_collection, data, &reports));

  IDProperty *root = source_root();
  ASSERT_NE(root, nullptr);
  IDProperty *schema = IDP_GetPropertyTypeFromGroup(root, "schema_version", IDP_INT);
  ASSERT_NE(schema, nullptr);
  IDP_int_set(schema, kPMXSourceDataSchemaVersion + 1);

  PMXSourceData read;
  EXPECT_FALSE(deserialize_pmx_source_data(*model_collection, read, &reports));
}

TEST_F(PMXSourceDataTest, rejects_count_metadata_mismatch)
{
  const PMXModel model = make_source_model();
  PMXImportContext ctx{};
  const PMXSourceData data = build_pmx_source_data(model, make_params(), {}, {}, {}, ctx);
  ASSERT_TRUE(serialize_pmx_source_data(*model_collection, data, &reports));

  IDProperty *root = source_root();
  ASSERT_NE(root, nullptr);
  IDProperty *bone_count = IDP_GetPropertyTypeFromGroup(root, "bone_count", IDP_INT);
  ASSERT_NE(bone_count, nullptr);
  IDP_int_set(bone_count, 99);

  PMXSourceData read;
  EXPECT_FALSE(deserialize_pmx_source_data(*model_collection, read, &reports));
}

TEST_F(PMXSourceDataTest, replaces_previously_persisted_value)
{
  const PMXModel model = make_source_model();
  PMXImportContext ctx{};
  const PMXSourceData first = build_pmx_source_data(model, make_params(), {}, {}, {}, ctx);
  ASSERT_TRUE(serialize_pmx_source_data(*model_collection, first, &reports));

  PMXModel second_model = make_source_model();
  second_model.name_universal = "Replaced";
  second_model.display_frames.clear();
  const PMXSourceData second = build_pmx_source_data(
      second_model, make_params(), {}, {}, {}, ctx);
  ASSERT_TRUE(serialize_pmx_source_data(*model_collection, second, &reports));

  PMXSourceData read;
  ASSERT_TRUE(deserialize_pmx_source_data(*model_collection, read, &reports));
  EXPECT_EQ(read.name_universal, "Replaced");
  EXPECT_TRUE(read.display_frames.empty());
}

TEST_F(PMXSourceDataTest, writes_vertex_index_attribute_in_single_mesh_mode)
{
  const PMXModel model = make_source_model();
  Object *obj = add_mesh_object("SingleMesh", int(model.vertices.size()));

  ASSERT_TRUE(write_pmx_vertex_index_attribute(obj, model, nullptr));

  const Mesh *mesh = reinterpret_cast<const Mesh *>(obj->data);
  const bke::AttributeAccessor attributes = mesh->attributes();
  const bke::AttributeReader<int> indices = attributes.lookup<int>(kPMXVertexIndexAttribute,
                                                                  bke::AttrDomain::Point);
  ASSERT_TRUE(indices);
  ASSERT_EQ(indices.varray.size(), 3);
  EXPECT_EQ(indices.varray[0], 0);
  EXPECT_EQ(indices.varray[1], 1);
  EXPECT_EQ(indices.varray[2], 2);
}

TEST_F(PMXSourceDataTest, writes_vertex_index_attribute_in_split_mode)
{
  const PMXModel model = make_source_model();
  Object *obj = add_mesh_object("SplitMesh", 2);
  const Vector<int> new_to_old = {2, 0};

  ASSERT_TRUE(write_pmx_vertex_index_attribute(obj, model, &new_to_old));

  const Mesh *mesh = reinterpret_cast<const Mesh *>(obj->data);
  const bke::AttributeAccessor attributes = mesh->attributes();
  const bke::AttributeReader<int> indices = attributes.lookup<int>(kPMXVertexIndexAttribute,
                                                                  bke::AttrDomain::Point);
  ASSERT_TRUE(indices);
  ASSERT_EQ(indices.varray.size(), 2);
  EXPECT_EQ(indices.varray[0], 2);
  EXPECT_EQ(indices.varray[1], 0);
}

TEST_F(PMXSourceDataTest, refuses_vertex_index_write_on_count_mismatch)
{
  const PMXModel model = make_source_model();
  Object *obj = add_mesh_object("WrongCount", 5);

  EXPECT_FALSE(write_pmx_vertex_index_attribute(obj, model, nullptr));

  const Mesh *mesh = reinterpret_cast<const Mesh *>(obj->data);
  const bke::AttributeAccessor attributes = mesh->attributes();
  EXPECT_FALSE(attributes.lookup<int>(kPMXVertexIndexAttribute, bke::AttrDomain::Point));
}

TEST_F(PMXSourceDataTest, writes_additional_uv_attributes)
{
  const PMXModel model = make_source_model();
  Object *obj = add_mesh_object("AddUVMesh", int(model.vertices.size()));

  ASSERT_TRUE(write_pmx_additional_uv_attributes(obj, model, nullptr));

  const Mesh *mesh = reinterpret_cast<const Mesh *>(obj->data);
  const bke::AttributeAccessor attributes = mesh->attributes();

  const bke::AttributeReader<float2> uv0_xy = attributes.lookup<float2>("pmx_add_uv0_xy",
                                                                       bke::AttrDomain::Point);
  const bke::AttributeReader<float2> uv0_zw = attributes.lookup<float2>("pmx_add_uv0_zw",
                                                                       bke::AttrDomain::Point);
  const bke::AttributeReader<float2> uv1_xy = attributes.lookup<float2>("pmx_add_uv1_xy",
                                                                       bke::AttrDomain::Point);
  const bke::AttributeReader<float2> uv1_zw = attributes.lookup<float2>("pmx_add_uv1_zw",
                                                                       bke::AttrDomain::Point);
  ASSERT_TRUE(uv0_xy);
  ASSERT_TRUE(uv0_zw);
  ASSERT_TRUE(uv1_xy);
  ASSERT_TRUE(uv1_zw);

  /* Set 3 is never created: the header declares only two. */
  EXPECT_FALSE(attributes.lookup<float2>("pmx_add_uv2_xy", bke::AttrDomain::Point));

  EXPECT_EQ(uv0_xy.varray[1], float2(1.1f, 1.2f));
  EXPECT_EQ(uv0_zw.varray[1], float2(1.3f, 1.4f));
  EXPECT_EQ(uv1_xy.varray[2], float2(2.5f, 2.6f));
  EXPECT_EQ(uv1_zw.varray[2], float2(2.7f, 2.8f));
}

TEST_F(PMXSourceDataTest, skips_additional_uv_when_model_has_none)
{
  PMXModel model = make_source_model();
  model.header.add_uv_cnt = 0;
  Object *obj = add_mesh_object("NoAddUV", int(model.vertices.size()));

  EXPECT_TRUE(write_pmx_additional_uv_attributes(obj, model, nullptr));

  const Mesh *mesh = reinterpret_cast<const Mesh *>(obj->data);
  const bke::AttributeAccessor attributes = mesh->attributes();
  EXPECT_FALSE(attributes.lookup<float2>("pmx_add_uv0_xy", bke::AttrDomain::Point));
}

TEST_F(PMXSourceDataTest, persists_source_data_and_attributes_for_one_import)
{
  const PMXModel model = make_source_model();
  const PMXImportParams params = make_params();
  Object *obj = add_mesh_object("PersistMesh", int(model.vertices.size()));

  PMXImportContext ctx{};
  ctx.bmain = bmain;
  ctx.params = &params;
  ctx.reports = &reports;
  ctx.model_collection = model_collection;
  ctx.mesh_objects.append(obj);
  SubMeshInfo sub;
  sub.obj = obj;
  sub.new_to_old = {0, 1, 2};
  ctx.sub_meshes.append(std::move(sub));

  const Vector<std::string> bone_names = {"Root", "Twist", "IK"};
  persist_pmx_source_data(ctx, model, bone_names.as_span());

  PMXSourceData read;
  ASSERT_TRUE(deserialize_pmx_source_data(*model_collection, read, &reports));
  EXPECT_EQ(read.bones[1].blender_bone_name, "Twist");
  EXPECT_EQ(read.name_universal, "Source Model");

  const Mesh *mesh = reinterpret_cast<const Mesh *>(obj->data);
  const bke::AttributeAccessor attributes = mesh->attributes();
  EXPECT_TRUE(attributes.lookup<int>(kPMXVertexIndexAttribute, bke::AttrDomain::Point));
  EXPECT_TRUE(attributes.lookup<float2>("pmx_add_uv1_zw", bke::AttrDomain::Point));
}

/**
 * A PMX with no textures, materials, bones, morphs or display frames is valid.
 * Every retained section then serializes as a zero-length IDPArray, which must
 * still read back rather than being mistaken for a missing property.
 */
TEST_F(PMXSourceDataTest, round_trips_data_with_no_sections)
{
  PMXSourceData data;
  data.global_scale = 0.08f;

  ASSERT_TRUE(serialize_pmx_source_data(*model_collection, data, &reports));
  PMXSourceData read;
  ASSERT_TRUE(deserialize_pmx_source_data(*model_collection, read, &reports));

  EXPECT_EQ(read.schema_version, kPMXSourceDataSchemaVersion);
  EXPECT_TRUE(read.textures.empty());
  EXPECT_TRUE(read.materials.empty());
  EXPECT_TRUE(read.bones.empty());
  EXPECT_TRUE(read.morphs.empty());
  EXPECT_TRUE(read.display_frames.empty());
}

}  // namespace
}  // namespace blender::io::pmx::tests
