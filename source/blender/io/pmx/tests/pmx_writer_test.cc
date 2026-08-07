/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "testing/testing.h"

#include "BLI_fileops.hh"
#include "BLI_path_utils.hh"
#include "BLI_tempfile.hh"

#include "intern/pmx_reader.h"
#include "intern/pmx_types.h"
#include "intern/pmx_writer.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace blender::io::pmx::tests {
namespace {

/* UTF-8 byte sequences are spelled out so the test does not depend on the
 * compiler's execution character set. */
constexpr const char *kNameLocal = "\xE3\x83\x86\xE3\x82\xB9\xE3\x83\x88"; /* テスト */
constexpr const char *kNameUniversal = "Test Model";
/* Includes U+1F3B5 so UTF-16LE surrogate pairs are covered. */
constexpr const char *kCommentLocal = "\xE3\x82\xB3\xE3\x83\xA1 \xF0\x9F\x8E\xB5";
constexpr const char *kCommentUniversal = "comment";

void set3(float out[3], const float x, const float y, const float z)
{
  out[0] = x;
  out[1] = y;
  out[2] = z;
}

void set4(float out[4], const float x, const float y, const float z, const float w)
{
  out[0] = x;
  out[1] = y;
  out[2] = z;
  out[3] = w;
}

PMXVertex make_vertex(const BoneWeightType weight_type,
                      const std::vector<int> &bones,
                      const std::vector<float> &weights,
                      const float base)
{
  PMXVertex v{};
  /* Every vertex carries the header's additional UV count, mirroring the reader. */
  v.additional_uv_count = 2;
  set3(v.pos, base, base + 1.0f, base + 2.0f);
  set3(v.normal, 0.0f, 1.0f, 0.0f);
  v.uv[0] = base * 0.125f;
  v.uv[1] = base * 0.25f;
  v.additional_uv[0] = {base, base + 0.5f, base + 1.5f, base + 2.5f};
  v.additional_uv[1] = {-base, -base - 0.5f, -base - 1.5f, -base - 2.5f};
  v.weight_type = weight_type;
  v.bone_indices = bones;
  v.bone_weights = weights;
  v.edge_factor = base * 0.5f;
  return v;
}

/**
 * A model exercising every PMX section, both toon modes, all five weight types,
 * every morph type and each optional bone-flag payload.
 */
PMXModel make_round_trip_model()
{
  PMXModel model;
  model.header.version = 2.1f;
  model.header.header_size = 8;
  model.header.encoding = uint8_t(PMXEncoding::UTF16LE);
  model.header.add_uv_cnt = 2;
  model.header.signature[0] = 'P';
  model.header.signature[1] = 'M';
  model.header.signature[2] = 'X';
  model.header.signature[3] = ' ';
  /* Index sizes are left at zero on purpose: the writer recomputes them. */

  model.name_local = kNameLocal;
  model.name_universal = kNameUniversal;
  model.comment_local = kCommentLocal;
  model.comment_universal = kCommentUniversal;

  /* --- Bones (4) --- */
  PMXBone root{};
  root.name_local = "root";
  root.name_universal = "root_e";
  set3(root.pos, 0.0f, 0.0f, 0.0f);
  root.parent_index = -1;
  root.transform_order = 0;
  root.flag = BONE_FLAG_ROTATABLE | BONE_FLAG_VISIBLE | BONE_FLAG_ENABLED;
  root.tail_pos_bone = -2; /* TAIL_POS clear: the offset form is written. */
  set3(root.tail_pos_offset, 0.0f, 1.0f, 0.0f);
  root.inherit_parent_index = -1;
  root.ik_target_index = -1;
  root.external_parent_index = -1;
  model.bones.push_back(root);

  PMXBone tail_indexed{};
  tail_indexed.name_local = "spine";
  tail_indexed.name_universal = "spine_e";
  set3(tail_indexed.pos, 0.0f, 1.0f, 0.0f);
  tail_indexed.parent_index = 0;
  tail_indexed.transform_order = 1;
  tail_indexed.flag = BONE_FLAG_TAIL_POS | BONE_FLAG_ROTATABLE | BONE_FLAG_TRANSLATABLE |
                      BONE_FLAG_VISIBLE | BONE_FLAG_ENABLED;
  tail_indexed.tail_pos_bone = 2;
  tail_indexed.inherit_parent_index = -1;
  tail_indexed.ik_target_index = -1;
  tail_indexed.external_parent_index = -1;
  model.bones.push_back(tail_indexed);

  PMXBone decorated{};
  decorated.name_local = "twist";
  decorated.name_universal = "twist_e";
  set3(decorated.pos, 0.0f, 2.0f, 0.0f);
  decorated.parent_index = 1;
  decorated.transform_order = 2;
  decorated.flag = BONE_FLAG_APPEND_ROTATION | BONE_FLAG_APPEND_TRANSLATE |
                   BONE_FLAG_FIXED_AXIS | BONE_FLAG_LOCAL_AXIS |
                   BONE_FLAG_PHYSICS_AFTER_DEF | BONE_FLAG_EXTERNAL_PARENT;
  decorated.tail_pos_bone = -2;
  set3(decorated.tail_pos_offset, 0.25f, 0.5f, 0.75f);
  decorated.inherit_parent_index = 1;
  decorated.inherit_parent_ratio = 0.5f;
  set3(decorated.fixed_axis, 1.0f, 0.0f, 0.0f);
  set3(decorated.local_x, 1.0f, 0.0f, 0.0f);
  set3(decorated.local_z, 0.0f, 0.0f, 1.0f);
  decorated.ik_target_index = -1;
  decorated.external_parent_index = 7;
  model.bones.push_back(decorated);

  PMXBone ik{};
  ik.name_local = "ik";
  ik.name_universal = "ik_e";
  set3(ik.pos, 0.0f, 3.0f, 0.0f);
  ik.parent_index = 0;
  ik.transform_order = 3;
  ik.flag = BONE_FLAG_IK | BONE_FLAG_ROTATABLE | BONE_FLAG_VISIBLE | BONE_FLAG_ENABLED;
  ik.tail_pos_bone = -2;
  set3(ik.tail_pos_offset, 0.0f, -1.0f, 0.0f);
  ik.inherit_parent_index = -1;
  ik.ik_target_index = 2;
  ik.ik_loop_count = 40;
  ik.ik_angle_limit = 0.5f;
  ik.external_parent_index = -1;
  PMXIKLink limited{};
  limited.bone_index = 1;
  limited.limit_angle = true;
  set3(limited.limit_min, -1.0f, -0.5f, -0.25f);
  set3(limited.limit_max, 1.0f, 0.5f, 0.25f);
  ik.ik_links.push_back(limited);
  PMXIKLink unlimited{};
  unlimited.bone_index = 0;
  unlimited.limit_angle = false;
  ik.ik_links.push_back(unlimited);
  model.bones.push_back(ik);

  /* --- Vertices (6), one per weight type plus a spare --- */
  model.vertices.push_back(make_vertex(BoneWeightType::BDEF1, {0}, {1.0f}, 0.0f));
  model.vertices.push_back(
      make_vertex(BoneWeightType::BDEF2, {0, 1}, {0.25f, 1.0f - 0.25f}, 1.0f));
  model.vertices.push_back(
      make_vertex(BoneWeightType::BDEF4, {0, 1, 2, 3}, {0.4f, 0.3f, 0.2f, 0.1f}, 2.0f));
  PMXVertex sdef = make_vertex(BoneWeightType::SDEF, {1, 2}, {0.6f, 1.0f - 0.6f}, 3.0f);
  set3(sdef.sdef_c, 1.0f, 2.0f, 3.0f);
  set3(sdef.sdef_r0, 4.0f, 5.0f, 6.0f);
  set3(sdef.sdef_r1, 7.0f, 8.0f, 9.0f);
  model.vertices.push_back(sdef);
  model.vertices.push_back(
      make_vertex(BoneWeightType::QDEF, {0, 1, 2, 3}, {0.25f, 0.25f, 0.25f, 0.25f}, 4.0f));
  model.vertices.push_back(make_vertex(BoneWeightType::BDEF1, {3}, {1.0f}, 5.0f));

  /* --- Faces (2 triangles) --- */
  model.face_indices = {0, 1, 2, 3, 4, 5};

  /* --- Textures (2) --- */
  model.textures.push_back({"tex\\body.png"});
  model.textures.push_back({"tex\\toon.bmp"});

  /* --- Materials (2): internal toon and external toon --- */
  PMXMaterial internal_toon{};
  internal_toon.name_local = "body";
  internal_toon.name_universal = "body_e";
  set4(internal_toon.diffuse, 1.0f, 0.5f, 0.25f, 1.0f);
  set3(internal_toon.specular, 0.1f, 0.2f, 0.3f);
  internal_toon.specular_power = 5.0f;
  set3(internal_toon.ambient, 0.4f, 0.5f, 0.6f);
  internal_toon.flag = PMX_MATERIAL_FLAG_DOUBLE_SIDED | PMX_MATERIAL_FLAG_EDGE;
  set4(internal_toon.edge_color, 0.0f, 0.0f, 0.0f, 1.0f);
  internal_toon.edge_size = 1.0f;
  internal_toon.texture_idx = 0;
  internal_toon.sphere_texture_idx = 1;
  internal_toon.sphere_mode = SphereMode::Sphere;
  internal_toon.toon_flag = 0;
  internal_toon.toon_texture_idx = -1; /* Untouched by the reader when toon_flag == 0. */
  internal_toon.toon_internal_value = 3;
  internal_toon.memo = "memo";
  internal_toon.face_vertex_count = 3;
  model.materials.push_back(internal_toon);

  PMXMaterial external_toon{};
  external_toon.name_local = "hair";
  external_toon.name_universal = "hair_e";
  set4(external_toon.diffuse, 0.2f, 0.3f, 0.4f, 0.5f);
  set3(external_toon.specular, 0.0f, 0.0f, 0.0f);
  external_toon.specular_power = 1.0f;
  set3(external_toon.ambient, 0.0f, 0.0f, 0.0f);
  external_toon.flag = PMX_MATERIAL_FLAG_SELF_SHADOW;
  set4(external_toon.edge_color, 1.0f, 1.0f, 1.0f, 1.0f);
  external_toon.edge_size = 0.5f;
  external_toon.texture_idx = -1;
  external_toon.sphere_texture_idx = -1;
  external_toon.sphere_mode = SphereMode::None;
  external_toon.toon_flag = 1;
  external_toon.toon_texture_idx = 1;
  external_toon.toon_internal_value = 0; /* Untouched by the reader when toon_flag == 1. */
  external_toon.face_vertex_count = 3;
  model.materials.push_back(external_toon);

  /* --- Morphs (8), covering every type --- */
  PMXMorph vertex_morph{};
  vertex_morph.name_local = "vertex";
  vertex_morph.name_universal = "vertex_e";
  vertex_morph.panel = 1;
  vertex_morph.type = MorphType::Vertex;
  vertex_morph.vertex_offsets.push_back({0, {0.1f, 0.2f, 0.3f}});
  vertex_morph.vertex_offsets.push_back({3, {-0.1f, -0.2f, -0.3f}});
  model.morphs.push_back(vertex_morph);

  PMXMorph group_morph{};
  group_morph.name_local = "group";
  group_morph.name_universal = "group_e";
  group_morph.panel = 0;
  group_morph.type = MorphType::Group;
  group_morph.group_offsets.push_back({0, 0.5f});
  model.morphs.push_back(group_morph);

  PMXMorph bone_morph{};
  bone_morph.name_local = "bone";
  bone_morph.name_universal = "bone_e";
  bone_morph.panel = 3;
  bone_morph.type = MorphType::Bone;
  PMXBoneMorphOffset bone_offset{};
  bone_offset.bone_index = 1;
  set3(bone_offset.pos, 1.0f, 2.0f, 3.0f);
  set4(bone_offset.rot, 0.0f, 0.0f, 0.0f, 1.0f);
  bone_morph.bone_offsets.push_back(bone_offset);
  model.morphs.push_back(bone_morph);

  PMXMorph uv_morph{};
  uv_morph.name_local = "uv1";
  uv_morph.name_universal = "uv1_e";
  uv_morph.panel = 4;
  uv_morph.type = MorphType::UV;
  uv_morph.uv_offsets.push_back({2, {0.1f, 0.2f, 0.3f, 0.4f}});
  model.morphs.push_back(uv_morph);

  PMXMorph uv3_morph{};
  uv3_morph.name_local = "uv3";
  uv3_morph.name_universal = "uv3_e";
  uv3_morph.panel = 4;
  uv3_morph.type = MorphType::UV_3rd;
  uv3_morph.uv_offsets.push_back({4, {-0.1f, -0.2f, -0.3f, -0.4f}});
  model.morphs.push_back(uv3_morph);

  PMXMorph material_morph{};
  material_morph.name_local = "material";
  material_morph.name_universal = "material_e";
  material_morph.panel = 4;
  material_morph.type = MorphType::Material;
  PMXMaterialMorphOffset mul{};
  mul.material_index = 0;
  mul.calc_mode = 0;
  set4(mul.diffuse, 1.0f, 1.0f, 1.0f, 1.0f);
  set3(mul.specular, 1.0f, 1.0f, 1.0f);
  mul.specular_power = 1.0f;
  set3(mul.ambient, 1.0f, 1.0f, 1.0f);
  set4(mul.edge_color, 1.0f, 1.0f, 1.0f, 1.0f);
  mul.edge_size = 1.0f;
  set4(mul.texture_factor, 1.0f, 1.0f, 1.0f, 1.0f);
  set4(mul.sphere_texture_factor, 1.0f, 1.0f, 1.0f, 1.0f);
  set4(mul.toon_texture_factor, 1.0f, 1.0f, 1.0f, 1.0f);
  material_morph.material_offsets.push_back(mul);
  PMXMaterialMorphOffset add_all{};
  add_all.material_index = -1; /* -1 targets every material. */
  add_all.calc_mode = 1;
  set4(add_all.diffuse, 0.1f, 0.2f, 0.3f, 0.4f);
  set3(add_all.specular, 0.5f, 0.6f, 0.7f);
  add_all.specular_power = 2.0f;
  set3(add_all.ambient, 0.8f, 0.9f, 1.0f);
  set4(add_all.edge_color, 0.1f, 0.1f, 0.1f, 0.1f);
  add_all.edge_size = 0.25f;
  set4(add_all.texture_factor, 0.2f, 0.3f, 0.4f, 0.5f);
  set4(add_all.sphere_texture_factor, 0.3f, 0.4f, 0.5f, 0.6f);
  set4(add_all.toon_texture_factor, 0.4f, 0.5f, 0.6f, 0.7f);
  material_morph.material_offsets.push_back(add_all);
  model.morphs.push_back(material_morph);

  PMXMorph flip_morph{};
  flip_morph.name_local = "flip";
  flip_morph.name_universal = "flip_e";
  flip_morph.panel = 4;
  flip_morph.type = MorphType::Flip;
  flip_morph.group_offsets.push_back({0, 1.0f});
  model.morphs.push_back(flip_morph);

  PMXMorph impulse_morph{};
  impulse_morph.name_local = "impulse";
  impulse_morph.name_universal = "impulse_e";
  impulse_morph.panel = 4;
  impulse_morph.type = MorphType::Impulse;
  PMXImpulseMorphOffset impulse{};
  impulse.rigid_index = 0;
  impulse.local_flag = 1;
  set3(impulse.velocity, 1.0f, 2.0f, 3.0f);
  set3(impulse.torque, 4.0f, 5.0f, 6.0f);
  impulse_morph.impulse_offsets.push_back(impulse);
  model.morphs.push_back(impulse_morph);

  /* --- Display frames (2): bone frame and morph frame --- */
  PMXDisplayFrame bone_frame{};
  bone_frame.name_local = "Root";
  bone_frame.name_universal = "Root_e";
  bone_frame.flag = 0;
  bone_frame.items.push_back({0, 0});
  bone_frame.items.push_back({0, 3});
  model.display_frames.push_back(bone_frame);

  PMXDisplayFrame morph_frame{};
  morph_frame.name_local = "Exp";
  morph_frame.name_universal = "Exp_e";
  morph_frame.flag = 1;
  morph_frame.items.push_back({1, 0});
  morph_frame.items.push_back({1, 7});
  model.display_frames.push_back(morph_frame);

  /* --- Rigid bodies (2) --- */
  PMXRigidBody sphere{};
  sphere.name_local = "rb_sphere";
  sphere.name_universal = "rb_sphere_e";
  sphere.bone_index = 1;
  sphere.collision_group = 0;
  sphere.no_collision_group = 0xFFFE;
  sphere.shape_type = 0;
  set3(sphere.shape_size, 1.0f, 0.0f, 0.0f);
  set3(sphere.pos, 0.0f, 1.0f, 0.0f);
  set3(sphere.rot, 0.0f, 0.0f, 0.0f);
  sphere.mass = 1.0f;
  sphere.linear_damping = 0.5f;
  sphere.angular_damping = 0.5f;
  sphere.restitution = 0.0f;
  sphere.friction = 0.5f;
  sphere.physics_type = 0;
  model.rigid_bodies.push_back(sphere);

  PMXRigidBody capsule{};
  capsule.name_local = "rb_capsule";
  capsule.name_universal = "rb_capsule_e";
  capsule.bone_index = -1; /* Unbound rigid body. */
  capsule.collision_group = 3;
  capsule.no_collision_group = 0;
  capsule.shape_type = 2;
  set3(capsule.shape_size, 0.5f, 2.0f, 0.0f);
  set3(capsule.pos, 0.0f, 2.0f, 0.0f);
  set3(capsule.rot, 0.1f, 0.2f, 0.3f);
  capsule.mass = 2.0f;
  capsule.linear_damping = 0.25f;
  capsule.angular_damping = 0.25f;
  capsule.restitution = 0.1f;
  capsule.friction = 0.9f;
  capsule.physics_type = 2;
  model.rigid_bodies.push_back(capsule);

  /* --- Joints (1) --- */
  PMXJoint joint{};
  joint.name_local = "joint";
  joint.name_universal = "joint_e";
  joint.type = 0;
  joint.rigid_a_index = 0;
  joint.rigid_b_index = 1;
  set3(joint.pos, 0.0f, 1.5f, 0.0f);
  set3(joint.rot, 0.0f, 0.0f, 0.0f);
  set3(joint.translation_limit_min, -1.0f, -1.0f, -1.0f);
  set3(joint.translation_limit_max, 1.0f, 1.0f, 1.0f);
  set3(joint.rotation_limit_min, -0.5f, -0.5f, -0.5f);
  set3(joint.rotation_limit_max, 0.5f, 0.5f, 0.5f);
  set3(joint.spring_translation, 1.0f, 2.0f, 3.0f);
  set3(joint.spring_rotation, 4.0f, 5.0f, 6.0f);
  model.joints.push_back(joint);

  return model;
}

void expect_float3_equal(const float expected[3], const float actual[3])
{
  for (int i = 0; i < 3; i++) {
    EXPECT_FLOAT_EQ(actual[i], expected[i]);
  }
}

void expect_float4_equal(const float expected[4], const float actual[4])
{
  for (int i = 0; i < 4; i++) {
    EXPECT_FLOAT_EQ(actual[i], expected[i]);
  }
}

void expect_models_equal(const PMXModel &expected, const PMXModel &actual)
{
  EXPECT_FLOAT_EQ(actual.header.version, expected.header.version);
  EXPECT_EQ(actual.header.header_size, 8);
  EXPECT_EQ(actual.header.encoding, expected.header.encoding);
  EXPECT_EQ(actual.header.add_uv_cnt, expected.header.add_uv_cnt);

  EXPECT_EQ(actual.name_local, expected.name_local);
  EXPECT_EQ(actual.name_universal, expected.name_universal);
  EXPECT_EQ(actual.comment_local, expected.comment_local);
  EXPECT_EQ(actual.comment_universal, expected.comment_universal);

  ASSERT_EQ(actual.vertices.size(), expected.vertices.size());
  for (size_t i = 0; i < expected.vertices.size(); i++) {
    const PMXVertex &e = expected.vertices[i];
    const PMXVertex &a = actual.vertices[i];
    SCOPED_TRACE("vertex " + std::to_string(i));
    expect_float3_equal(e.pos, a.pos);
    expect_float3_equal(e.normal, a.normal);
    EXPECT_FLOAT_EQ(a.uv[0], e.uv[0]);
    EXPECT_FLOAT_EQ(a.uv[1], e.uv[1]);
    EXPECT_EQ(a.additional_uv_count, e.additional_uv_count);
    for (int uv = 0; uv < 4; uv++) {
      expect_float4_equal(e.additional_uv[uv].data(), a.additional_uv[uv].data());
    }
    EXPECT_EQ(int(a.weight_type), int(e.weight_type));
    EXPECT_EQ(a.bone_indices, e.bone_indices);
    ASSERT_EQ(a.bone_weights.size(), e.bone_weights.size());
    for (size_t w = 0; w < e.bone_weights.size(); w++) {
      EXPECT_FLOAT_EQ(a.bone_weights[w], e.bone_weights[w]);
    }
    if (e.weight_type == BoneWeightType::SDEF) {
      expect_float3_equal(e.sdef_c, a.sdef_c);
      expect_float3_equal(e.sdef_r0, a.sdef_r0);
      expect_float3_equal(e.sdef_r1, a.sdef_r1);
    }
    EXPECT_FLOAT_EQ(a.edge_factor, e.edge_factor);
  }

  EXPECT_EQ(actual.face_indices, expected.face_indices);

  ASSERT_EQ(actual.textures.size(), expected.textures.size());
  for (size_t i = 0; i < expected.textures.size(); i++) {
    EXPECT_EQ(actual.textures[i].path, expected.textures[i].path);
  }

  ASSERT_EQ(actual.materials.size(), expected.materials.size());
  for (size_t i = 0; i < expected.materials.size(); i++) {
    const PMXMaterial &e = expected.materials[i];
    const PMXMaterial &a = actual.materials[i];
    SCOPED_TRACE("material " + std::to_string(i));
    EXPECT_EQ(a.name_local, e.name_local);
    EXPECT_EQ(a.name_universal, e.name_universal);
    expect_float4_equal(e.diffuse, a.diffuse);
    expect_float3_equal(e.specular, a.specular);
    EXPECT_FLOAT_EQ(a.specular_power, e.specular_power);
    expect_float3_equal(e.ambient, a.ambient);
    EXPECT_EQ(a.flag, e.flag);
    expect_float4_equal(e.edge_color, a.edge_color);
    EXPECT_FLOAT_EQ(a.edge_size, e.edge_size);
    EXPECT_EQ(a.texture_idx, e.texture_idx);
    EXPECT_EQ(a.sphere_texture_idx, e.sphere_texture_idx);
    EXPECT_EQ(int(a.sphere_mode), int(e.sphere_mode));
    EXPECT_EQ(a.toon_flag, e.toon_flag);
    EXPECT_EQ(a.toon_texture_idx, e.toon_texture_idx);
    EXPECT_EQ(a.toon_internal_value, e.toon_internal_value);
    EXPECT_EQ(a.memo, e.memo);
    EXPECT_EQ(a.face_vertex_count, e.face_vertex_count);
  }

  ASSERT_EQ(actual.bones.size(), expected.bones.size());
  for (size_t i = 0; i < expected.bones.size(); i++) {
    const PMXBone &e = expected.bones[i];
    const PMXBone &a = actual.bones[i];
    SCOPED_TRACE("bone " + std::to_string(i));
    EXPECT_EQ(a.name_local, e.name_local);
    EXPECT_EQ(a.name_universal, e.name_universal);
    expect_float3_equal(e.pos, a.pos);
    EXPECT_EQ(a.parent_index, e.parent_index);
    EXPECT_EQ(a.transform_order, e.transform_order);
    EXPECT_EQ(a.flag, e.flag);
    EXPECT_EQ(a.tail_pos_bone, e.tail_pos_bone);
    if ((e.flag & BONE_FLAG_TAIL_POS) == 0) {
      expect_float3_equal(e.tail_pos_offset, a.tail_pos_offset);
    }
    if (e.flag & (BONE_FLAG_APPEND_ROTATION | BONE_FLAG_APPEND_TRANSLATE)) {
      EXPECT_EQ(a.inherit_parent_index, e.inherit_parent_index);
      EXPECT_FLOAT_EQ(a.inherit_parent_ratio, e.inherit_parent_ratio);
    }
    if (e.flag & BONE_FLAG_FIXED_AXIS) {
      expect_float3_equal(e.fixed_axis, a.fixed_axis);
    }
    if (e.flag & BONE_FLAG_LOCAL_AXIS) {
      expect_float3_equal(e.local_x, a.local_x);
      expect_float3_equal(e.local_z, a.local_z);
    }
    if (e.flag & BONE_FLAG_EXTERNAL_PARENT) {
      EXPECT_EQ(a.external_parent_index, e.external_parent_index);
    }
    if (e.flag & BONE_FLAG_IK) {
      EXPECT_EQ(a.ik_target_index, e.ik_target_index);
      EXPECT_EQ(a.ik_loop_count, e.ik_loop_count);
      EXPECT_FLOAT_EQ(a.ik_angle_limit, e.ik_angle_limit);
      ASSERT_EQ(a.ik_links.size(), e.ik_links.size());
      for (size_t l = 0; l < e.ik_links.size(); l++) {
        EXPECT_EQ(a.ik_links[l].bone_index, e.ik_links[l].bone_index);
        EXPECT_EQ(a.ik_links[l].limit_angle, e.ik_links[l].limit_angle);
        if (e.ik_links[l].limit_angle) {
          expect_float3_equal(e.ik_links[l].limit_min, a.ik_links[l].limit_min);
          expect_float3_equal(e.ik_links[l].limit_max, a.ik_links[l].limit_max);
        }
      }
    }
  }

  ASSERT_EQ(actual.morphs.size(), expected.morphs.size());
  for (size_t i = 0; i < expected.morphs.size(); i++) {
    const PMXMorph &e = expected.morphs[i];
    const PMXMorph &a = actual.morphs[i];
    SCOPED_TRACE("morph " + std::to_string(i));
    EXPECT_EQ(a.name_local, e.name_local);
    EXPECT_EQ(a.name_universal, e.name_universal);
    EXPECT_EQ(a.panel, e.panel);
    EXPECT_EQ(int(a.type), int(e.type));

    ASSERT_EQ(a.group_offsets.size(), e.group_offsets.size());
    for (size_t o = 0; o < e.group_offsets.size(); o++) {
      EXPECT_EQ(a.group_offsets[o].morph_index, e.group_offsets[o].morph_index);
      EXPECT_FLOAT_EQ(a.group_offsets[o].influence, e.group_offsets[o].influence);
    }
    ASSERT_EQ(a.vertex_offsets.size(), e.vertex_offsets.size());
    for (size_t o = 0; o < e.vertex_offsets.size(); o++) {
      EXPECT_EQ(a.vertex_offsets[o].vertex_index, e.vertex_offsets[o].vertex_index);
      expect_float3_equal(e.vertex_offsets[o].offset, a.vertex_offsets[o].offset);
    }
    ASSERT_EQ(a.bone_offsets.size(), e.bone_offsets.size());
    for (size_t o = 0; o < e.bone_offsets.size(); o++) {
      EXPECT_EQ(a.bone_offsets[o].bone_index, e.bone_offsets[o].bone_index);
      expect_float3_equal(e.bone_offsets[o].pos, a.bone_offsets[o].pos);
      expect_float4_equal(e.bone_offsets[o].rot, a.bone_offsets[o].rot);
    }
    ASSERT_EQ(a.uv_offsets.size(), e.uv_offsets.size());
    for (size_t o = 0; o < e.uv_offsets.size(); o++) {
      EXPECT_EQ(a.uv_offsets[o].vertex_index, e.uv_offsets[o].vertex_index);
      expect_float4_equal(e.uv_offsets[o].offset, a.uv_offsets[o].offset);
    }
    ASSERT_EQ(a.material_offsets.size(), e.material_offsets.size());
    for (size_t o = 0; o < e.material_offsets.size(); o++) {
      const PMXMaterialMorphOffset &eo = e.material_offsets[o];
      const PMXMaterialMorphOffset &ao = a.material_offsets[o];
      EXPECT_EQ(ao.material_index, eo.material_index);
      EXPECT_EQ(ao.calc_mode, eo.calc_mode);
      expect_float4_equal(eo.diffuse, ao.diffuse);
      expect_float3_equal(eo.specular, ao.specular);
      EXPECT_FLOAT_EQ(ao.specular_power, eo.specular_power);
      expect_float3_equal(eo.ambient, ao.ambient);
      expect_float4_equal(eo.edge_color, ao.edge_color);
      EXPECT_FLOAT_EQ(ao.edge_size, eo.edge_size);
      expect_float4_equal(eo.texture_factor, ao.texture_factor);
      expect_float4_equal(eo.sphere_texture_factor, ao.sphere_texture_factor);
      expect_float4_equal(eo.toon_texture_factor, ao.toon_texture_factor);
    }
    ASSERT_EQ(a.impulse_offsets.size(), e.impulse_offsets.size());
    for (size_t o = 0; o < e.impulse_offsets.size(); o++) {
      EXPECT_EQ(a.impulse_offsets[o].rigid_index, e.impulse_offsets[o].rigid_index);
      EXPECT_EQ(a.impulse_offsets[o].local_flag, e.impulse_offsets[o].local_flag);
      expect_float3_equal(e.impulse_offsets[o].velocity, a.impulse_offsets[o].velocity);
      expect_float3_equal(e.impulse_offsets[o].torque, a.impulse_offsets[o].torque);
    }
  }

  ASSERT_EQ(actual.display_frames.size(), expected.display_frames.size());
  for (size_t i = 0; i < expected.display_frames.size(); i++) {
    const PMXDisplayFrame &e = expected.display_frames[i];
    const PMXDisplayFrame &a = actual.display_frames[i];
    SCOPED_TRACE("display frame " + std::to_string(i));
    EXPECT_EQ(a.name_local, e.name_local);
    EXPECT_EQ(a.name_universal, e.name_universal);
    EXPECT_EQ(a.flag, e.flag);
    ASSERT_EQ(a.items.size(), e.items.size());
    for (size_t it = 0; it < e.items.size(); it++) {
      EXPECT_EQ(a.items[it].type, e.items[it].type);
      EXPECT_EQ(a.items[it].index, e.items[it].index);
    }
  }

  ASSERT_EQ(actual.rigid_bodies.size(), expected.rigid_bodies.size());
  for (size_t i = 0; i < expected.rigid_bodies.size(); i++) {
    const PMXRigidBody &e = expected.rigid_bodies[i];
    const PMXRigidBody &a = actual.rigid_bodies[i];
    SCOPED_TRACE("rigid body " + std::to_string(i));
    EXPECT_EQ(a.name_local, e.name_local);
    EXPECT_EQ(a.name_universal, e.name_universal);
    EXPECT_EQ(a.bone_index, e.bone_index);
    EXPECT_EQ(a.collision_group, e.collision_group);
    EXPECT_EQ(a.no_collision_group, e.no_collision_group);
    EXPECT_EQ(a.shape_type, e.shape_type);
    expect_float3_equal(e.shape_size, a.shape_size);
    expect_float3_equal(e.pos, a.pos);
    expect_float3_equal(e.rot, a.rot);
    EXPECT_FLOAT_EQ(a.mass, e.mass);
    EXPECT_FLOAT_EQ(a.linear_damping, e.linear_damping);
    EXPECT_FLOAT_EQ(a.angular_damping, e.angular_damping);
    EXPECT_FLOAT_EQ(a.restitution, e.restitution);
    EXPECT_FLOAT_EQ(a.friction, e.friction);
    EXPECT_EQ(a.physics_type, e.physics_type);
  }

  ASSERT_EQ(actual.joints.size(), expected.joints.size());
  for (size_t i = 0; i < expected.joints.size(); i++) {
    const PMXJoint &e = expected.joints[i];
    const PMXJoint &a = actual.joints[i];
    SCOPED_TRACE("joint " + std::to_string(i));
    EXPECT_EQ(a.name_local, e.name_local);
    EXPECT_EQ(a.name_universal, e.name_universal);
    EXPECT_EQ(a.type, e.type);
    EXPECT_EQ(a.rigid_a_index, e.rigid_a_index);
    EXPECT_EQ(a.rigid_b_index, e.rigid_b_index);
    expect_float3_equal(e.pos, a.pos);
    expect_float3_equal(e.rot, a.rot);
    expect_float3_equal(e.translation_limit_min, a.translation_limit_min);
    expect_float3_equal(e.translation_limit_max, a.translation_limit_max);
    expect_float3_equal(e.rotation_limit_min, a.rotation_limit_min);
    expect_float3_equal(e.rotation_limit_max, a.rotation_limit_max);
    expect_float3_equal(e.spring_translation, a.spring_translation);
    expect_float3_equal(e.spring_rotation, a.spring_rotation);
  }
}

PMXModel write_then_read(const PMXModel &model,
                         const PMXWriteOptions &options,
                         std::vector<uint8_t> *r_buffer = nullptr)
{
  const std::vector<uint8_t> buffer = PMXWriter::write_to_memory(model, options);
  if (r_buffer != nullptr) {
    *r_buffer = buffer;
  }
  return PMXReader::read_from_memory(buffer.data(), buffer.size(), "<memory>");
}

class PMXWriterTest : public testing::Test {};

TEST_F(PMXWriterTest, round_trips_every_section_utf16)
{
  const PMXModel model = make_round_trip_model();
  std::vector<uint8_t> buffer;
  const PMXModel reloaded = write_then_read(model, {}, &buffer);

  expect_models_equal(model, reloaded);
  /* The reader refuses trailing bytes, so a clean parse proves the writer emitted
   * exactly the sections it declared. */
  EXPECT_EQ(reloaded.parse_end_offset, buffer.size());
  EXPECT_EQ(reloaded.file_size, buffer.size());
}

TEST_F(PMXWriterTest, round_trips_every_section_utf8)
{
  PMXModel model = make_round_trip_model();
  model.header.encoding = uint8_t(PMXEncoding::UTF8);

  const PMXModel reloaded = write_then_read(model, {});
  expect_models_equal(model, reloaded);
  EXPECT_TRUE(reloaded.is_utf8());
}

TEST_F(PMXWriterTest, write_read_write_is_byte_identical)
{
  const PMXModel model = make_round_trip_model();
  const std::vector<uint8_t> first = PMXWriter::write_to_memory(model, {});
  const PMXModel reloaded = PMXReader::read_from_memory(first.data(), first.size(), "<memory>");
  const std::vector<uint8_t> second = PMXWriter::write_to_memory(reloaded, {});

  EXPECT_EQ(second, first);
}

TEST_F(PMXWriterTest, recomputes_index_sizes_from_section_counts)
{
  const PMXModel model = make_round_trip_model();
  const PMXModel reloaded = write_then_read(model, {});

  /* Every section in the fixture is small, so all index widths collapse to one byte. */
  EXPECT_EQ(reloaded.header.vertex_idx_size, 1);
  EXPECT_EQ(reloaded.header.texture_idx_size, 1);
  EXPECT_EQ(reloaded.header.material_idx_size, 1);
  EXPECT_EQ(reloaded.header.bone_idx_size, 1);
  EXPECT_EQ(reloaded.header.morph_idx_size, 1);
  EXPECT_EQ(reloaded.header.rigid_idx_size, 1);
}

TEST_F(PMXWriterTest, honors_existing_header_when_not_recomputing)
{
  PMXModel model = make_round_trip_model();
  model.header.vertex_idx_size = 4;
  model.header.texture_idx_size = 2;
  model.header.material_idx_size = 2;
  model.header.bone_idx_size = 4;
  model.header.morph_idx_size = 2;
  model.header.rigid_idx_size = 4;

  PMXWriteOptions options;
  options.recompute_index_sizes = false;
  const PMXModel reloaded = write_then_read(model, options);

  EXPECT_EQ(reloaded.header.vertex_idx_size, 4);
  EXPECT_EQ(reloaded.header.texture_idx_size, 2);
  EXPECT_EQ(reloaded.header.material_idx_size, 2);
  EXPECT_EQ(reloaded.header.bone_idx_size, 4);
  EXPECT_EQ(reloaded.header.morph_idx_size, 2);
  EXPECT_EQ(reloaded.header.rigid_idx_size, 4);
  expect_models_equal(model, reloaded);
}

TEST_F(PMXWriterTest, rejects_unset_index_sizes_when_not_recomputing)
{
  PMXModel model = make_round_trip_model();
  PMXWriteOptions options;
  options.recompute_index_sizes = false;
  /* The fixture deliberately leaves index sizes at zero, which is not encodable. */
  EXPECT_THROW(PMXWriter::write_to_memory(model, options), PMXWriterError);
}

TEST_F(PMXWriterTest, defaults_version_for_authored_models)
{
  PMXModel model;
  model.header.encoding = uint8_t(PMXEncoding::UTF8);
  model.name_local = "authored";

  const PMXModel reloaded = write_then_read(model, {});
  EXPECT_FLOAT_EQ(reloaded.header.version, 2.0f);
  EXPECT_EQ(reloaded.header.header_size, 8);
  EXPECT_EQ(reloaded.name_local, "authored");
  EXPECT_TRUE(reloaded.vertices.empty());
  EXPECT_TRUE(reloaded.bones.empty());
}

TEST_F(PMXWriterTest, header_additional_uv_count_is_authoritative)
{
  PMXModel model = make_round_trip_model();
  /* Vertices still carry two populated additional UV sets; the header says none,
   * so none are written and the reload reports none. This mirrors how the reader
   * derives each vertex's count from the header. */
  model.header.add_uv_cnt = 0;

  const PMXModel reloaded = write_then_read(model, {});
  ASSERT_EQ(reloaded.vertices.size(), model.vertices.size());
  for (const PMXVertex &vertex : reloaded.vertices) {
    EXPECT_EQ(vertex.additional_uv_count, 0);
    for (int uv = 0; uv < 4; uv++) {
      for (int c = 0; c < 4; c++) {
        EXPECT_FLOAT_EQ(vertex.additional_uv[uv][c], 0.0f);
      }
    }
  }
}

TEST_F(PMXWriterTest, minimum_index_size_matches_mmd_tools_thresholds)
{
  /* Unsigned (vertex indices). */
  EXPECT_EQ(PMXWriter::minimum_index_size(0, false), 1);
  EXPECT_EQ(PMXWriter::minimum_index_size(255, false), 1);
  EXPECT_EQ(PMXWriter::minimum_index_size(256, false), 2);
  EXPECT_EQ(PMXWriter::minimum_index_size(65535, false), 2);
  EXPECT_EQ(PMXWriter::minimum_index_size(65536, false), 4);

  /* Signed (every other index kind). */
  EXPECT_EQ(PMXWriter::minimum_index_size(0, true), 1);
  EXPECT_EQ(PMXWriter::minimum_index_size(127, true), 1);
  EXPECT_EQ(PMXWriter::minimum_index_size(128, true), 2);
  EXPECT_EQ(PMXWriter::minimum_index_size(32767, true), 2);
  EXPECT_EQ(PMXWriter::minimum_index_size(32768, true), 4);
}

TEST_F(PMXWriterTest, rejects_non_finite_floats)
{
  PMXModel model = make_round_trip_model();
  model.vertices[0].pos[0] = std::numeric_limits<float>::infinity();
  EXPECT_THROW(PMXWriter::write_to_memory(model, {}), PMXWriterError);

  PMXModel nan_model = make_round_trip_model();
  nan_model.joints[0].spring_rotation[2] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(PMXWriter::write_to_memory(nan_model, {}), PMXWriterError);
}

TEST_F(PMXWriterTest, rejects_index_that_does_not_fit_declared_width)
{
  PMXModel model = make_round_trip_model();
  model.header.vertex_idx_size = 1;
  model.header.texture_idx_size = 1;
  model.header.material_idx_size = 1;
  model.header.bone_idx_size = 1;
  model.header.morph_idx_size = 1;
  model.header.rigid_idx_size = 1;
  /* 200 exceeds the signed one-byte range the header declares for bone indices. */
  model.vertices[0].bone_indices[0] = 200;

  PMXWriteOptions options;
  options.recompute_index_sizes = false;
  EXPECT_THROW(PMXWriter::write_to_memory(model, options), PMXWriterError);
}

TEST_F(PMXWriterTest, rejects_invalid_utf8_when_encoding_utf16)
{
  PMXModel model = make_round_trip_model();
  model.name_local = "\xFF\xFE";
  EXPECT_THROW(PMXWriter::write_to_memory(model, {}), PMXWriterError);

  PMXModel truncated = make_round_trip_model();
  truncated.materials[0].memo = "\xE3\x83"; /* Truncated three-byte sequence. */
  EXPECT_THROW(PMXWriter::write_to_memory(truncated, {}), PMXWriterError);
}

TEST_F(PMXWriterTest, rejects_weight_arrays_that_are_too_short)
{
  PMXModel model = make_round_trip_model();
  model.vertices[2].bone_indices.resize(2); /* BDEF4 needs four. */
  EXPECT_THROW(PMXWriter::write_to_memory(model, {}), PMXWriterError);
}

TEST_F(PMXWriterTest, rejects_invalid_enumerations)
{
  PMXModel bad_sphere = make_round_trip_model();
  bad_sphere.materials[0].sphere_mode = SphereMode(9);
  EXPECT_THROW(PMXWriter::write_to_memory(bad_sphere, {}), PMXWriterError);

  PMXModel bad_toon = make_round_trip_model();
  bad_toon.materials[0].toon_flag = 5;
  EXPECT_THROW(PMXWriter::write_to_memory(bad_toon, {}), PMXWriterError);

  PMXModel bad_shape = make_round_trip_model();
  bad_shape.rigid_bodies[0].shape_type = 7;
  EXPECT_THROW(PMXWriter::write_to_memory(bad_shape, {}), PMXWriterError);

  PMXModel bad_joint = make_round_trip_model();
  bad_joint.joints[0].type = 1;
  EXPECT_THROW(PMXWriter::write_to_memory(bad_joint, {}), PMXWriterError);

  PMXModel bad_frame = make_round_trip_model();
  bad_frame.display_frames[0].flag = 3;
  EXPECT_THROW(PMXWriter::write_to_memory(bad_frame, {}), PMXWriterError);

  PMXModel bad_encoding = make_round_trip_model();
  bad_encoding.header.encoding = 4;
  EXPECT_THROW(PMXWriter::write_to_memory(bad_encoding, {}), PMXWriterError);

  PMXModel bad_uv_count = make_round_trip_model();
  bad_uv_count.header.add_uv_cnt = 5;
  EXPECT_THROW(PMXWriter::write_to_memory(bad_uv_count, {}), PMXWriterError);
}

TEST_F(PMXWriterTest, writes_file_readable_by_the_file_entry_point)
{
  const PMXModel model = make_round_trip_model();
  char temp_dir[FILE_MAX];
  BLI_temp_directory_path_get(temp_dir, sizeof(temp_dir));
  const std::string filepath = std::string(temp_dir) + "pmx_writer_round_trip.pmx";

  PMXWriter::write(model, filepath);
  const PMXModel reloaded = PMXReader::read(filepath);
  expect_models_equal(model, reloaded);

  BLI_delete(filepath.c_str(), false, false);
}

}  // namespace
}  // namespace blender::io::pmx::tests
