/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "testing/testing.h"

#include "intern/pmx_model_diff.h"
#include "intern/pmx_types.h"

#include <string>
#include <utility>

namespace blender::io::pmx::tests {
namespace {

/**
 * A model that touches every section and every variable-length branch, so a
 * comparison that silently skips a field shows up as a false "equal".
 *
 * Every struct is `{}`-initialized on purpose: `PMXMaterial` / `PMXBone` /
 * `PMXMorph` hold `std::string`, so they are not trivially default
 * constructible and their POD members would otherwise keep whatever was on the
 * stack. `PMXReader` zeroes them, so a test model that does not is not
 * representative of anything the comparator will ever see.
 */
PMXModel make_base_model()
{
  PMXModel model;
  model.header.signature[0] = 'P';
  model.header.signature[1] = 'M';
  model.header.signature[2] = 'X';
  model.header.signature[3] = ' ';
  model.header.version = 2.1f;
  model.header.header_size = 8;
  model.header.encoding = 0;
  model.header.add_uv_cnt = 2;
  model.header.vertex_idx_size = 2;
  model.header.texture_idx_size = 1;
  model.header.material_idx_size = 1;
  model.header.bone_idx_size = 1;
  model.header.morph_idx_size = 1;
  model.header.rigid_idx_size = 1;

  model.name_local = "name local";
  model.name_universal = "name universal";
  model.comment_local = "comment local";
  model.comment_universal = "comment universal";

  /* Asymmetric pos / normal / uv so an axis permutation or a V-flip cannot
   * coincidentally compare equal. */
  const auto add_vertex = [&](const BoneWeightType type) {
    PMXVertex v{};
    v.additional_uv_count = 2;
    v.pos[0] = 1.0f;
    v.pos[1] = 2.0f;
    v.pos[2] = 3.0f;
    v.normal[0] = 0.1f;
    v.normal[1] = 0.2f;
    v.normal[2] = 0.3f;
    v.uv[0] = 0.25f;
    v.uv[1] = 0.75f;
    v.additional_uv[0] = {0.1f, 0.2f, 0.3f, 0.4f};
    v.additional_uv[1] = {0.5f, 0.6f, 0.7f, 0.8f};
    v.weight_type = type;
    v.edge_factor = 1.0f;
    switch (type) {
      case BoneWeightType::BDEF1:
        v.bone_indices = {0};
        v.bone_weights = {1.0f};
        break;
      case BoneWeightType::BDEF2:
      case BoneWeightType::SDEF:
        v.bone_indices = {0, 1};
        v.bone_weights = {0.75f, 1.0f - 0.75f};
        break;
      case BoneWeightType::BDEF4:
      case BoneWeightType::QDEF:
        v.bone_indices = {0, 1, 2, 0};
        v.bone_weights = {0.4f, 0.3f, 0.2f, 0.1f};
        break;
    }
    if (type == BoneWeightType::SDEF) {
      v.sdef_c[0] = 1.0f;
      v.sdef_c[1] = 2.0f;
      v.sdef_c[2] = 3.0f;
      v.sdef_r0[0] = 4.0f;
      v.sdef_r0[1] = 5.0f;
      v.sdef_r0[2] = 6.0f;
      v.sdef_r1[0] = 7.0f;
      v.sdef_r1[1] = 8.0f;
      v.sdef_r1[2] = 9.0f;
    }
    model.vertices.push_back(v);
  };
  add_vertex(BoneWeightType::BDEF1);
  add_vertex(BoneWeightType::BDEF2);
  add_vertex(BoneWeightType::BDEF4);
  add_vertex(BoneWeightType::SDEF);
  add_vertex(BoneWeightType::QDEF);
  add_vertex(BoneWeightType::BDEF1);

  model.face_indices = {0, 1, 2, 3, 4, 5};

  model.textures.push_back({"tex\\base.png"});
  model.textures.push_back({"tex\\toon.png"});

  PMXMaterial mat0{};
  mat0.name_local = "Mat0";
  mat0.name_universal = "Mat0 EN";
  mat0.diffuse[0] = 0.1f;
  mat0.diffuse[3] = 1.0f;
  mat0.specular[0] = 0.4f;
  mat0.specular_power = 12.5f;
  mat0.ambient[0] = 0.7f;
  mat0.flag = PMX_MATERIAL_FLAG_DOUBLE_SIDED | PMX_MATERIAL_FLAG_EDGE;
  mat0.edge_color[3] = 1.0f;
  mat0.edge_size = 1.25f;
  mat0.texture_idx = 0;
  mat0.sphere_texture_idx = 1;
  mat0.sphere_mode = SphereMode::Sphere;
  mat0.toon_flag = 0;
  mat0.toon_internal_value = 3;
  mat0.toon_texture_idx = -1;
  mat0.memo = "memo 0";
  mat0.face_vertex_count = 3;
  model.materials.push_back(mat0);

  PMXMaterial mat1{};
  mat1.name_local = "Mat1";
  mat1.diffuse[3] = 1.0f;
  mat1.flag = PMX_MATERIAL_FLAG_SELF_SHADOW;
  mat1.texture_idx = -1;
  mat1.sphere_texture_idx = -1;
  mat1.sphere_mode = SphereMode::None;
  mat1.toon_flag = 1;
  mat1.toon_texture_idx = 1;
  mat1.face_vertex_count = 3;
  model.materials.push_back(mat1);

  PMXBone bone0{};
  bone0.name_local = "Root";
  bone0.name_universal = "Root EN";
  bone0.parent_index = -1;
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
  bone2.ik_links.push_back(link1);
  model.bones.push_back(bone2);

  PMXMorph vertex_morph{};
  vertex_morph.name_local = "VertexMorph";
  vertex_morph.panel = 1;
  vertex_morph.type = MorphType::Vertex;
  vertex_morph.vertex_offsets.push_back({0, {1.0f, 2.0f, 3.0f}});
  model.morphs.push_back(vertex_morph);

  PMXMorph group_morph{};
  group_morph.name_local = "GroupMorph";
  group_morph.panel = 2;
  group_morph.type = MorphType::Group;
  group_morph.group_offsets.push_back({0, 0.5f});
  model.morphs.push_back(group_morph);

  PMXMorph bone_morph{};
  bone_morph.name_local = "BoneMorph";
  bone_morph.type = MorphType::Bone;
  bone_morph.bone_offsets.push_back({1, {0.1f, 0.2f, 0.3f}, {0.4f, 0.5f, 0.6f, 0.7f}});
  model.morphs.push_back(bone_morph);

  PMXMorph uv_morph{};
  uv_morph.name_local = "UVMorph";
  uv_morph.type = MorphType::UV_2nd;
  uv_morph.uv_offsets.push_back({1, {0.11f, 0.22f, 0.33f, 0.44f}});
  model.morphs.push_back(uv_morph);

  PMXMorph material_morph{};
  material_morph.name_local = "MaterialMorph";
  material_morph.type = MorphType::Material;
  PMXMaterialMorphOffset material_offset{};
  material_offset.material_index = -1;
  material_offset.calc_mode = 1;
  material_offset.diffuse[0] = 0.9f;
  material_offset.specular_power = 4.5f;
  material_offset.edge_size = 2.5f;
  material_offset.texture_factor[0] = 0.5f;
  material_offset.sphere_texture_factor[1] = 0.4f;
  material_offset.toon_texture_factor[2] = 0.3f;
  material_morph.material_offsets.push_back(material_offset);
  model.morphs.push_back(material_morph);

  PMXMorph impulse_morph{};
  impulse_morph.name_local = "ImpulseMorph";
  impulse_morph.type = MorphType::Impulse;
  PMXImpulseMorphOffset impulse_offset{};
  impulse_offset.rigid_index = 0;
  impulse_offset.local_flag = 1;
  impulse_offset.velocity[0] = 1.5f;
  impulse_offset.torque[2] = 2.5f;
  impulse_morph.impulse_offsets.push_back(impulse_offset);
  model.morphs.push_back(impulse_morph);

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

  PMXRigidBody rigid{};
  rigid.name_local = "Rigid";
  rigid.name_universal = "Rigid EN";
  rigid.bone_index = 0;
  rigid.collision_group = 2;
  rigid.no_collision_group = 0x00FF;
  rigid.shape_type = 2;
  rigid.shape_size[0] = 0.5f;
  rigid.shape_size[1] = 1.5f;
  rigid.shape_size[2] = 2.5f;
  rigid.pos[0] = 1.0f;
  rigid.pos[1] = 2.0f;
  rigid.pos[2] = 3.0f;
  rigid.rot[0] = 0.1f;
  rigid.rot[1] = 0.2f;
  rigid.rot[2] = 0.3f;
  rigid.mass = 1.25f;
  rigid.linear_damping = 0.5f;
  rigid.angular_damping = 0.25f;
  rigid.restitution = 0.125f;
  rigid.friction = 0.75f;
  rigid.physics_type = 1;
  model.rigid_bodies.push_back(rigid);

  PMXRigidBody rigid_b = rigid;
  rigid_b.name_local = "Rigid B";
  model.rigid_bodies.push_back(rigid_b);

  PMXJoint joint{};
  joint.name_local = "Joint";
  joint.name_universal = "Joint EN";
  joint.type = 0;
  joint.rigid_a_index = 0;
  joint.rigid_b_index = 1;
  joint.pos[0] = 1.0f;
  joint.pos[1] = 2.0f;
  joint.pos[2] = 3.0f;
  joint.rot[0] = 0.1f;
  joint.rot[1] = 0.2f;
  joint.rot[2] = 0.3f;
  joint.translation_limit_min[0] = -1.0f;
  joint.translation_limit_max[0] = 1.0f;
  joint.rotation_limit_min[1] = -0.5f;
  joint.rotation_limit_max[1] = 0.5f;
  joint.spring_translation[0] = 10.0f;
  joint.spring_rotation[2] = 20.0f;
  model.joints.push_back(joint);

  model.file_size = 4096;
  model.parse_end_offset = 4096;
  return model;
}

/** True when any reported issue path starts with `prefix`. */
bool has_issue_under(const PMXModelDiffReport &report, const std::string &prefix)
{
  for (const PMXModelDiffIssue &issue : report.issues) {
    if (issue.path.compare(0, prefix.size(), prefix) == 0) {
      return true;
    }
  }
  return false;
}

int section_issue_count(const PMXModelDiffReport &report, const std::string &name)
{
  for (const PMXModelDiffSection &section : report.sections) {
    if (section.name == name) {
      return section.issue_count;
    }
  }
  return -1;
}

class PMXModelDiffTest : public ::testing::Test {};

/* --- The property everything else depends on ------------------------------- */

TEST_F(PMXModelDiffTest, identical_models_compare_equal)
{
  const PMXModel model = make_base_model();
  const PMXModelDiffReport report = diff_pmx_models(model, model);
  EXPECT_TRUE(report.equal()) << report.to_string();
  EXPECT_EQ(report.total_issues, 0);
  EXPECT_TRUE(report.to_string().empty());
}

/* --- The seven inversions the exporter has to get right --------------------- */

TEST_F(PMXModelDiffTest, catches_position_scale_error)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* A missing divide-by-scale: 0.08 import scale applied twice. */
  for (PMXVertex &v : actual.vertices) {
    v.pos[0] *= 0.08f;
    v.pos[1] *= 0.08f;
    v.pos[2] *= 0.08f;
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[0].pos"));
}

TEST_F(PMXModelDiffTest, catches_position_axis_permutation)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* Forgetting to invert the PMX (x,y,z) -> Blender (x,z,y) mapping. */
  for (PMXVertex &v : actual.vertices) {
    std::swap(v.pos[1], v.pos[2]);
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[0].pos"));
}

TEST_F(PMXModelDiffTest, catches_normal_axis_permutation)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  for (PMXVertex &v : actual.vertices) {
    std::swap(v.normal[1], v.normal[2]);
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[0].normal"));
}

TEST_F(PMXModelDiffTest, catches_uv_v_flip)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* Import does `1 - v`; export must undo it. */
  for (PMXVertex &v : actual.vertices) {
    v.uv[1] = 1.0f - v.uv[1];
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[0].uv"));
}

TEST_F(PMXModelDiffTest, catches_reversed_triangle_winding)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* Import reverses each triangle; a missing re-reverse flips every face. */
  for (size_t face = 0; face * 3 + 2 < actual.face_indices.size(); face++) {
    std::swap(actual.face_indices[face * 3], actual.face_indices[face * 3 + 2]);
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "faces["));
}

TEST_F(PMXModelDiffTest, catches_sdef_frame_error)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  for (PMXVertex &v : actual.vertices) {
    if (v.weight_type != BoneWeightType::SDEF) {
      continue;
    }
    std::swap(v.sdef_c[1], v.sdef_c[2]);
    v.sdef_r0[0] *= 0.08f;
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[3].sdef_c"));
  EXPECT_TRUE(has_issue_under(report, "vertices[3].sdef_r0"));
}

TEST_F(PMXModelDiffTest, catches_physics_space_error)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* mmd_physics_definition stores Blender-space values; export must invert. */
  for (PMXRigidBody &rigid : actual.rigid_bodies) {
    std::swap(rigid.pos[1], rigid.pos[2]);
  }
  for (PMXJoint &joint : actual.joints) {
    joint.pos[0] *= 0.08f;
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "rigid_bodies[0].pos"));
  EXPECT_TRUE(has_issue_under(report, "joints[0].pos"));
}

/* --- Retention-backed sections --------------------------------------------- */

TEST_F(PMXModelDiffTest, catches_header_and_model_info_loss)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.header.version = 2.0f;
  actual.header.encoding = 1;
  actual.header.add_uv_cnt = 0;
  actual.name_universal.clear();
  actual.comment_local = "changed";
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "header.version"));
  EXPECT_TRUE(has_issue_under(report, "header.encoding"));
  EXPECT_TRUE(has_issue_under(report, "header.add_uv_cnt"));
  EXPECT_TRUE(has_issue_under(report, "model_info.name_universal"));
  EXPECT_TRUE(has_issue_under(report, "model_info.comment_local"));
}

TEST_F(PMXModelDiffTest, catches_texture_and_material_loss)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.textures[0].path = "tex/base.png"; /* separator changed */
  actual.materials[0].specular_power = 0.0f;
  actual.materials[0].sphere_mode = SphereMode::None;
  actual.materials[0].toon_internal_value = 0;
  actual.materials[0].memo.clear();
  actual.materials[1].toon_flag = 0;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "textures[0].path"));
  EXPECT_TRUE(has_issue_under(report, "materials[0].specular_power"));
  EXPECT_TRUE(has_issue_under(report, "materials[0].sphere_mode"));
  EXPECT_TRUE(has_issue_under(report, "materials[0].toon_internal_value"));
  EXPECT_TRUE(has_issue_under(report, "materials[0].memo"));
  EXPECT_TRUE(has_issue_under(report, "materials[1].toon_flag"));
}

TEST_F(PMXModelDiffTest, catches_bone_metadata_loss)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.bones[1].transform_order = 0;
  actual.bones[1].flag &= uint16_t(~BONE_FLAG_PHYSICS_AFTER_DEF);
  actual.bones[1].inherit_parent_ratio = 0.75f; /* sign dropped */
  actual.bones[1].local_z[2] = 0.0f;
  actual.bones[1].external_parent_index = -1;
  actual.bones[2].ik_loop_count = 0;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "bones[1].transform_order"));
  EXPECT_TRUE(has_issue_under(report, "bones[1].flag"));
  EXPECT_TRUE(has_issue_under(report, "bones[1].inherit_parent_ratio"));
  EXPECT_TRUE(has_issue_under(report, "bones[1].local_z"));
  EXPECT_TRUE(has_issue_under(report, "bones[1].external_parent_index"));
  EXPECT_TRUE(has_issue_under(report, "bones[2].ik_loop_count"));
}

TEST_F(PMXModelDiffTest, catches_ik_link_loss)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.bones[2].ik_links[0].limit_angle = false;
  actual.bones[2].ik_links.pop_back();
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_GT(section_issue_count(report, "bones"), 0);
}

TEST_F(PMXModelDiffTest, catches_morph_loss_for_every_kind)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.morphs[0].vertex_offsets[0].offset[1] = 0.0f;
  actual.morphs[1].group_offsets[0].influence = 1.0f;
  actual.morphs[2].bone_offsets[0].rot[3] = 0.0f;
  actual.morphs[3].uv_offsets[0].offset[2] = 0.0f;
  actual.morphs[4].material_offsets[0].calc_mode = 0;
  actual.morphs[5].impulse_offsets[0].torque[2] = 0.0f;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_EQ(section_issue_count(report, "morphs"), 6);
}

TEST_F(PMXModelDiffTest, catches_morph_panel_and_type_loss)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.morphs[0].panel = 0;
  actual.morphs[3].type = MorphType::UV;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "morphs[0].panel"));
  EXPECT_TRUE(has_issue_under(report, "morphs[3].type"));
}

TEST_F(PMXModelDiffTest, catches_display_frame_loss)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.display_frames[0].items[1].index = 1;
  actual.display_frames[1].flag = 0;
  actual.display_frames[1].name_local = "changed";
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_EQ(section_issue_count(report, "display_frames"), 3);
}

TEST_F(PMXModelDiffTest, catches_additional_uv_loss)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.vertices[0].additional_uv[1] = {0.0f, 0.0f, 0.0f, 0.0f};
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[0].additional_uv"));
}

TEST_F(PMXModelDiffTest, catches_weight_type_and_weight_loss)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.vertices[3].weight_type = BoneWeightType::BDEF2;
  actual.vertices[2].bone_weights[1] = 0.0f;
  actual.vertices[2].bone_indices[3] = 1;
  actual.vertices[0].edge_factor = 0.5f;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[3].weight_type"));
  EXPECT_TRUE(has_issue_under(report, "vertices[2].influences"));
  EXPECT_TRUE(has_issue_under(report, "vertices[0].edge_factor"));
}

/* --- Skinning canonicalization ---------------------------------------------
 *
 * Skinning is compared as a bone -> total-weight map, not slot by slot. Two PMX
 * weight arrays can describe the identical deformation without agreeing slot for
 * slot, and both ways that happens are properties of real files (measured by
 * `PMXRoundTripTest.reports_skinning_slots_blender_cannot_represent`). These
 * four tests pin the rule down from both directions: the equivalences must not
 * be reported, and the real errors must still be.
 */

TEST_F(PMXModelDiffTest, treats_collapsed_duplicate_bone_slots_as_equal)
{
  PMXModel expected = make_base_model();
  /* The same bone twice, as a PMX file may legally store it. */
  expected.vertices[2].bone_indices = {5, 5, 0, 0};
  expected.vertices[2].bone_weights = {0.6f, 0.4f, 0.0f, 0.0f};

  PMXModel actual = expected;
  /* What import -> export produces: a Blender vertex group cannot hold the
   * duplicate, so the two slots arrive accumulated into one. */
  actual.vertices[2].bone_indices = {5, 0, 0, 0};
  actual.vertices[2].bone_weights = {1.0f, 0.0f, 0.0f, 0.0f};

  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_TRUE(report.equal()) << report.to_string();
}

TEST_F(PMXModelDiffTest, ignores_bone_index_in_zero_weight_slot)
{
  PMXModel expected = make_base_model();
  expected.vertices[2].bone_indices = {3, 7, 9, 11};
  expected.vertices[2].bone_weights = {1.0f, 0.0f, 0.0f, 0.0f};

  PMXModel actual = expected;
  /* Import drops zero-weight slots, so export cannot recover 7/9/11 and pads
   * with 0. Those slots deform nothing, so this must not be a difference. */
  actual.vertices[2].bone_indices = {3, 0, 0, 0};

  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_TRUE(report.equal()) << report.to_string();
}

TEST_F(PMXModelDiffTest, catches_weight_moved_to_a_different_bone)
{
  PMXModel expected = make_base_model();
  expected.vertices[2].bone_indices = {3, 4, 0, 0};
  expected.vertices[2].bone_weights = {0.6f, 0.4f, 0.0f, 0.0f};

  PMXModel actual = expected;
  /* Same magnitudes, wrong bone: the exact error summing must not hide. */
  actual.vertices[2].bone_indices = {3, 5, 0, 0};

  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[2].influences"));
}

TEST_F(PMXModelDiffTest, catches_total_weight_change_across_duplicate_slots)
{
  PMXModel expected = make_base_model();
  expected.vertices[2].bone_indices = {5, 5, 0, 0};
  expected.vertices[2].bone_weights = {0.6f, 0.4f, 0.0f, 0.0f};

  PMXModel actual = expected;
  /* Still duplicate slots on bone 5, but the total is no longer 1.0. Summing
   * must not turn a magnitude change into a pass. */
  actual.vertices[2].bone_weights = {0.6f, 0.3f, 0.0f, 0.0f};

  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[2].influences"));
}

/* --- Count handling -------------------------------------------------------- */

TEST_F(PMXModelDiffTest, reports_count_mismatch_and_still_compares_prefix)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.vertices.pop_back();
  actual.vertices[0].pos[0] = 99.0f;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  /* A count error must not mask field errors in the common prefix. */
  EXPECT_TRUE(has_issue_under(report, "vertices.count"));
  EXPECT_TRUE(has_issue_under(report, "vertices[0].pos"));
}

TEST_F(PMXModelDiffTest, reports_count_mismatch_for_every_section)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.face_indices.resize(3);
  actual.textures.pop_back();
  actual.materials.pop_back();
  actual.bones.pop_back();
  actual.morphs.pop_back();
  actual.display_frames.pop_back();
  actual.rigid_bodies.pop_back();
  actual.joints.pop_back();
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "faces.count"));
  EXPECT_TRUE(has_issue_under(report, "textures.count"));
  EXPECT_TRUE(has_issue_under(report, "materials.count"));
  EXPECT_TRUE(has_issue_under(report, "bones.count"));
  EXPECT_TRUE(has_issue_under(report, "morphs.count"));
  EXPECT_TRUE(has_issue_under(report, "display_frames.count"));
  EXPECT_TRUE(has_issue_under(report, "rigid_bodies.count"));
  EXPECT_TRUE(has_issue_under(report, "joints.count"));
}

/* --- Tolerances ------------------------------------------------------------ */

TEST_F(PMXModelDiffTest, accepts_geometry_within_tolerance)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* The scale round-trip's residual, well inside geometry_tolerance. */
  for (PMXVertex &v : actual.vertices) {
    v.pos[0] += 1.0e-5f;
    v.pos[2] -= 1.0e-5f;
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_TRUE(report.equal()) << report.to_string();
}

TEST_F(PMXModelDiffTest, rejects_geometry_beyond_tolerance)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  for (PMXVertex &v : actual.vertices) {
    v.pos[0] += 1.0e-2f;
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[0].pos"));
}

TEST_F(PMXModelDiffTest, metadata_tolerance_is_exact_by_default)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* Retention stores float32 in float32; any drift means a lost field. */
  actual.materials[0].edge_size += 1.0e-6f;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "materials[0].edge_size"));
}

TEST_F(PMXModelDiffTest, unit_tolerance_applies_to_uv)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  for (PMXVertex &v : actual.vertices) {
    v.uv[0] += 1.0e-7f;
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_TRUE(report.equal()) << report.to_string();
}

/* Normals get their own, much looser budget than every other unscaled value,
 * because import stores them as a `short2` in the corner tangent frame rather
 * than keeping the `float3`. See `PMXNormalFidelityTest` for the measurement
 * that sets `normal_tolerance`. */
TEST_F(PMXModelDiffTest, normal_tolerance_covers_blender_encoding_loss)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* 5e-3 sits inside the measured worst case of that encode/decode round-trip,
   * yet is 500x above `unit_tolerance`. Regression guard: while normals were
   * compared against `unit_tolerance` this produced one false difference per
   * vertex -- 62896 of them on a real model, which buried the real findings. */
  for (PMXVertex &v : actual.vertices) {
    v.normal[0] += 5.0e-3f;
  }
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_TRUE(report.equal()) << report.to_string();
}

TEST_F(PMXModelDiffTest, rejects_normal_error_beyond_encoding_loss)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* 5x the tolerance, and still two orders of magnitude below what an axis
   * permutation does: the looser budget must not need a gross error to fire. */
  actual.vertices[0].normal[0] += 0.05f;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "vertices[0].normal"));
}

/* Vertex Morph offsets are the one section the exporter rebuilds from Shape
 * Keys rather than from retention, so losing them is a live failure mode. */
TEST_F(PMXModelDiffTest, reports_dropped_vertex_morph_offsets)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* Regression guard: these offsets were once compared by taking min(a, b) and
   * walking the common prefix, so an empty offset list compared *equal* and
   * hid 119 morphs' worth of lost data on a real model. Every lost offset must
   * be reported individually, not summarized as one count difference. */
  actual.morphs[0].vertex_offsets.clear();
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "morphs[0].vertex_offsets"));
}

/* --- Vertex Morph offsets: the order-independent rule ----------------------
 *
 * Export rebuilds these from Shape Keys, which are indexed by vertex and cannot
 * carry the source file's offset order. The four tests below pin both halves of
 * the rule: the permutations and sums it must tolerate, and the real losses it
 * must still catch. Measured justification is in
 * `PMXRoundTripTest.reports_vertex_morph_offsets_blender_cannot_represent`. */

TEST_F(PMXModelDiffTest, treats_reordered_vertex_morph_offsets_as_equal)
{
  PMXModel expected = make_base_model();
  expected.morphs[0].vertex_offsets.clear();
  expected.morphs[0].vertex_offsets.push_back({2, {7.0f, 8.0f, 9.0f}});
  expected.morphs[0].vertex_offsets.push_back({0, {1.0f, 2.0f, 3.0f}});

  PMXModel actual = expected;
  /* What the exporter actually emits: the same offsets, ascending by vertex. */
  actual.morphs[0].vertex_offsets.clear();
  actual.morphs[0].vertex_offsets.push_back({0, {1.0f, 2.0f, 3.0f}});
  actual.morphs[0].vertex_offsets.push_back({2, {7.0f, 8.0f, 9.0f}});

  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_TRUE(report.equal()) << report.to_string();
}

TEST_F(PMXModelDiffTest, treats_summed_duplicate_vertex_offsets_as_equal)
{
  PMXModel expected = make_base_model();
  expected.morphs[0].vertex_offsets.clear();
  /* Legal PMX: one vertex listed twice. Import accumulates, so export can only
   * emit the total. */
  expected.morphs[0].vertex_offsets.push_back({0, {1.0f, 2.0f, 3.0f}});
  expected.morphs[0].vertex_offsets.push_back({0, {0.5f, 0.5f, 0.5f}});

  PMXModel actual = expected;
  actual.morphs[0].vertex_offsets.clear();
  actual.morphs[0].vertex_offsets.push_back({0, {1.5f, 2.5f, 3.5f}});

  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_TRUE(report.equal()) << report.to_string();
}

TEST_F(PMXModelDiffTest, ignores_explicitly_zero_vertex_morph_offset)
{
  PMXModel expected = make_base_model();
  /* A zero offset displaces nothing, and a Shape Key cannot distinguish it from
   * a vertex the Morph never touched, so export legitimately omits it. */
  expected.morphs[0].vertex_offsets.push_back({1, {0.0f, 0.0f, 0.0f}});

  PMXModel actual = make_base_model();

  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_TRUE(report.equal()) << report.to_string();
}

TEST_F(PMXModelDiffTest, catches_vertex_morph_offset_moved_to_another_vertex)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* Same offset, same count, different vertex: a broken identity map. Order
   * independence must not make this invisible. */
  actual.morphs[0].vertex_offsets[0].vertex_index = 2;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_FALSE(report.equal());
  EXPECT_TRUE(has_issue_under(report, "morphs[0].vertex_offsets"));
  /* One absent, one gained. */
  EXPECT_EQ(section_issue_count(report, "morphs"), 2);
}

/* --- Deliberate skips: lock them in so nobody "fixes" them ----------------- */

TEST_F(PMXModelDiffTest, ignores_index_sizes_and_format_constants)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* The writer recomputes the narrowest index width that fits. That is not a
   * semantic difference, so it must not fail the round-trip. */
  actual.header.vertex_idx_size = 4;
  actual.header.texture_idx_size = 4;
  actual.header.material_idx_size = 4;
  actual.header.bone_idx_size = 4;
  actual.header.morph_idx_size = 4;
  actual.header.rigid_idx_size = 4;
  actual.header.header_size = 8;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_TRUE(report.equal()) << report.to_string();
}

TEST_F(PMXModelDiffTest, ignores_parse_diagnostics)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  /* Two files with the same content can differ in size purely from index
   * widths, so these are diagnostics rather than model data. */
  actual.file_size = 12345;
  actual.parse_end_offset = 12345;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  EXPECT_TRUE(report.equal()) << report.to_string();
}

/* --- Reporting ------------------------------------------------------------- */

TEST_F(PMXModelDiffTest, caps_recorded_issues_but_keeps_counting)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  for (PMXVertex &v : actual.vertices) {
    v.pos[0] += 1.0f;
    v.pos[1] += 1.0f;
    v.pos[2] += 1.0f;
    v.normal[0] += 1.0f;
  }
  PMXModelDiffOptions options;
  options.max_issues_per_section = 2;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual, options);
  EXPECT_FALSE(report.equal());
  const int count = section_issue_count(report, "vertices");
  EXPECT_GT(count, 2);
  int reported = 0;
  for (const PMXModelDiffSection &section : report.sections) {
    if (section.name == "vertices") {
      reported = section.reported_count;
    }
  }
  EXPECT_EQ(reported, 2);
  EXPECT_EQ(int(report.issues.size()), reported);
  /* total_issues counts everything, not just what was recorded. */
  EXPECT_EQ(report.total_issues, count);
}

TEST_F(PMXModelDiffTest, summary_names_the_failing_section_and_field)
{
  const PMXModel expected = make_base_model();
  PMXModel actual = expected;
  actual.materials[1].face_vertex_count = 999;
  const PMXModelDiffReport report = diff_pmx_models(expected, actual);
  ASSERT_FALSE(report.equal());
  const std::string summary = report.to_string();
  EXPECT_NE(summary.find("materials"), std::string::npos);
  EXPECT_NE(summary.find("face_vertex_count"), std::string::npos);
  EXPECT_NE(summary.find("999"), std::string::npos);
}

}  // namespace
}  // namespace blender::io::pmx::tests
