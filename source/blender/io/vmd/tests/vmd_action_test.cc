/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "vmd_action.hh"
#include "vmd_export.hh"
#include "vmd_import.hh"

#include "IO_vmd.hh"

#include "ANIM_action.hh"
#include "ANIM_action_iterators.hh"
#include "ANIM_fcurve.hh"
#include "ANIM_pose.hh"

#include "BKE_action.hh"
#include "BKE_anim_data.hh"
#include "BKE_animsys.hh"
#include "BKE_armature.hh"
#include "BKE_fcurve.hh"
#include "BKE_gtest_base.hh"
#include "BKE_idprop.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"
#include "BKE_pose.hh"

#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_anim_types.h"
#include "DNA_object_types.h"

#include "BLI_listbase.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_rotation_c.hh"
#include "BLI_math_vector.hh"
#include "BLI_fileops.hh"
#include "BLI_path_utils.hh"
#include "BLI_string.hh"
#include "BLI_tempfile.hh"

#include "testing/testing.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

namespace blender::io::vmd::tests {
namespace {

constexpr float kEpsilon = 1.0e-5f;

class VMDActionContractTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Object *armature_object = nullptr;
  bArmature *armature = nullptr;

  void SetUp() override
  {
    bmain = BKE_main_new();
    armature = BKE_armature_add(bmain, "C1D_Armature");
    armature_object = BKE_object_add_only_object(bmain, OB_ARMATURE, "C1D_ArmatureObject");
    armature_object->data = id_cast<ID *>(armature);

    add_bone("BoneA");
    add_bone("BoneB");
    BKE_pose_ensure(bmain, armature_object, armature, false);
  }

  void TearDown() override
  {
    BKE_main_free(bmain);
  }

  void add_bone(const std::string &name)
  {
    Bone *bone = MEM_new<Bone>("C1D_TestBone");
    STRNCPY(bone->name, name.c_str());
    BLI_addtail(&armature->bonebase, bone);
  }

  static void write_u32(FILE *file, const uint32_t value)
  {
    const uint8_t bytes[4] = {uint8_t(value & 0xff),
                               uint8_t((value >> 8) & 0xff),
                               uint8_t((value >> 16) & 0xff),
                               uint8_t((value >> 24) & 0xff)};
    fwrite(bytes, 1, sizeof(bytes), file);
  }

  static void write_f32(FILE *file, const float value)
  {
    uint32_t bits;
    static_assert(sizeof(bits) == sizeof(value));
    memcpy(&bits, &value, sizeof(bits));
    write_u32(file, bits);
  }

  struct TestMorphRecord {
    const char *name;
    uint32_t frame;
    float weight;
  };

  static std::string make_morph_vmd(std::initializer_list<TestMorphRecord> morphs,
                                    const char *filename)
  {
    char temp_dir[FILE_MAX];
    BLI_temp_directory_path_get(temp_dir, sizeof(temp_dir));
    char filepath[FILE_MAX];
    BLI_path_join(filepath, sizeof(filepath), temp_dir, filename);

    FILE *file = BLI_fopen(filepath, "wb");
    EXPECT_NE(file, nullptr);
    if (file == nullptr) {
      return filepath;
    }

    char header[50] = {};
    memcpy(header, "Vocaloid Motion Data 0002", 25);
    fwrite(header, 1, sizeof(header), file);
    write_u32(file, 0); /* bone */
    write_u32(file, uint32_t(morphs.size()));
    for (const TestMorphRecord &morph : morphs) {
      char name[15] = {};
      STRNCPY(name, morph.name);
      fwrite(name, 1, sizeof(name), file);
      write_u32(file, morph.frame);
      write_f32(file, morph.weight);
    }
    write_u32(file, 0); /* camera */
    write_u32(file, 0); /* light */
    write_u32(file, 0); /* self-shadow */
    write_u32(file, 0); /* property */
    fclose(file);
    return filepath;
  }

  static std::string make_malformed_morph_vmd(const bool write_weight,
                                               const char *filename)
  {
    char temp_dir[FILE_MAX];
    BLI_temp_directory_path_get(temp_dir, sizeof(temp_dir));
    char filepath[FILE_MAX];
    BLI_path_join(filepath, sizeof(filepath), temp_dir, filename);

    FILE *file = BLI_fopen(filepath, "wb");
    EXPECT_NE(file, nullptr);
    if (file == nullptr) {
      return filepath;
    }

    char header[50] = {};
    memcpy(header, "Vocaloid Motion Data 0002", 25);
    fwrite(header, 1, sizeof(header), file);
    write_u32(file, 0); /* bone */
    write_u32(file, 1); /* morph */
    char name[15] = {};
    STRNCPY(name, "Morph");
    fwrite(name, 1, sizeof(name), file);
    write_u32(file, 3);
    if (write_weight) {
      write_f32(file, std::numeric_limits<float>::quiet_NaN());
    }
    else {
      /* Stop at the incomplete morph record; section counts would otherwise
       * be consumed as the missing weight bytes. */
      fclose(file);
      return filepath;
    }
    write_u32(file, 0); /* camera */
    write_u32(file, 0); /* light */
    write_u32(file, 0); /* self-shadow */
    write_u32(file, 0); /* property */
    fclose(file);
    return filepath;
  }

  static std::string make_single_bone_vmd(const char *bone_name,
                                           const uint32_t frame,
                                           const char *filename)
  {
    char temp_dir[FILE_MAX];
    BLI_temp_directory_path_get(temp_dir, sizeof(temp_dir));
    char filepath[FILE_MAX];
    BLI_path_join(filepath, sizeof(filepath), temp_dir, filename);

    FILE *file = BLI_fopen(filepath, "wb");
    EXPECT_NE(file, nullptr);
    if (file == nullptr) {
      return filepath;
    }

    char header[50] = {};
    memcpy(header, "Vocaloid Motion Data 0002", 25);
    fwrite(header, 1, sizeof(header), file);
    write_u32(file, 1);

    char name[15] = {};
    STRNCPY(name, bone_name);
    fwrite(name, 1, sizeof(name), file);
    write_u32(file, frame);
    write_f32(file, 1.0f);
    write_f32(file, 2.0f);
    write_f32(file, 3.0f);
    write_f32(file, 0.0f);
    write_f32(file, 0.0f);
    write_f32(file, 0.0f);
    write_f32(file, 1.0f);
    const uint8_t interpolation[64] = {};
    fwrite(interpolation, 1, sizeof(interpolation), file);

    write_u32(file, 0); /* morph */
    write_u32(file, 0); /* camera */
    write_u32(file, 0); /* light */
    write_u32(file, 0); /* self-shadow */
    write_u32(file, 0); /* property */
    fclose(file);
    return filepath;
  }

  static std::string make_single_bone_ik_property_vmd(const char *bone_name,
                                                       const uint32_t frame,
                                                       const char *filename)
  {
    char temp_dir[FILE_MAX];
    BLI_temp_directory_path_get(temp_dir, sizeof(temp_dir));
    char filepath[FILE_MAX];
    BLI_path_join(filepath, sizeof(filepath), temp_dir, filename);

    FILE *file = BLI_fopen(filepath, "wb");
    EXPECT_NE(file, nullptr);
    if (file == nullptr) {
      return filepath;
    }
    char header[50] = {};
    memcpy(header, "Vocaloid Motion Data 0002", 25);
    fwrite(header, 1, sizeof(header), file);
    write_u32(file, 1); /* bone */
    char bone_record_name[15] = {};
    STRNCPY(bone_record_name, bone_name);
    fwrite(bone_record_name, 1, sizeof(bone_record_name), file);
    write_u32(file, 0);
    write_f32(file, 0.0f);
    write_f32(file, 0.0f);
    write_f32(file, 0.0f);
    write_f32(file, 0.0f);
    write_f32(file, 0.0f);
    write_f32(file, 0.0f);
    write_f32(file, 1.0f);
    const uint8_t interpolation[64] = {};
    fwrite(interpolation, 1, sizeof(interpolation), file);
    write_u32(file, 0); /* morph */
    write_u32(file, 0); /* camera */
    write_u32(file, 0); /* light */
    write_u32(file, 0); /* self-shadow */
    write_u32(file, 1); /* property */
    write_u32(file, frame);
    const uint8_t visible = 1;
    fwrite(&visible, 1, 1, file);
    write_u32(file, 1); /* IK state */
    char ik_name[20] = {};
    STRNCPY(ik_name, bone_name);
    fwrite(ik_name, 1, sizeof(ik_name), file);
    const uint8_t enabled = 1;
    fwrite(&enabled, 1, 1, file);
    fclose(file);
    return filepath;
  }

  void write_minimal_ik_definition(const char *ik_bone_name)
  {
    IDProperty *system = IDP_ID_system_properties_ensure(&armature_object->id);
    IDProperty *definition = bke::idprop::create_group(
        "mmd_pmx_bone_ik_definition").release();
    IDP_AddToGroup(definition, IDP_NewInt(2, "schema_version"));
    IDProperty *ik_bones = IDP_NewIDPArray("ik_bones");
    IDProperty *ik_bone = bke::idprop::create_group("ik_bone").release();
    IDP_AddToGroup(ik_bone, IDP_NewString(ik_bone_name, "name"));
    IDP_AddToGroup(ik_bone, IDP_NewString("BoneB", "target"));
    IDProperty *links = IDP_NewIDPArray("links");
    IDProperty *link = bke::idprop::create_group("link").release();
    IDP_AddToGroup(link, IDP_NewString("BoneB", "bone"));
    IDP_AppendArray(links, link);
    MEM_delete(link);
    IDP_AddToGroup(ik_bone, links);
    IDP_AppendArray(ik_bones, ik_bone);
    MEM_delete(ik_bone);
    IDP_AddToGroup(definition, ik_bones);
    IDP_AddToGroup(system, definition);
  }

  VMDModel make_model(std::initializer_list<VMDBoneKeyframe> keyframes)
  {
    VMDModel model;
    model.bone_keyframes.assign(keyframes.begin(), keyframes.end());
    return model;
  }

  VMDBoneKeyframe keyframe(const char *bone_name,
                           const uint32_t frame,
                           const std::array<float, 3> &translation,
                           const std::array<float, 4> &rotation)
  {
    VMDBoneKeyframe key;
    key.bone_name = bone_name;
    key.frame = frame;
    key.translation = translation;
    key.rotation = rotation;
    return key;
  }

  VMDActionReport build(const VMDModel &model,
                        const std::vector<std::string> &target_names,
                        const std::string &action_name = "C1D_TestAction",
                        const VMDActionOptions &options = {})
  {
    const VMDMappingReport mapping = map_bone_tracks(model, target_names);
    VMDActionReport result;
    EXPECT_TRUE(build_vmd_action(bmain,
                                  *armature_object,
                                  model,
                                  mapping,
                                  action_name,
                                  options,
                                  nullptr,
                                  result));
    return result;
  }

  int count_fcurves() const
  {
    const AnimData *anim_data = BKE_animdata_from_id(&armature_object->id);
    if (anim_data == nullptr || anim_data->action == nullptr) {
      return 0;
    }
    int count = 0;
    animrig::foreach_fcurve_in_action(anim_data->action->wrap(),
                                      [&](const FCurve & /*fcurve*/) { count++; });
    return count;
  }

  int count_keyframes() const
  {
    const AnimData *anim_data = BKE_animdata_from_id(&armature_object->id);
    if (anim_data == nullptr || anim_data->action == nullptr) {
      return 0;
    }
    int count = 0;
    animrig::foreach_fcurve_in_action(anim_data->action->wrap(),
                                      [&](const FCurve &fcurve) { count += fcurve.totvert; });
    return count;
  }

  bPoseChannel *pose_bone(const char *name) const
  {
    return BKE_pose_channel_find_name(armature_object->pose, name);
  }

  FCurve *find_fcurve(const char *path, const int array_index) const
  {
    const AnimData *anim_data = BKE_animdata_from_id(&armature_object->id);
    if (anim_data == nullptr || anim_data->action == nullptr) {
      return nullptr;
    }
    FCurve *found = nullptr;
    animrig::foreach_fcurve_in_action(anim_data->action->wrap(), [&](FCurve &fcurve) {
      if (found == nullptr && fcurve.array_index == array_index && fcurve.rna_path() == path) {
        found = &fcurve;
      }
    });
    return found;
  }
};

TEST_F(VMDActionContractTest, reader_preserves_morph_keyframes)
{
  const std::string filepath = make_morph_vmd({
      {"Smile", 0, 0.0f},
      {"Smile", 12, 0.75f},
      {"Blink", 24, 1.0f},
  }, "c2_1a_morph_reader.vmd");

  VMDReadReport report;
  VMDModel model;
  ASSERT_NO_THROW(model = read_vmd(filepath, &report));
  EXPECT_TRUE(report.ok());
  EXPECT_EQ(report.morph_frame_count, 3);
  EXPECT_EQ(model.morph_frame_count, 3);
  ASSERT_EQ(model.morph_keyframes.size(), 3);
  EXPECT_EQ(model.morph_keyframes[0].morph_name, "Smile");
  EXPECT_EQ(model.morph_keyframes[0].frame, 0);
  EXPECT_FLOAT_EQ(model.morph_keyframes[0].weight, 0.0f);
  EXPECT_EQ(model.morph_keyframes[1].morph_name, "Smile");
  EXPECT_EQ(model.morph_keyframes[1].frame, 12);
  EXPECT_FLOAT_EQ(model.morph_keyframes[1].weight, 0.75f);
  EXPECT_EQ(model.morph_keyframes[2].morph_name, "Blink");
  EXPECT_EQ(model.morph_keyframes[2].frame, 24);
  EXPECT_FLOAT_EQ(model.morph_keyframes[2].weight, 1.0f);
  EXPECT_EQ(model.morph_keyframes[0].source_offset, uint64_t(58));
  EXPECT_EQ(model.parse_end_offset, model.file_size);

  BLI_delete(filepath.c_str(), false, false);
}

TEST_F(VMDActionContractTest, reader_accepts_truncated_cp932_model_name_tail)
{
  const std::string filepath = make_morph_vmd({}, "truncated_cp932_model_name.vmd");
  FILE *file = BLI_fopen(filepath.c_str(), "r+b");
  ASSERT_NE(file, nullptr);
  fseek(file, 30, SEEK_SET);
  const uint8_t model_name[20] = {'M', 'o', 'd', 'e', 'l', 0x81};
  fwrite(model_name, 1, sizeof(model_name), file);
  fclose(file);

  VMDReadReport report;
  VMDModel model;
  ASSERT_NO_THROW(model = read_vmd(filepath, &report));
  EXPECT_TRUE(report.ok());
  EXPECT_EQ(model.header.model_name, "Model");
  ASSERT_EQ(report.warnings.size(), 1);
  EXPECT_NE(report.warnings[0].find("incomplete trailing CP932"), std::string::npos);

  BLI_delete(filepath.c_str(), false, false);
}

TEST_F(VMDActionContractTest, reader_accepts_absent_property_section)
{
  char temp_dir[FILE_MAX];
  BLI_temp_directory_path_get(temp_dir, sizeof(temp_dir));
  char filepath_buffer[FILE_MAX];
  BLI_path_join(
      filepath_buffer, sizeof(filepath_buffer), temp_dir, "absent_property_section.vmd");
  const std::string filepath = filepath_buffer;
  FILE *file = BLI_fopen(filepath.c_str(), "wb");
  ASSERT_NE(file, nullptr);
  char header[50] = {};
  memcpy(header, "Vocaloid Motion Data 0002", 25);
  fwrite(header, 1, sizeof(header), file);
  write_u32(file, 0); /* bone */
  write_u32(file, 0); /* morph */
  write_u32(file, 0); /* camera */
  write_u32(file, 0); /* light */
  write_u32(file, 0); /* self-shadow */
  fclose(file);

  VMDReadReport report;
  VMDModel model;
  ASSERT_NO_THROW(model = read_vmd(filepath, &report));
  EXPECT_TRUE(report.ok());
  EXPECT_TRUE(model.property_keyframes.empty());
  EXPECT_EQ(model.parse_end_offset, model.file_size);

  BLI_delete(filepath.c_str(), false, false);
}

TEST_F(VMDActionContractTest, writer_reader_round_trip)
{
  char temp_dir[FILE_MAX];
  BLI_temp_directory_path_get(temp_dir, sizeof(temp_dir));
  char filepath_buffer[FILE_MAX];
  BLI_path_join(filepath_buffer, sizeof(filepath_buffer), temp_dir, "vmd_writer_round_trip.vmd");
  const std::string filepath = filepath_buffer;

  VMDModel source;
  source.header.model_name = "Model";
  VMDBoneKeyframe key;
  const std::string chinese_bone_name =
      "\xe5\x89\x8d\xe5\xb8\xa6\xe5\xad\x90_0_1"; /* 前带子_0_1 */
  key.bone_name = chinese_bone_name;
  key.frame = 12;
  key.translation = {1.0f, 2.0f, 3.0f};
  key.rotation = {0.0f, 0.0f, 0.0f, 1.0f};
  source.bone_keyframes.push_back(key);
  VMDMorphKeyframe morph_key;
  morph_key.morph_name = "Smile";
  morph_key.frame = 8;
  morph_key.weight = 0.75f;
  source.morph_keyframes.push_back(morph_key);

  VMDWriteReport write_report;
  ASSERT_TRUE(write_vmd(filepath, source, &write_report));
  EXPECT_TRUE(write_report.success);
  EXPECT_EQ(write_report.bone_frame_count, 1);
  EXPECT_EQ(write_report.morph_frame_count, 1);

  VMDReadReport read_report;
  const VMDModel result = read_vmd(filepath, &read_report);
  ASSERT_TRUE(read_report.ok());
  ASSERT_EQ(result.bone_keyframes.size(), 1);
  EXPECT_EQ(result.header.model_name, "Model");
  EXPECT_EQ(result.bone_keyframes[0].bone_name, chinese_bone_name);
  EXPECT_EQ(result.bone_keyframes[0].frame, 12);
  EXPECT_EQ(result.bone_keyframes[0].translation, key.translation);
  EXPECT_EQ(result.bone_keyframes[0].rotation, key.rotation);
  ASSERT_EQ(result.morph_keyframes.size(), 1);
  EXPECT_EQ(result.morph_keyframes[0].morph_name, "Smile");
  EXPECT_EQ(result.morph_keyframes[0].frame, 8);
  EXPECT_FLOAT_EQ(result.morph_keyframes[0].weight, 0.75f);

  BLI_delete(filepath.c_str(), false, false);
}

TEST_F(VMDActionContractTest, bone_converter_inverse_round_trip)
{
  BoneConverter converter;
  const float source_location[3] = {1.25f, -2.5f, 3.75f};
  float blender_location[3], result_location[3];
  converter.convert_location(source_location, blender_location, 0.08f);
  converter.inverse_location(blender_location, result_location, 0.08f);
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(result_location[i], source_location[i], 1.0e-6f);
  }

  const float source_rotation[4] = {0.1f, -0.2f, 0.3f, 0.9f};
  float blender_rotation[4], result_rotation[4];
  converter.convert_rotation(source_rotation, blender_rotation);
  converter.inverse_rotation(blender_rotation, result_rotation);
  float normalized_source[4] = {
      source_rotation[3], source_rotation[0], source_rotation[1], source_rotation[2]};
  normalize_qt(normalized_source);
  EXPECT_NEAR(result_rotation[0], normalized_source[1], 1.0e-6f);
  EXPECT_NEAR(result_rotation[1], normalized_source[2], 1.0e-6f);
  EXPECT_NEAR(result_rotation[2], normalized_source[3], 1.0e-6f);
  EXPECT_NEAR(result_rotation[3], normalized_source[0], 1.0e-6f);
}

TEST_F(VMDActionContractTest, reader_rejects_truncated_or_non_finite_morph)
{
  const std::string truncated_path = make_malformed_morph_vmd(
      false, "c2_1a_morph_truncated.vmd");
  EXPECT_THROW(read_vmd(truncated_path), VMDReaderError);
  BLI_delete(truncated_path.c_str(), false, false);

  const std::string non_finite_path = make_malformed_morph_vmd(
      true, "c2_1a_morph_non_finite.vmd");
  EXPECT_THROW(read_vmd(non_finite_path), VMDReaderError);
  BLI_delete(non_finite_path.c_str(), false, false);
}

TEST_F(VMDActionContractTest, coordinates_and_time)
{
  /* Set BoneA's matrix_local to identity so the converter behaves like the
   * simple (x,z,y) swap for this existing regression test. */
  Bone *bone_a = static_cast<Bone *>(armature->bonebase.first);
  ASSERT_NE(bone_a, nullptr);
  unit_m4(bone_a->arm_mat);

  ASSERT_NE(pose_bone("BoneB"), nullptr);
  pose_bone("BoneB")->rotmode = ROT_MODE_XYZ;

  const VMDModel model = make_model({
      keyframe("BoneA", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}),
      keyframe("BoneA", 10, {10.0f, 2.0f, 3.0f}, {0.0f, 0.0f, 0.0f, 1.0f}),
  });

  const VMDActionReport result = build(model, {"BoneA", "BoneB"});
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.action_bound);
  EXPECT_EQ(result.mapped_track_count, 1);
  EXPECT_EQ(result.missing_track_count, 0);
  EXPECT_EQ(result.location_fcurve_count, 3);
  EXPECT_EQ(result.rotation_fcurve_count, 4);
  EXPECT_EQ(result.location_keyframe_count, 6);
  EXPECT_EQ(result.rotation_keyframe_count, 8);
  EXPECT_EQ(result.first_frame, 0);
  EXPECT_EQ(result.last_frame, 10);
  EXPECT_EQ(count_fcurves(), 7);

  const AnimData *anim_data = BKE_animdata_from_id(&armature_object->id);
  ASSERT_NE(anim_data, nullptr);
  ASSERT_NE(anim_data->action, nullptr);
  EXPECT_EQ(anim_data->action->wrap().get_frame_range(), (float2{0.0f, 10.0f}));
  EXPECT_EQ(pose_bone("BoneA")->rotmode, ROT_MODE_QUAT);
  EXPECT_EQ(pose_bone("BoneB")->rotmode, ROT_MODE_XYZ);

  FCurve *location_x = find_fcurve("pose.bones[\"BoneA\"].location", 0);
  ASSERT_NE(location_x, nullptr);
  ASSERT_EQ(location_x->totvert, 2);
  EXPECT_EQ(location_x->bezt[0].ipo, BEZT_IPO_LIN);
  EXPECT_EQ(location_x->bezt[1].ipo, BEZT_IPO_LIN);
  EXPECT_NEAR(evaluate_fcurve(location_x, 5.0f), 0.4f, kEpsilon);
  EXPECT_NEAR(location_x->bezt[1].vec[1][1], 0.8f, kEpsilon);
  FCurve *location_y = find_fcurve("pose.bones[\"BoneA\"].location", 1);
  FCurve *location_z = find_fcurve("pose.bones[\"BoneA\"].location", 2);
  ASSERT_NE(location_y, nullptr);
  ASSERT_NE(location_z, nullptr);
  EXPECT_NEAR(location_y->bezt[1].vec[1][1], 0.24f, kEpsilon);
  EXPECT_NEAR(location_z->bezt[1].vec[1][1], 0.16f, kEpsilon);

  AnimationEvalContext eval_context = {nullptr, 5.0f};
  animrig::pose_apply_action_all_bones(
      armature_object, anim_data->action, anim_data->slot_handle, &eval_context);
  EXPECT_NEAR(pose_bone("BoneA")->loc[0], 0.4f, kEpsilon);
  EXPECT_NEAR(pose_bone("BoneA")->loc[1], 0.12f, kEpsilon);
  EXPECT_NEAR(pose_bone("BoneA")->loc[2], 0.08f, kEpsilon);

  VMDActionOptions offset_options;
  offset_options.frame_offset = 10;
  offset_options.replace_existing_action = true;
  const VMDActionReport offset_result = build(
      model, {"BoneA"}, "C1D_OffsetAction", offset_options);
  EXPECT_TRUE(offset_result.success);
  EXPECT_EQ(offset_result.first_frame, 10);
  EXPECT_EQ(offset_result.last_frame, 20);
}

TEST_F(VMDActionContractTest, quaternion_normalization_and_sign_continuity)
{
  const VMDModel model = make_model({
      keyframe("BoneA", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 2.0f}),
      keyframe("BoneA", 1, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, -2.0f}),
  });

  const VMDActionReport result = build(model, {"BoneA"});
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.quaternion_sign_flip_count, 1);

  FCurve *rotation_w = find_fcurve("pose.bones[\"BoneA\"].rotation_quaternion", 0);
  FCurve *rotation_x = find_fcurve("pose.bones[\"BoneA\"].rotation_quaternion", 1);
  FCurve *rotation_y = find_fcurve("pose.bones[\"BoneA\"].rotation_quaternion", 2);
  FCurve *rotation_z = find_fcurve("pose.bones[\"BoneA\"].rotation_quaternion", 3);
  ASSERT_NE(rotation_w, nullptr);
  ASSERT_NE(rotation_x, nullptr);
  ASSERT_NE(rotation_y, nullptr);
  ASSERT_NE(rotation_z, nullptr);
  EXPECT_NEAR(evaluate_fcurve(rotation_w, 0.0f), 1.0f, kEpsilon);
  EXPECT_NEAR(evaluate_fcurve(rotation_w, 1.0f), 1.0f, kEpsilon);
  EXPECT_NEAR(evaluate_fcurve(rotation_x, 1.0f), 0.0f, kEpsilon);
  EXPECT_NEAR(evaluate_fcurve(rotation_y, 1.0f), 0.0f, kEpsilon);
  EXPECT_NEAR(evaluate_fcurve(rotation_z, 1.0f), 0.0f, kEpsilon);

  const AnimData *anim_data = BKE_animdata_from_id(&armature_object->id);
  ASSERT_NE(anim_data, nullptr);
  ASSERT_NE(anim_data->action, nullptr);
  AnimationEvalContext eval_context = {nullptr, 1.0f};
  animrig::pose_apply_action_all_bones(
      armature_object, anim_data->action, anim_data->slot_handle, &eval_context);
  const bPoseChannel *bone = pose_bone("BoneA");
  ASSERT_NE(bone, nullptr);
  EXPECT_NEAR(bone->quat[0], 1.0f, kEpsilon);
  EXPECT_NEAR(bone->quat[1], 0.0f, kEpsilon);
  EXPECT_NEAR(bone->quat[2], 0.0f, kEpsilon);
  EXPECT_NEAR(bone->quat[3], 0.0f, kEpsilon);
}

TEST_F(VMDActionContractTest, import_orchestration_success_and_options)
{
  const std::string filepath = make_single_bone_vmd("BoneA", 5, "c1e3_orchestration.vmd");

  VMDImportOptions options;
  VMDImportReport result;
  EXPECT_TRUE(import_vmd_action(
      bmain, *armature_object, filepath, options, nullptr, result));
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.action.action_name, "c1e3_orchestration | VMD");
  EXPECT_EQ(result.mapping.mapped_track_count, 1);
  EXPECT_EQ(result.mapping.missing_track_count, 0);
  EXPECT_EQ(result.action.first_frame, 5);
  EXPECT_EQ(result.action.last_frame, 5);
  EXPECT_EQ(result.action.location_fcurve_count, 3);
  EXPECT_EQ(result.action.rotation_fcurve_count, 4);
  EXPECT_EQ(count_fcurves(), 7);

  VMDImportOptions offset_options;
  offset_options.frame_offset = 10;
  offset_options.replace_existing_action = true;
  VMDImportReport offset_result;
  EXPECT_TRUE(import_vmd_action(
      bmain, *armature_object, filepath, offset_options, nullptr, offset_result));
  EXPECT_TRUE(offset_result.success);
  EXPECT_EQ(offset_result.action.first_frame, 15);
  EXPECT_EQ(offset_result.action.last_frame, 15);
  EXPECT_EQ(count_fcurves(), 7);

  BLI_delete(filepath.c_str(), false, false);
}

TEST_F(VMDActionContractTest, import_orchestration_failure_preserves_action)
{
  const std::string valid_path = make_single_bone_vmd("BoneA", 0, "c1e3_failure_valid.vmd");
  VMDImportReport first_result;
  ASSERT_TRUE(import_vmd_action(
      bmain, *armature_object, valid_path, {}, nullptr, first_result));
  ASSERT_EQ(count_fcurves(), 7);
  bAction *existing_action = BKE_animdata_from_id(&armature_object->id)->action;

  const std::string missing_path = make_single_bone_vmd("Missing", 0, "c1e3_failure_missing.vmd");
  VMDImportReport missing_result;
  EXPECT_FALSE(import_vmd_action(
      bmain, *armature_object, missing_path, {}, nullptr, missing_result));
  EXPECT_FALSE(missing_result.success);
  EXPECT_EQ(count_fcurves(), 7);
  EXPECT_EQ(BKE_animdata_from_id(&armature_object->id)->action, existing_action);

  VMDImportReport reader_result;
  EXPECT_FALSE(import_vmd_action(
      bmain, *armature_object, "c1e3_file_does_not_exist.vmd", {}, nullptr, reader_result));
  EXPECT_FALSE(reader_result.success);
  EXPECT_EQ(count_fcurves(), 7);
  EXPECT_EQ(BKE_animdata_from_id(&armature_object->id)->action, existing_action);

  BLI_delete(valid_path.c_str(), false, false);
  BLI_delete(missing_path.c_str(), false, false);
}

TEST_F(VMDActionContractTest, real_sample_reader_and_action)
{
  const char *sample_path = BLI_getenv("VMD_C1D_SAMPLE");
  if (sample_path == nullptr || sample_path[0] == '\0') {
    GTEST_SKIP() << "VMD_C1D_SAMPLE is not set; skipping real VMD sample regression";
  }

  VMDReadReport read_report;
  VMDModel model;
  try {
    model = read_vmd(sample_path, &read_report);
  }
  catch (const VMDReaderError &error) {
    FAIL() << "Failed to read VMD_C1D_SAMPLE: " << error.what();
  }

  ASSERT_TRUE(read_report.ok());
  EXPECT_EQ(read_report.file_size, uint64_t(1503183));
  EXPECT_EQ(read_report.parse_end_offset, read_report.file_size);
  EXPECT_EQ(read_report.bone_frame_count, uint32_t(13521));
  EXPECT_EQ(model.bone_keyframes.size(), size_t(13521));
  EXPECT_EQ(model.morph_frame_count, uint32_t(95));
  EXPECT_EQ(model.camera_frame_count, uint32_t(0));
  EXPECT_EQ(model.light_frame_count, uint32_t(0));
  EXPECT_EQ(model.shadow_frame_count, uint32_t(0));
  EXPECT_EQ(read_report.property_frame_count, uint32_t(1));

  std::unordered_set<std::string> unique_bone_names;
  unique_bone_names.reserve(model.bone_keyframes.size());
  int first_frame = std::numeric_limits<int>::max();
  int last_frame = std::numeric_limits<int>::min();
  for (const VMDBoneKeyframe &keyframe : model.bone_keyframes) {
    unique_bone_names.insert(keyframe.bone_name);
    ASSERT_LE(keyframe.frame, uint32_t(std::numeric_limits<int>::max()));
    first_frame = std::min(first_frame, int(keyframe.frame));
    last_frame = std::max(last_frame, int(keyframe.frame));
  }
  EXPECT_EQ(unique_bone_names.size(), size_t(425));
  EXPECT_EQ(first_frame, 0);
  EXPECT_EQ(last_frame, 645);

  /* Build the real-sample armature from an empty pose. The fixture already has
   * BoneA/BoneB and an initialized pose for the synthetic contract tests; adding
   * hundreds of bones to that live pose and rebuilding it is not a valid minimal
   * setup for this regression and can leave stale pose-channel pointers. */
  armature = BKE_armature_add(bmain, "C1D_RealSampleArmature");
  armature_object = BKE_object_add_only_object(
      bmain, OB_ARMATURE, "C1D_RealSampleArmatureObject");
  armature_object->data = id_cast<ID *>(armature);
  for (const std::string &bone_name : unique_bone_names) {
    add_bone(bone_name);
  }
  BKE_pose_ensure(bmain, armature_object, armature, false);

  std::vector<std::string> target_names;
  target_names.reserve(unique_bone_names.size());
  for (const std::string &bone_name : unique_bone_names) {
    target_names.push_back(bone_name);
  }

  const VMDMappingReport mapping = map_bone_tracks(model, target_names);
  ASSERT_TRUE(mapping.mapping_valid);
  EXPECT_EQ(mapping.vmd_track_count, 425);
  EXPECT_EQ(mapping.mapped_track_count, 425);
  EXPECT_EQ(mapping.missing_track_count, 0);
  EXPECT_EQ(mapping.mapped_keyframe_count + mapping.ignored_keyframe_count, 13521);
  EXPECT_EQ(mapping.first_frame, 0);
  EXPECT_EQ(mapping.last_frame, 645);

  VMDActionReport action_report;
  ASSERT_TRUE(build_vmd_action(bmain,
                               *armature_object,
                               model,
                               mapping,
                               "C1D_RealSampleAction",
                               {},
                               nullptr,
                               action_report));
  EXPECT_TRUE(action_report.success);
  EXPECT_TRUE(action_report.action_bound);
  EXPECT_EQ(action_report.mapped_track_count, 425);
  EXPECT_EQ(action_report.missing_track_count, 0);
  EXPECT_EQ(action_report.location_fcurve_count, 425 * 3);
  EXPECT_EQ(action_report.rotation_fcurve_count, 425 * 4);
  EXPECT_EQ(action_report.location_keyframe_count, mapping.mapped_keyframe_count * 3);
  EXPECT_EQ(action_report.rotation_keyframe_count, mapping.mapped_keyframe_count * 4);
  EXPECT_EQ(action_report.first_frame, 0);
  EXPECT_EQ(action_report.last_frame, 645);
  EXPECT_EQ(count_fcurves(), 425 * 7);
  EXPECT_EQ(count_keyframes(), mapping.mapped_keyframe_count * 7);
}

TEST_F(VMDActionContractTest, partial_missing_track_is_reported)
{
  const VMDModel model = make_model({
      keyframe("BoneA", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}),
      keyframe("Missing", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}),
  });

  const VMDMappingReport mapping = map_bone_tracks(model, {"BoneA", "BoneB"});
  EXPECT_TRUE(mapping.mapping_valid);
  EXPECT_EQ(mapping.missing_track_count, 1);

  VMDActionReport result;
  EXPECT_TRUE(build_vmd_action(bmain,
                               *armature_object,
                               model,
                               mapping,
                               "C1D_PartialMissingAction",
                               {},
                               nullptr,
                               result));
  EXPECT_EQ(result.location_fcurve_count, 3);
  EXPECT_EQ(result.rotation_fcurve_count, 4);
  EXPECT_EQ(count_fcurves(), 7);
}

TEST_F(VMDActionContractTest, all_missing_and_existing_action_are_protected)
{
  const VMDModel missing_model = make_model({
      keyframe("Missing", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}),
  });
  const VMDMappingReport missing_mapping = map_bone_tracks(missing_model, {"BoneA"});
  EXPECT_FALSE(missing_mapping.mapping_valid);

  VMDActionReport missing_result;
  EXPECT_FALSE(build_vmd_action(bmain,
                                *armature_object,
                                missing_model,
                                missing_mapping,
                                "C1D_ShouldNotExist",
                                {},
                                nullptr,
                                missing_result));
  EXPECT_EQ(count_fcurves(), 0);
  EXPECT_EQ(BKE_animdata_from_id(&armature_object->id), nullptr);

  const VMDModel valid_model = make_model({
      keyframe("BoneA", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}),
  });
  VMDActionReport first_result;
  ASSERT_TRUE(build_vmd_action(bmain,
                               *armature_object,
                               valid_model,
                               map_bone_tracks(valid_model, {"BoneA"}),
                               "C1D_ExistingAction",
                               {},
                               nullptr,
                               first_result));
  ASSERT_EQ(count_fcurves(), 7);

  VMDActionReport protected_result;
  EXPECT_FALSE(build_vmd_action(bmain,
                                *armature_object,
                                valid_model,
                                map_bone_tracks(valid_model, {"BoneA"}),
                                "C1D_ReplacementAction",
                                {},
                                nullptr,
                                protected_result));
  EXPECT_EQ(count_fcurves(), 7);
  EXPECT_EQ(protected_result.action_name, "C1D_ReplacementAction");
}

void fill_interp(VMDBoneKeyframe &kf, int ax, int ay, int bx, int by)
{
  for (int channel = 0; channel < 4; channel++) {
    const int base = channel * 16;
    kf.interpolation[base] = int8_t(ax);
    kf.interpolation[base + 4] = int8_t(ay);
    kf.interpolation[base + 8] = int8_t(bx);
    kf.interpolation[base + 12] = int8_t(by);
  }
}

TEST_F(VMDActionContractTest, bone_interpolation_default_is_collinear_free_handles)
{
  VMDBoneKeyframe k0 = keyframe("BoneA", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
  VMDBoneKeyframe k1 = keyframe("BoneA", 30, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
  fill_interp(k0, 20, 20, 107, 107);
  fill_interp(k1, 20, 20, 107, 107);

  VMDActionOptions options;
  options.use_linear_interpolation = false;
  options.use_vmd_bezier_interpolation = true;
  build(make_model({k0, k1}), {"BoneA"}, "C3_Collinear", options);

  FCurve *lc = find_fcurve("pose.bones[\"BoneA\"].location", 0);
  ASSERT_NE(lc, nullptr);
  ASSERT_EQ(lc->totvert, 2);
  /* C3 must produce FREE handles, never the LINEAR enum. */
  EXPECT_EQ(lc->bezt[0].h2, HD_FREE);
  EXPECT_EQ(lc->bezt[1].h1, HD_FREE);
  EXPECT_EQ(lc->bezt[0].h1, HD_FREE);
  EXPECT_EQ(lc->bezt[1].h2, HD_FREE);
  /* Collinear preset => control point lies on the P0->P3 line (visual linear). */
  const float f0 = lc->bezt[0].vec[1][0], v0 = lc->bezt[0].vec[1][1];
  const float df = lc->bezt[1].vec[1][0] - f0, dv = lc->bezt[1].vec[1][1] - v0;
  const float ax = 20.0f / 127.0f;
  EXPECT_NEAR(lc->bezt[0].vec[2][0], f0 + ax * df, kEpsilon);
  EXPECT_NEAR(lc->bezt[0].vec[2][1], v0 + ax * dv, kEpsilon);
}

TEST_F(VMDActionContractTest, bone_interpolation_nonlinear_maps_control_points)
{
  VMDBoneKeyframe k0 = keyframe("BoneA", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
  VMDBoneKeyframe k1 = keyframe("BoneA", 30, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
  fill_interp(k0, 20, 20, 107, 107);
  fill_interp(k1, 20, 20, 107, 107);
  k1.interpolation[0] = 10;  /* X.ax */
  k1.interpolation[4] = 20;  /* X.ay */
  k1.interpolation[8] = 107; /* X.bx */
  k1.interpolation[12] = 80; /* X.by */

  VMDActionOptions options;
  options.use_linear_interpolation = false;
  options.use_vmd_bezier_interpolation = true;
  build(make_model({k0, k1}), {"BoneA"}, "C3_Nonlinear", options);

  FCurve *lc = find_fcurve("pose.bones[\"BoneA\"].location", 0);
  ASSERT_NE(lc, nullptr);
  const float f0 = lc->bezt[0].vec[1][0], v0 = lc->bezt[0].vec[1][1];
  const float df = lc->bezt[1].vec[1][0] - f0, dv = lc->bezt[1].vec[1][1] - v0;
  const float ax = 10.0f / 127.0f, ay = 20.0f / 127.0f, bx = 107.0f / 127.0f, by = 80.0f / 127.0f;
  EXPECT_NEAR(lc->bezt[0].vec[2][0], f0 + ax * df, kEpsilon);
  EXPECT_NEAR(lc->bezt[0].vec[2][1], v0 + ay * dv, kEpsilon);
  EXPECT_NEAR(lc->bezt[1].vec[0][0], f0 + bx * df, kEpsilon);
  EXPECT_NEAR(lc->bezt[1].vec[0][1], v0 + by * dv, kEpsilon);
}

TEST_F(VMDActionContractTest, bone_rotation_shares_single_interpolation_curve)
{
  VMDBoneKeyframe k0 = keyframe("BoneA", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
  VMDBoneKeyframe k1 = keyframe("BoneA", 30, {0.0f, 0.0f, 0.0f}, {1.0f, 0.5f, 0.3f, 0.8f});
  fill_interp(k0, 20, 20, 107, 107);
  fill_interp(k1, 20, 20, 107, 107);
  k1.interpolation[48] = 10;
  k1.interpolation[52] = 20;
  k1.interpolation[56] = 107;
  k1.interpolation[60] = 80;

  VMDActionOptions options;
  options.use_linear_interpolation = false;
  options.use_vmd_bezier_interpolation = true;
  build(make_model({k0, k1}), {"BoneA"}, "C3_RotationShared", options);

  float right_x[4], left_x[4];
  for (int i = 0; i < 4; i++) {
    FCurve *rc = find_fcurve("pose.bones[\"BoneA\"].rotation_quaternion", i);
    ASSERT_NE(rc, nullptr);
    right_x[i] = rc->bezt[0].vec[2][0];
    left_x[i] = rc->bezt[1].vec[0][0];
  }
  EXPECT_NEAR(right_x[0], right_x[1], kEpsilon);
  EXPECT_NEAR(right_x[0], right_x[2], kEpsilon);
  EXPECT_NEAR(right_x[0], right_x[3], kEpsilon);
  EXPECT_NEAR(left_x[0], left_x[1], kEpsilon);
  EXPECT_NEAR(left_x[0], left_x[2], kEpsilon);
  EXPECT_NEAR(left_x[0], left_x[3], kEpsilon);
}

TEST_F(VMDActionContractTest, bone_location_uses_reordered_channels)
{
  /* Distinct ax per VMD channel proves the Y-up<->Z-up reorder is honoured:
   * location[0]<-VMD X(ax=10), location[1]<-VMD Z(ax=107), location[2]<-VMD Y(ax=20). */
  Bone *bone_a = static_cast<Bone *>(armature->bonebase.first);
  ASSERT_NE(bone_a, nullptr);
  unit_m4(bone_a->arm_mat);

  VMDBoneKeyframe k0 = keyframe("BoneA", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
  VMDBoneKeyframe k1 = keyframe("BoneA", 30, {1.0f, 2.0f, 3.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
  fill_interp(k0, 20, 20, 107, 107);
  fill_interp(k1, 20, 20, 107, 107);
  k1.interpolation[0] = 10;  /* VMD X.ax -> location[0] */
  k1.interpolation[32] = 107; /* VMD Z.ax -> location[1] */
  k1.interpolation[16] = 20;  /* VMD Y.ax -> location[2] */

  VMDActionOptions options;
  options.use_linear_interpolation = false;
  options.use_vmd_bezier_interpolation = true;
  build(make_model({k0, k1}), {"BoneA"}, "C3_Reordered", options);

  FCurve *l0 = find_fcurve("pose.bones[\"BoneA\"].location", 0);
  FCurve *l1 = find_fcurve("pose.bones[\"BoneA\"].location", 1);
  FCurve *l2 = find_fcurve("pose.bones[\"BoneA\"].location", 2);
  ASSERT_NE(l0, nullptr);
  ASSERT_NE(l1, nullptr);
  ASSERT_NE(l2, nullptr);
  const float f0 = l0->bezt[0].vec[1][0];
  const float df = l0->bezt[1].vec[1][0] - f0; /* same frame delta for all components */
  const float ax_x = 10.0f / 127.0f, ax_z = 107.0f / 127.0f, ax_y = 20.0f / 127.0f;
  EXPECT_NEAR(l0->bezt[0].vec[2][0], f0 + ax_x * df, kEpsilon);
  EXPECT_NEAR(l1->bezt[0].vec[2][0], f0 + ax_z * df, kEpsilon);
  EXPECT_NEAR(l2->bezt[0].vec[2][0], f0 + ax_y * df, kEpsilon);
  EXPECT_NE(l0->bezt[0].vec[2][0], l1->bezt[0].vec[2][0]);
  EXPECT_NE(l0->bezt[0].vec[2][0], l2->bezt[0].vec[2][0]);
}

TEST_F(VMDActionContractTest, ik_property_frames_include_import_offset)
{
  write_minimal_ik_definition("BoneA");
  const std::string filepath = make_single_bone_ik_property_vmd(
      "BoneA", 5, "vmd_ik_property_offset.vmd");

  VMDImportOptions options;
  options.frame_offset = 10;
  VMDImportReport result;
  ASSERT_TRUE(import_vmd_action(
      bmain, *armature_object, filepath, options, nullptr, result));

  FCurve *toggle = find_fcurve("pose.bones[\"BoneA\"].mmd_ik_toggle", 0);
  ASSERT_NE(toggle, nullptr);
  ASSERT_EQ(toggle->totvert, 1);
  EXPECT_FLOAT_EQ(toggle->bezt[0].vec[1][0], 15.0f);
  EXPECT_FLOAT_EQ(toggle->bezt[0].vec[1][1], 1.0f);

  BLI_delete(filepath.c_str(), false, false);
}

TEST_F(VMDActionContractTest, nonlinear_interpolation_survives_export_and_reimport)
{
  Bone *bone = static_cast<Bone *>(armature->bonebase.first);
  ASSERT_NE(bone, nullptr);
  unit_m4(bone->arm_mat);

  VMDBoneKeyframe k0 = keyframe(
      "BoneA", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f});
  VMDBoneKeyframe k1 = keyframe(
      "BoneA", 30, {3.0f, 4.0f, 5.0f}, {0.2f, 0.3f, 0.4f, 0.8f});
  fill_interp(k0, 20, 20, 107, 107);
  fill_interp(k1, 20, 20, 107, 107);
  const int controls[4][4] = {
      {10, 24, 91, 75}, {17, 31, 98, 82}, {26, 12, 105, 69}, {8, 35, 112, 88}};
  for (int channel = 0; channel < 4; channel++) {
    const int base = channel * 16;
    k1.interpolation[base] = int8_t(controls[channel][0]);
    k1.interpolation[base + 4] = int8_t(controls[channel][1]);
    k1.interpolation[base + 8] = int8_t(controls[channel][2]);
    k1.interpolation[base + 12] = int8_t(controls[channel][3]);
  }

  VMDActionOptions import_options;
  import_options.use_linear_interpolation = false;
  import_options.use_vmd_bezier_interpolation = true;
  build(make_model({k0, k1}), {"BoneA"}, "VMD Export Bezier Source", import_options);

  /* Blender-authored handles need not land exactly on the VMD 1/127 grid.
   * Keep them representable by rounding to the nearest VMD control byte. */
  FCurve *location_x = find_fcurve("pose.bones[\"BoneA\"].location", 0);
  ASSERT_NE(location_x, nullptr);
  ASSERT_NE(location_x->bezt, nullptr);
  location_x->bezt[0].vec[2][0] += (30.0f / 127.0f) * 0.25f;
  location_x->bezt[0].vec[2][1] +=
      ((location_x->bezt[1].vec[1][1] - location_x->bezt[0].vec[1][1]) / 127.0f) * 0.25f;

  const char *paths[2] = {"pose.bones[\"BoneA\"].location",
                          "pose.bones[\"BoneA\"].rotation_quaternion"};
  std::array<float, 21> source_samples{};
  int sample_index = 0;
  for (const int frame : {7, 15, 23}) {
    for (int component = 0; component < 3; component++) {
      source_samples[sample_index++] = evaluate_fcurve(find_fcurve(paths[0], component), frame);
    }
    for (int component = 0; component < 4; component++) {
      source_samples[sample_index++] = evaluate_fcurve(find_fcurve(paths[1], component), frame);
    }
  }

  char temp_dir[FILE_MAX];
  BLI_temp_directory_path_get(temp_dir, sizeof(temp_dir));
  char filepath[FILE_MAX];
  BLI_path_join(filepath, sizeof(filepath), temp_dir, "vmd_bezier_export_round_trip.vmd");
  VMDExportOptions export_options;
  export_options.frame_end = 30;
  VMDExportReport export_report;
  ASSERT_TRUE(export_vmd_action(
      *armature_object, filepath, export_options, nullptr, export_report));

  VMDReadReport read_report;
  const VMDModel exported = read_vmd(filepath, &read_report);
  ASSERT_TRUE(read_report.ok());
  ASSERT_EQ(exported.bone_keyframes.size(), 2);
  for (int channel = 0; channel < 4; channel++) {
    const int base = channel * 16;
    for (const int offset : {0, 4, 8, 12}) {
      EXPECT_EQ(uint8_t(exported.bone_keyframes[1].interpolation[base + offset]),
                uint8_t(k1.interpolation[base + offset]))
          << "interpolation byte offset " << base + offset;
    }
  }

  import_options.replace_existing_action = true;
  build(exported, {"BoneA"}, "VMD Export Bezier Reimport", import_options);
  sample_index = 0;
  for (const int frame : {7, 15, 23}) {
    for (int component = 0; component < 3; component++) {
      EXPECT_NEAR(evaluate_fcurve(find_fcurve(paths[0], component), frame),
                  source_samples[sample_index++],
                  2.0e-3f);
    }
    for (int component = 0; component < 4; component++) {
      EXPECT_NEAR(evaluate_fcurve(find_fcurve(paths[1], component), frame),
                  source_samples[sample_index++],
                  2.0e-3f);
    }
  }
  BLI_delete(filepath, false, false);
}

TEST_F(VMDActionContractTest, ordinary_linear_curves_export_linear_interpolation)
{
  const VMDModel model = make_model({
      keyframe("BoneA", 0, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}),
      keyframe("BoneA", 30, {3.0f, 4.0f, 5.0f}, {0.2f, 0.3f, 0.4f, 0.8f}),
  });
  build(model, {"BoneA"}, "VMD Export Linear Source");

  char temp_dir[FILE_MAX];
  BLI_temp_directory_path_get(temp_dir, sizeof(temp_dir));
  char filepath[FILE_MAX];
  BLI_path_join(filepath, sizeof(filepath), temp_dir, "vmd_linear_export_contract.vmd");
  VMDExportOptions export_options;
  export_options.frame_end = 30;
  VMDExportReport export_report;
  ASSERT_TRUE(export_vmd_action(
      *armature_object, filepath, export_options, nullptr, export_report));

  const VMDModel exported = read_vmd(filepath);
  ASSERT_EQ(exported.bone_keyframes.size(), 2);
  const int linear[4] = {20, 20, 107, 107};
  for (int channel = 0; channel < 4; channel++) {
    EXPECT_EQ(uint8_t(exported.bone_keyframes[0].interpolation[channel]), linear[0]);
    EXPECT_EQ(uint8_t(exported.bone_keyframes[0].interpolation[channel + 4]), linear[1]);
    EXPECT_EQ(uint8_t(exported.bone_keyframes[0].interpolation[channel + 8]), linear[2]);
    EXPECT_EQ(uint8_t(exported.bone_keyframes[0].interpolation[channel + 12]), linear[3]);
  }
  BLI_delete(filepath, false, false);
}

/* -------------------------------------------------------------------- */
/** \name BoneConverter Tests
 * \{ */

static constexpr float kConverterEpsilon = 1.0e-5f;

TEST_F(VMDActionContractTest, bone_converter_identity_matrix_local)
{
  /* Bone.matrix_local is backed by Bone::arm_mat in Blender. */
  Bone *bone = static_cast<Bone *>(armature->bonebase.first);
  ASSERT_NE(bone, nullptr);
  unit_m4(bone->arm_mat);

  bPoseChannel *pchan = BKE_pose_channel_find_name(armature_object->pose, "BoneA");
  ASSERT_NE(pchan, nullptr);

  BoneConverter conv;
  conv.compute_from_pose_bone(*pchan, *armature_object);

  /* Location: identity matrix_local → pure (x,z,y) swap with scale. */
  float loc_in[3] = {1.0f, 2.0f, 3.0f};
  float loc_out[3];
  conv.convert_location(loc_in, loc_out, 1.0f);
  EXPECT_NEAR(loc_out[0], 1.0f, kConverterEpsilon);
  EXPECT_NEAR(loc_out[1], 3.0f, kConverterEpsilon);
  EXPECT_NEAR(loc_out[2], 2.0f, kConverterEpsilon);

  /* Scale factor applied correctly. */
  conv.convert_location(loc_in, loc_out, 0.1f);
  EXPECT_NEAR(loc_out[0], 0.1f, kConverterEpsilon);
  EXPECT_NEAR(loc_out[1], 0.3f, kConverterEpsilon);
  EXPECT_NEAR(loc_out[2], 0.2f, kConverterEpsilon);

  /* Rotation: verify the output is normalized (unit quaternion). */
  float rot_in[4] = {0.0f, 1.0f, 0.0f, 0.0f}; /* VMD raw order: x,y,z,w = 180° about Y. */
  float rot_out[4];
  conv.convert_rotation(rot_in, rot_out);
  const float rot_len = rot_out[0] * rot_out[0] + rot_out[1] * rot_out[1] +
                        rot_out[2] * rot_out[2] + rot_out[3] * rot_out[3];
  EXPECT_NEAR(rot_len, 1.0f, kConverterEpsilon);
  EXPECT_TRUE(std::isfinite(rot_out[0]));
  EXPECT_TRUE(std::isfinite(rot_out[1]));
  EXPECT_TRUE(std::isfinite(rot_out[2]));
  EXPECT_TRUE(std::isfinite(rot_out[3]));

  /* 90° about VMD Y (= 90° about Blender Z after Y↔Z swap).
   * VMD: {qx=0, qy=0.7071, qz=0, qw=0.7071}
   * After identity conversion: should represent 90° about Z.
   * Both positive and negative z are valid (quaternion double cover). */
  rot_in[0] = 0.0f;
  rot_in[1] = 0.7071068f;
  rot_in[2] = 0.0f;
  rot_in[3] = 0.7071068f;
  conv.convert_rotation(rot_in, rot_out);
  EXPECT_NEAR(rot_out[0], 0.7071068f, kConverterEpsilon);
  EXPECT_NEAR(std::abs(rot_out[3]), 0.7071068f, kConverterEpsilon);
  EXPECT_NEAR(rot_out[1], 0.0f, kConverterEpsilon);
  EXPECT_NEAR(rot_out[2], 0.0f, kConverterEpsilon);
}

TEST_F(VMDActionContractTest, bone_converter_non_identity_matrix_local)
{
  /* Set Bone.matrix_local to a 90° rotation about X axis. */
  Bone *bone = static_cast<Bone *>(armature->bonebase.first);
  ASSERT_NE(bone, nullptr);
  unit_m4(bone->arm_mat);
  bone->arm_mat[1][1] = 0.0f;
  bone->arm_mat[1][2] = -1.0f;
  bone->arm_mat[2][1] = 1.0f;
  bone->arm_mat[2][2] = 0.0f;

  bPoseChannel *pchan = BKE_pose_channel_find_name(armature_object->pose, "BoneA");
  ASSERT_NE(pchan, nullptr);

  BoneConverter conv;
  conv.compute_from_pose_bone(*pchan, *armature_object);

  /* Location: non-identity matrix_local → no longer pure (x,z,y) swap.
   * Just verify the output is finite and non-zero. */
  float loc_in[3] = {1.0f, 0.0f, 0.0f};
  float loc_out[3];
  conv.convert_location(loc_in, loc_out, 1.0f);
  EXPECT_TRUE(std::isfinite(loc_out[0]));
  EXPECT_TRUE(std::isfinite(loc_out[1]));
  EXPECT_TRUE(std::isfinite(loc_out[2]));

  /* Rotation: verify the output is a valid unit quaternion. */
  float rot_in[4] = {0.0f, 0.0f, 0.0f, 1.0f}; /* VMD raw order: x,y,z,w = identity. */
  float rot_out[4];
  conv.convert_rotation(rot_in, rot_out);
  const float rot_len = rot_out[0] * rot_out[0] + rot_out[1] * rot_out[1] +
                        rot_out[2] * rot_out[2] + rot_out[3] * rot_out[3];
  EXPECT_NEAR(rot_len, 1.0f, kConverterEpsilon);
  EXPECT_TRUE(std::isfinite(rot_out[0]));
  EXPECT_TRUE(std::isfinite(rot_out[1]));
  EXPECT_TRUE(std::isfinite(rot_out[2]));
  EXPECT_TRUE(std::isfinite(rot_out[3]));
}

/** \} */

}  // namespace
}  // namespace blender::io::vmd::tests
