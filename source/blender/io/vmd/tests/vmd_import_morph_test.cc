/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "vmd_import.hh"

#include "ANIM_action_iterators.hh"

#include "BKE_anim_data.hh"
#include "BKE_armature.hh"
#include "BKE_gtest_base.hh"
#include "BKE_idprop.hh"
#include "BKE_key.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_mesh.hh"
#include "BKE_object.hh"
#include "BKE_pose.hh"
#include "BKE_mesh.h"

#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_key_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "ANIM_action.hh"

#include "BLI_fileops.hh"
#include "BLI_listbase.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_path_utils.hh"
#include "BLI_tempfile.hh"
#include "BLI_string.hh"

#include "MEM_guardedalloc.h"

#include "testing/testing.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace blender::io::vmd::tests {
namespace {

class VMDImportMorphTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Object *armature_object = nullptr;
  Object *controller_object = nullptr;
  bArmature *armature = nullptr;
  Mesh *controller_mesh = nullptr;
  Key *controller_key = nullptr;

  void SetUp() override
  {
    bmain = BKE_main_new();
    armature = BKE_armature_add(bmain, "C2_1D_Armature");
    armature_object = BKE_object_add_only_object(bmain, OB_ARMATURE, "C2_1D_ArmatureObject");
    armature_object->data = id_cast<ID *>(armature);
    add_bone("BoneA");
    BKE_pose_ensure(bmain, armature_object, armature, false);

    controller_mesh = BKE_mesh_add(bmain, "PMXMorphControllerMesh");
    controller_mesh->verts_num = 3;
    bke::mesh_ensure_required_data_layers(*controller_mesh);
    const std::array<float3, 3> positions = {
        float3(0.0f, 0.0f, 0.0f), float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f)};
    controller_mesh->vert_positions_for_write().copy_from(positions);
    controller_key = BKE_key_add(bmain, &controller_mesh->id);
    controller_mesh->key = controller_key;
    controller_key->type = KEY_RELATIVE;
    add_key_block("Basis");
    add_key_block("Smile");
    add_key_block("Blink");

    controller_object = BKE_object_add_only_object(bmain, OB_MESH, "PMXMorphController");
    controller_object->data = id_cast<ID *>(controller_mesh);

    Object *root = BKE_object_add_only_object(bmain, OB_EMPTY, "C2_1D_Root");
    armature_object->parent = root;
    controller_object->parent = root;
  }

  void TearDown() override
  {
    BKE_main_free(bmain);
  }

  void add_bone(const char *name)
  {
    Bone *bone = MEM_new<Bone>("C2_1D_TestBone");
    STRNCPY(bone->name, name);
    BLI_addtail(&armature->bonebase, bone);
  }

  KeyBlock *add_key_block(const char *name)
  {
    KeyBlock *key_block = BKE_keyblock_add(controller_key, name);
    BKE_keyblock_convert_from_mesh(controller_mesh, controller_key, key_block);
    return key_block;
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

  struct MorphRecord {
    const char *name;
    uint32_t frame;
    float weight;
  };

  static std::string make_vmd(const std::array<MorphRecord, 3> &morphs, const char *filename)
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
    write_u32(file, 1); /* bone frame count */
    char bone_name[15] = {};
    STRNCPY(bone_name, "BoneA");
    fwrite(bone_name, 1, sizeof(bone_name), file);
    write_u32(file, 0);
    for (int i = 0; i < 3; i++) {
      write_f32(file, 0.0f);
    }
    write_f32(file, 0.0f);
    write_f32(file, 0.0f);
    write_f32(file, 0.0f);
    write_f32(file, 1.0f);
    const uint8_t interpolation[64] = {};
    fwrite(interpolation, 1, sizeof(interpolation), file);

    write_u32(file, uint32_t(morphs.size()));
    for (const MorphRecord &morph : morphs) {
      char morph_name[15] = {};
      STRNCPY(morph_name, morph.name);
      fwrite(morph_name, 1, sizeof(morph_name), file);
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

  static int count_action_fcurves(const ID &id)
  {
    const AnimData *anim_data = BKE_animdata_from_id(&id);
    if (anim_data == nullptr || anim_data->action == nullptr) {
      return 0;
    }
    int count = 0;
    animrig::foreach_fcurve_in_action(anim_data->action->wrap(),
                                      [&](const FCurve & /*fcurve*/) { count++; });
    return count;
  }
};

TEST_F(VMDImportMorphTest, explicit_controller_builds_bone_and_morph_actions)
{
  const std::string filepath = make_vmd(
      std::array<MorphRecord, 3>{{
          {"Smile", 0, 0.0f},
          {"Smile", 10, 0.75f},
          {"Blink", 10, 1.25f},
      }},
      "c2_1d_explicit_controller.vmd");

  VMDImportReport result;
  VMDImportOptions options;
  options.frame_offset = 5;
  ASSERT_TRUE(import_vmd_action_with_morphs(
      bmain, *armature_object, *controller_object, filepath, options, nullptr, result));
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.mapping.mapped_track_count, 1);
  EXPECT_EQ(result.morph_mapping.mapped_track_count, 2);
  EXPECT_EQ(result.action.action_name, "c2_1d_explicit_controller | VMD");
  EXPECT_EQ(result.morph_action.action_name, "c2_1d_explicit_controller | VMD Morph");
  EXPECT_TRUE(result.action.action_bound);
  EXPECT_TRUE(result.morph_action.action_bound);
  EXPECT_EQ(result.action.first_frame, 5);
  EXPECT_EQ(result.action.last_frame, 5);
  EXPECT_EQ(result.morph_action.first_frame, 5);
  EXPECT_EQ(result.morph_action.last_frame, 15);
  EXPECT_EQ(result.morph_action.fcurve_count, 2);
  EXPECT_EQ(result.morph_action.keyframe_count, 3);
  EXPECT_EQ(count_action_fcurves(armature_object->id), 7);
  EXPECT_EQ(count_action_fcurves(controller_key->id), 2);
  const AnimData *morph_anim_data = BKE_animdata_from_id(&controller_key->id);
  ASSERT_NE(morph_anim_data, nullptr);
  ASSERT_NE(morph_anim_data->action, nullptr);
  EXPECT_EQ(std::string(morph_anim_data->action->id.name + 2),
            "c2_1d_explicit_controller | VMD Morph");

  BLI_delete(filepath.c_str(), false, false);
}

TEST_F(VMDImportMorphTest, all_missing_morphs_skip_without_empty_action)
{
  const std::string filepath = make_vmd(
      std::array<MorphRecord, 3>{{
          {"Missing", 0, 1.0f},
          {"Missing", 10, 0.0f},
          {"OtherMissing", 20, 0.5f},
      }},
      "c2_1d_all_missing.vmd");

  VMDImportReport result;
  ASSERT_TRUE(import_vmd_action_with_morphs(
      bmain, *armature_object, *controller_object, filepath, {}, nullptr, result));
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.morph_mapping.mapped_track_count, 0);
  EXPECT_EQ(result.morph_mapping.missing_track_count, 2);
  EXPECT_TRUE(result.morph_action.skipped);
  EXPECT_FALSE(result.morph_action.action_bound);
  EXPECT_EQ(count_action_fcurves(controller_key->id), 0);
  EXPECT_EQ(count_action_fcurves(armature_object->id), 7);

  BLI_delete(filepath.c_str(), false, false);
}

TEST_F(VMDImportMorphTest, invalid_controller_does_not_create_actions)
{
  const std::string filepath = make_vmd(
      std::array<MorphRecord, 3>{{
          {"Smile", 0, 1.0f},
          {"Smile", 10, 0.0f},
          {"Blink", 20, 0.5f},
      }},
      "c2_1d_invalid_controller.vmd");

  Mesh *mesh_without_key = BKE_mesh_add(bmain, "NoKeyMesh");
  Object *invalid_controller = BKE_object_add_only_object(bmain, OB_MESH, "InvalidController");
  invalid_controller->data = id_cast<ID *>(mesh_without_key);

  VMDImportReport result;
  EXPECT_FALSE(import_vmd_action_with_morphs(
      bmain, *armature_object, *invalid_controller, filepath, {}, nullptr, result));
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.errors.size() > 0);
  EXPECT_EQ(count_action_fcurves(armature_object->id), 0);
  EXPECT_EQ(count_action_fcurves(controller_key->id), 0);

  BLI_delete(filepath.c_str(), false, false);
}

TEST_F(VMDImportMorphTest, group_raw_channel_driven_by_vmd)
{
  /* Add a Group Morph raw channel to the controller (C2-2C creates these). */
  KeyBlock *group_key_block = BKE_keyblock_add(controller_key, "GroupA");
  BKE_keyblock_convert_from_mesh(controller_mesh, controller_key, group_key_block);
  ASSERT_NE(group_key_block, nullptr);

  /* Persist a minimal mmd_pmx_morph_definition on the controller object so the
   * VMD importer can annotate Group raw channels (C2-2E contract). morph_type 0
   * is PMX Group Morph; controller_channel true marks it as a real raw channel. */
  IDProperty *controller_props = IDP_ID_system_properties_ensure(&controller_object->id);
  IDProperty *definition =
      blender::bke::idprop::create_group("mmd_pmx_morph_definition").release();
  IDP_AddToGroup(definition, IDP_NewInt(1, "schema_version"));
  IDProperty *channels = IDP_NewIDPArray("channels");
  IDProperty *channel = blender::bke::idprop::create_group("channel").release();
  IDP_AddToGroup(channel, IDP_NewInt(0, "pmx_morph_index"));
  IDP_AddToGroup(channel, IDP_NewInt(0, "morph_type"));
  IDP_AddToGroup(channel, IDP_NewString("GroupA", "controller_key_name"));
  IDP_AddToGroup(channel, IDP_NewInt(1, "controller_channel"));
  IDP_AddToGroup(channel, IDP_NewInt(0, "vertex_output"));
  IDP_ResizeIDPArray(channels, 1);
  IDP_SetIndexArray(channels, 0, channel);
  MEM_delete(channel);
  IDP_AddToGroup(definition, channels);
  IDP_AddToGroup(controller_props, definition);

  const std::string filepath = make_vmd(
      std::array<MorphRecord, 3>{{
          {"GroupA", 0, 0.0f},
          {"GroupA", 10, 1.0f},
          {"GroupA", 20, 0.5f},
      }},
      "c2_2e_group_raw.vmd");

  VMDImportReport result;
  ASSERT_TRUE(import_vmd_action_with_morphs(
      bmain, *armature_object, *controller_object, filepath, {}, nullptr, result));
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.morph_mapping.mapped_track_count, 1);
  ASSERT_EQ(result.morph_mapping.mapped_tracks.size(), size_t(1));
  EXPECT_TRUE(result.morph_mapping.mapped_tracks[0].is_group_target);
  EXPECT_TRUE(result.morph_mapping.mapped_tracks[0].is_group_no_vertex_output);
  EXPECT_EQ(result.morph_action.fcurve_count, 1);
  EXPECT_EQ(count_action_fcurves(controller_key->id), 1);

  const AnimData *morph_anim_data = BKE_animdata_from_id(&controller_key->id);
  ASSERT_NE(morph_anim_data, nullptr);
  ASSERT_NE(morph_anim_data->action, nullptr);
  bool found_group_curve = false;
  animrig::foreach_fcurve_in_action(morph_anim_data->action->wrap(),
                                    [&](const FCurve &fcurve) {
                                      if (fcurve.rna_path != nullptr &&
                                          strstr(fcurve.rna_path, "GroupA") != nullptr) {
                                        found_group_curve = true;
                                      }
                                    });
  EXPECT_TRUE(found_group_curve);

  BLI_delete(filepath.c_str(), false, false);
}

}  // namespace
}  // namespace blender::io::vmd::tests
