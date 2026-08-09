/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "vmd_morph_action.hh"

#include "ANIM_action.hh"
#include "ANIM_action_iterators.hh"

#include "BKE_anim_data.hh"
#include "BKE_fcurve.hh"
#include "BKE_gtest_base.hh"
#include "BKE_key.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"

#include "DNA_action_types.h"
#include "DNA_anim_types.h"
#include "DNA_key_types.h"
#include "DNA_mesh_types.h"

#include "BLI_math_vector_types.hh"
#include "BLI_string.hh"

#include "testing/testing.h"

#include <array>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>

namespace blender::io::vmd::tests {
namespace {

class VMDMorphActionTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Mesh *mesh = nullptr;
  Key *key = nullptr;

  void SetUp() override
  {
    bmain = BKE_main_new();
    mesh = BKE_mesh_add(bmain, "C2_1C_TestMesh");
    mesh->verts_num = 3;
    bke::mesh_ensure_required_data_layers(*mesh);
    const std::array<float3, 3> positions = {
        float3(0.0f, 0.0f, 0.0f), float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f)};
    mesh->vert_positions_for_write().copy_from(positions);

    key = BKE_key_add(bmain, &mesh->id);
    mesh->key = key;
    key->type = KEY_RELATIVE;
    add_key_block("Basis");
    add_key_block("Smile");
    add_key_block("Blink");
  }

  void TearDown() override
  {
    BKE_main_free(bmain);
  }

  KeyBlock *add_key_block(const char *name)
  {
    KeyBlock *key_block = BKE_keyblock_add(key, name);
    BKE_keyblock_convert_from_mesh(mesh, key, key_block);
    return key_block;
  }

  static VMDMorphKeyframe morph_keyframe(const char *name,
                                         const uint32_t frame,
                                         const float weight)
  {
    VMDMorphKeyframe keyframe;
    keyframe.morph_name = name;
    keyframe.frame = frame;
    keyframe.weight = weight;
    return keyframe;
  }

  VMDModel make_model(std::initializer_list<VMDMorphKeyframe> keyframes) const
  {
    VMDModel model;
    model.morph_keyframes.assign(keyframes.begin(), keyframes.end());
    model.morph_frame_count = uint32_t(model.morph_keyframes.size());
    return model;
  }

  VMDMorphActionReport build(const VMDModel &model,
                             const std::vector<std::string> &target_names,
                             const VMDMorphActionOptions &options = {})
  {
    const VMDMorphMappingReport mapping = map_morph_tracks(model, target_names);
    VMDMorphActionReport result;
    const bool built = build_vmd_morph_action(
        bmain, *key, model, mapping, "C2_1C Morph Action", options, nullptr, result);
    EXPECT_EQ(built, result.success);
    return result;
  }

  const AnimData *anim_data() const
  {
    return BKE_animdata_from_id(&key->id);
  }

  int count_fcurves() const
  {
    const AnimData *adt = anim_data();
    if (adt == nullptr || adt->action == nullptr) {
      return 0;
    }
    int count = 0;
    animrig::foreach_fcurve_in_action(adt->action->wrap(),
                                      [&](const FCurve & /*fcurve*/) { count++; });
    return count;
  }

  FCurve *find_fcurve(const std::string &path) const
  {
    const AnimData *adt = anim_data();
    if (adt == nullptr || adt->action == nullptr) {
      return nullptr;
    }
    FCurve *found = nullptr;
    animrig::foreach_fcurve_in_action(adt->action->wrap(), [&](FCurve &fcurve) {
      if (found == nullptr && fcurve.array_index == 0 && fcurve.rna_path() == path) {
        found = &fcurve;
      }
    });
    return found;
  }
};

TEST_F(VMDMorphActionTest, writes_key_id_action_and_value_curves)
{
  const VMDModel model = make_model({
      morph_keyframe("Smile", 2, 0.25f),
      morph_keyframe("Blink", 5, -0.2f),
      morph_keyframe("Smile", 10, 1.5f),
  });

  VMDMorphActionOptions options;
  options.frame_offset = -1;
  const VMDMorphActionReport result = build(model, {"Basis", "Smile", "Blink"}, options);

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.action_bound);
  EXPECT_EQ(result.mapped_track_count, 2);
  EXPECT_EQ(result.fcurve_count, 2);
  EXPECT_EQ(result.keyframe_count, 3);
  EXPECT_EQ(result.first_frame, 1);
  EXPECT_EQ(result.last_frame, 9);
  EXPECT_EQ(count_fcurves(), 2);

  const AnimData *adt = anim_data();
  ASSERT_NE(adt, nullptr);
  ASSERT_NE(adt->action, nullptr);
  EXPECT_STREQ(adt->action->id.name + 2, "C2_1C Morph Action");
  EXPECT_EQ(adt->action->wrap().get_frame_range(), (float2{1.0f, 9.0f}));
  EXPECT_NE(adt->slot_handle, 0);

  FCurve *smile_curve = find_fcurve("key_blocks[\"Smile\"].value");
  ASSERT_NE(smile_curve, nullptr);
  ASSERT_EQ(smile_curve->totvert, 2);
  EXPECT_EQ(smile_curve->bezt[0].ipo, BEZT_IPO_LIN);
  EXPECT_FLOAT_EQ(smile_curve->bezt[0].vec[1][0], 1.0f);
  EXPECT_FLOAT_EQ(smile_curve->bezt[0].vec[1][1], 0.25f);
  EXPECT_FLOAT_EQ(smile_curve->bezt[1].vec[1][0], 9.0f);
  EXPECT_FLOAT_EQ(smile_curve->bezt[1].vec[1][1], 1.5f);

  FCurve *blink_curve = find_fcurve("key_blocks[\"Blink\"].value");
  ASSERT_NE(blink_curve, nullptr);
  ASSERT_EQ(blink_curve->totvert, 1);
  EXPECT_FLOAT_EQ(blink_curve->bezt[0].vec[1][0], 4.0f);
  EXPECT_FLOAT_EQ(blink_curve->bezt[0].vec[1][1], -0.2f);
}

TEST_F(VMDMorphActionTest, zero_mapped_tracks_do_not_create_action)
{
  const VMDModel model = make_model({morph_keyframe("Unknown", 3, 0.5f)});
  const VMDMorphActionReport result = build(model, {"Basis", "Smile", "Blink"});

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.action_bound);
  EXPECT_TRUE(result.errors.empty());
  EXPECT_FALSE(result.warnings.empty());
  EXPECT_EQ(count_fcurves(), 0);
  EXPECT_EQ(anim_data(), nullptr);
}

TEST_F(VMDMorphActionTest, invalid_mapping_does_not_create_action)
{
  const VMDModel model = make_model({morph_keyframe("Smile", 3, 0.5f)});
  VMDMorphMappingReport mapping = map_morph_tracks(model, {"Basis", "Smile"});
  ASSERT_TRUE(mapping.mapping_valid);
  mapping.mapping_valid = false;

  VMDMorphActionReport result;
  EXPECT_FALSE(build_vmd_morph_action(
      bmain, *key, model, mapping, "C2_1C Invalid", {}, nullptr, result));
  EXPECT_FALSE(result.action_bound);
  EXPECT_EQ(count_fcurves(), 0);
  EXPECT_EQ(anim_data(), nullptr);
}

TEST_F(VMDMorphActionTest, existing_action_is_protected)
{
  const VMDModel model = make_model({morph_keyframe("Smile", 3, 0.5f)});
  const VMDMorphActionReport first = build(model, {"Basis", "Smile"});
  ASSERT_TRUE(first.success);
  AnimData *adt = BKE_animdata_from_id(&key->id);
  ASSERT_NE(adt, nullptr);
  bAction *existing_action = adt->action;

  VMDMorphActionOptions options;
  options.replace_existing_action = false;
  VMDMorphActionReport second;
  EXPECT_FALSE(build_vmd_morph_action(
      bmain, *key, model, map_morph_tracks(model, {"Basis", "Smile"}), "C2_1C Replacement", options, nullptr, second));
  EXPECT_FALSE(second.action_bound);
  EXPECT_EQ(BKE_animdata_from_id(&key->id)->action, existing_action);
  EXPECT_EQ(count_fcurves(), 1);
}

TEST_F(VMDMorphActionTest, rejects_invalid_keyframe_input_before_binding)
{
  VMDModel model = make_model({morph_keyframe("Smile", 3, 0.5f)});
  VMDMorphMappingReport mapping = map_morph_tracks(model, {"Basis", "Smile"});
  ASSERT_TRUE(mapping.mapping_valid);
  ASSERT_EQ(mapping.mapped_tracks.size(), 1);
  mapping.mapped_tracks[0].keyframe_indices.push_back(99);

  VMDMorphActionReport result;
  EXPECT_FALSE(build_vmd_morph_action(
      bmain, *key, model, mapping, "C2_1C Invalid Input", {}, nullptr, result));
  EXPECT_FALSE(result.action_bound);
  EXPECT_EQ(count_fcurves(), 0);
  EXPECT_EQ(anim_data(), nullptr);
}

}  // namespace
}  // namespace blender::io::vmd::tests
