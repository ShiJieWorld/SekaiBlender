/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "vmd_camera_action.hh"

#include "vmd_export.hh"

#include "ANIM_action.hh"
#include "ANIM_action_iterators.hh"
#include "BKE_anim_data.hh"
#include "BKE_camera.h"
#include "BKE_collection.hh"
#include "BKE_gtest_base.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"

#include "DNA_action_types.h"
#include "DNA_camera_types.h"
#include "DNA_object_types.h"

#include "BLI_fileops.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_rotation_c.hh"
#include "BLI_path_utils.hh"
#include "BLI_string.hh"
#include "BLI_tempfile.hh"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>

#include "testing/testing.h"

namespace blender::io::vmd::tests {
namespace {

void write_u32(FILE *file, const uint32_t value)
{
  const uint8_t bytes[4] = {uint8_t(value & 0xff),
                             uint8_t((value >> 8) & 0xff),
                             uint8_t((value >> 16) & 0xff),
                             uint8_t((value >> 24) & 0xff)};
  fwrite(bytes, 1, sizeof(bytes), file);
}

void write_f32(FILE *file, const float value)
{
  uint32_t bits;
  static_assert(sizeof(bits) == sizeof(value));
  memcpy(&bits, &value, sizeof(bits));
  write_u32(file, bits);
}

struct CameraSpec {
  uint32_t frame;
  float distance;
  std::array<float, 3> position;
  std::array<float, 3> rotation;
  uint32_t view_angle;
  bool perspective;
};

std::string make_camera_vmd(const std::initializer_list<CameraSpec> cameras, const char *filename)
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
  write_u32(file, 0); /* morph */
  write_u32(file, uint32_t(cameras.size()));
  for (const CameraSpec &camera : cameras) {
    write_u32(file, camera.frame);
    write_f32(file, camera.distance);
    for (const float value : camera.position) {
      write_f32(file, value);
    }
    for (const float value : camera.rotation) {
      write_f32(file, value);
    }
    const uint8_t interpolation[VMD_CAMERA_INTERP_BYTES] = {
        20, 107, 20, 107, 20, 107, 20, 107, 20, 107, 20, 107,
        20, 107, 20, 107, 20, 107, 20, 107, 20, 107, 20, 107};
    fwrite(interpolation, 1, sizeof(interpolation), file);
    write_u32(file, camera.view_angle);
    fputc(camera.perspective ? 0 : 1, file);
  }
  write_u32(file, 0); /* light */
  write_u32(file, 0); /* self-shadow */
  write_u32(file, 0); /* property */
  fclose(file);
  return filepath;
}

FCurve *find_fcurve(ID &id, const char *path, const int array_index)
{
  const AnimData *anim_data = BKE_animdata_from_id(&id);
  if (anim_data == nullptr || anim_data->action == nullptr) {
    return nullptr;
  }
  FCurve *found = nullptr;
  animrig::foreach_fcurve_in_action(anim_data->action->wrap(), [&](FCurve &fcurve) {
    if (found == nullptr && fcurve.array_index == array_index && fcurve.rna_path() == path)
    {
      found = &fcurve;
    }
  });
  return found;
}

}  // namespace

TEST(VMDReaderCameraTest, parses_camera_records)
{
  const std::string filepath = make_camera_vmd(
      {{0, -45.0f, {1.0f, 2.0f, 3.0f}, {0.1f, 0.2f, 0.3f}, 30, true},
       {12, -40.0f, {4.0f, 5.0f, 6.0f}, {0.4f, 0.5f, 0.6f}, 45, false}},
      "vmd_camera_reader_test.vmd");

  VMDReadReport report;
  const VMDModel model = read_vmd(filepath, &report);
  BLI_delete(filepath.c_str(), false, false);

  ASSERT_TRUE(report.ok());
  ASSERT_EQ(report.camera_frame_count, 2u);
  ASSERT_EQ(model.camera_keyframes.size(), 2u);
  EXPECT_EQ(model.camera_keyframes[0].frame, 0u);
  EXPECT_FLOAT_EQ(model.camera_keyframes[0].distance, -45.0f);
  EXPECT_FLOAT_EQ(model.camera_keyframes[0].position[1], 2.0f);
  EXPECT_FLOAT_EQ(model.camera_keyframes[0].rotation[2], 0.3f);
  EXPECT_EQ(model.camera_keyframes[0].view_angle, 30u);
  EXPECT_TRUE(model.camera_keyframes[0].perspective);
  EXPECT_EQ(model.camera_keyframes[0].interpolation[0], 20u);
  EXPECT_EQ(model.camera_keyframes[0].interpolation[1], 107u);
  EXPECT_EQ(model.camera_keyframes[1].view_angle, 45u);
  EXPECT_FALSE(model.camera_keyframes[1].perspective);
}

class VMDCameraActionTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Collection *collection = nullptr;
  Object *target_empty = nullptr;
  Object *camera = nullptr;

  void SetUp() override
  {
    bmain = BKE_main_new();
    collection = BKE_collection_add(bmain, nullptr, "VMD Camera Test Collection");
  }

  void TearDown() override
  {
    BKE_main_free(bmain);
  }
};

TEST_F(VMDCameraActionTest, builds_native_camera_rig_and_actions)
{
  VMDCameraActionReport result;
  ASSERT_TRUE(create_vmd_camera_rig(
      bmain, *collection, "VMD Camera Test", 0.08f, target_empty, camera, nullptr, result));
  ASSERT_NE(target_empty, nullptr);
  ASSERT_NE(camera, nullptr);
  ASSERT_EQ(camera->parent, target_empty);
  EXPECT_EQ(target_empty->rotmode, ROT_MODE_YXZ);
  EXPECT_EQ(camera->rotmode, ROT_MODE_XYZ);
  EXPECT_FLOAT_EQ(camera->rot[0], float(M_PI_2));

  VMDModel model;
  VMDCameraKeyframe first;
  first.frame = 0;
  first.distance = -45.0f;
  first.position = {1.0f, 2.0f, 3.0f};
  first.rotation = {0.1f, 0.2f, 0.3f};
  first.view_angle = 30;
  first.perspective = true;
  first.interpolation = {20, 107, 20, 107, 20, 107, 20, 107, 20, 107, 20, 107,
                         20, 107, 20, 107, 20, 107, 20, 107, 20, 107, 20, 107};
  VMDCameraKeyframe second = first;
  second.frame = 12;
  second.distance = -40.0f;
  second.position = {4.0f, 5.0f, 6.0f};
  second.rotation = {0.4f, 0.5f, 0.6f};
  second.view_angle = 45;
  second.perspective = false;
  second.interpolation[0] = 10;
  second.interpolation[1] = 20;
  second.interpolation[2] = 30;
  second.interpolation[3] = 40;
  model.camera_keyframes = {first, second};

  VMDCameraActionOptions options;
  options.frame_offset = 30;
  options.use_linear_interpolation = false;
  options.use_vmd_bezier_interpolation = true;
  ASSERT_TRUE(build_vmd_camera_action(
      bmain, *target_empty, *camera, model, "VMD Camera Test Action", options, nullptr, result));

  EXPECT_TRUE(result.parent_action_bound);
  EXPECT_TRUE(result.camera_action_bound);
  EXPECT_TRUE(result.camera_data_action_bound);
  EXPECT_EQ(result.fcurve_count, 10);
  EXPECT_EQ(result.keyframe_count, 20);
  EXPECT_EQ(result.first_frame, 30);
  EXPECT_EQ(result.last_frame, 42);
  EXPECT_EQ(result.bezier_curve_count, 9);

  FCurve *location_x = find_fcurve(target_empty->id, "location", 0);
  ASSERT_NE(location_x, nullptr);
  ASSERT_EQ(location_x->totvert, 2);
  EXPECT_FLOAT_EQ(location_x->bezt[0].vec[1][0], 30.0f);
  EXPECT_FLOAT_EQ(location_x->bezt[0].vec[1][1], 0.08f);
  EXPECT_NEAR(location_x->bezt[0].vec[2][0], 30.0f + 12.0f * 10.0f / 127.0f, 1.0e-4f);
  EXPECT_NEAR(location_x->bezt[1].vec[0][0], 30.0f + 12.0f * 20.0f / 127.0f, 1.0e-4f);

  FCurve *location_y = find_fcurve(target_empty->id, "location", 1);
  ASSERT_NE(location_y, nullptr);
  EXPECT_FLOAT_EQ(location_y->bezt[0].vec[1][1], 0.24f);

  FCurve *distance = find_fcurve(camera->id, "location", 1);
  ASSERT_NE(distance, nullptr);
  ASSERT_EQ(distance->totvert, 2);
  EXPECT_FLOAT_EQ(distance->bezt[0].vec[1][1], -3.6f);

  Camera *camera_data = id_cast<Camera *>(camera->data);
  ASSERT_NE(camera_data, nullptr);
  FCurve *type = find_fcurve(camera_data->id, "type", 0);
  ASSERT_NE(type, nullptr);
  ASSERT_EQ(type->totvert, 2);
  EXPECT_FLOAT_EQ(type->bezt[0].vec[1][1], float(CAM_PERSP));
  EXPECT_FLOAT_EQ(type->bezt[1].vec[1][1], float(CAM_ORTHO));
  EXPECT_EQ(type->bezt[0].ipo, BEZT_IPO_CONST);
}

/* -------------------------------------------------------------------- */
/** \name Camera Export
 * \{ */

namespace {

std::string camera_temp_path(const char *filename)
{
  char temp_dir[FILE_MAX];
  BLI_temp_directory_path_get(temp_dir, sizeof(temp_dir));
  char filepath[FILE_MAX];
  BLI_path_join(filepath, sizeof(filepath), temp_dir, filename);
  return filepath;
}

/* The 20/107 pair is MMD's linear default, repeated across all six camera channels. */
std::array<uint8_t, VMD_CAMERA_INTERP_BYTES> linear_camera_bytes()
{
  std::array<uint8_t, VMD_CAMERA_INTERP_BYTES> result{};
  for (int channel = 0; channel < VMD_CAMERA_INTERP_CHANNELS; channel++) {
    const int base = channel * VMD_CAMERA_INTERP_CHANNEL_BYTES;
    result[base + 0] = 20;  /* ax */
    result[base + 1] = 107; /* bx */
    result[base + 2] = 20;  /* ay */
    result[base + 3] = 107; /* by */
  }
  return result;
}

Object *add_standalone_camera(Main *bmain, Collection *collection, const char *name)
{
  Camera *camera_data = BKE_camera_add(bmain, (std::string(name) + " Data").c_str());
  Object *camera = BKE_object_add_only_object(bmain, OB_CAMERA, name);
  camera->data = id_cast<ID *>(&camera_data->id);
  id_us_plus(&camera_data->id);
  BKE_collection_object_add(bmain, collection, camera);
  camera_data->sensor_fit = CAMERA_SENSOR_FIT_VERT;
  camera_data->type = CAM_PERSP;
  return camera;
}

}  // namespace

/* An imported rig must survive export unchanged: every VMD field and every per-channel Bezier
 * control byte has to come back byte-identical, because export is the exact inverse of import. */
TEST_F(VMDCameraActionTest, camera_rig_round_trips_through_export)
{
  VMDCameraActionReport result;
  ASSERT_TRUE(create_vmd_camera_rig(
      bmain, *collection, "VMD Camera Export", 0.08f, target_empty, camera, nullptr, result));

  VMDCameraKeyframe first;
  first.frame = 0;
  first.distance = -45.0f;
  first.position = {1.0f, 2.0f, 3.0f};
  first.rotation = {0.1f, 0.2f, 0.3f};
  first.view_angle = 30;
  first.perspective = true;
  first.interpolation = linear_camera_bytes();

  /* Distinct {ax, bx, ay, by} per channel so a swapped channel or offset cannot pass. */
  VMDCameraKeyframe second = first;
  second.frame = 12;
  second.distance = -40.0f;
  second.position = {4.0f, 5.0f, 6.0f};
  second.rotation = {0.4f, 0.5f, 0.6f};
  second.view_angle = 45;
  second.perspective = false;
  second.interpolation = {10, 100, 30, 90,  12, 102, 32, 92,  14, 104, 34, 94,
                          16, 106, 36, 96,  18, 108, 38, 98,  20, 110, 40, 100};

  VMDModel model;
  model.camera_keyframes = {first, second};

  VMDCameraActionOptions import_options;
  import_options.frame_offset = 30;
  import_options.use_linear_interpolation = false;
  import_options.use_vmd_bezier_interpolation = true;
  ASSERT_TRUE(build_vmd_camera_action(bmain,
                                      *target_empty,
                                      *camera,
                                      model,
                                      "VMD Camera Export Action",
                                      import_options,
                                      nullptr,
                                      result));

  const std::string filepath = camera_temp_path("vmd_camera_export_round_trip.vmd");
  VMDCameraExportOptions export_options;
  export_options.frame_start = 30;
  export_options.frame_end = 42;
  /* ASCII keeps the fixture independent of the CP932 encoder's platform support. */
  export_options.model_name = "Camera";
  VMDCameraExportReport export_report;
  ASSERT_TRUE(
      export_vmd_camera(*camera, filepath, export_options, nullptr, export_report));
  EXPECT_TRUE(export_report.used_camera_rig);
  EXPECT_EQ(export_report.camera_frame_count, 2);
  EXPECT_EQ(export_report.clamped_angle_count, 0);
  /* An untouched imported rig must export silently. */
  EXPECT_TRUE(export_report.warnings.empty());
  EXPECT_TRUE(export_report.errors.empty());

  VMDReadReport read_report;
  const VMDModel exported = read_vmd(filepath, &read_report);
  BLI_delete(filepath.c_str(), false, false);
  ASSERT_TRUE(read_report.ok());
  ASSERT_EQ(exported.camera_keyframes.size(), 2u);

  const VMDCameraKeyframe *source[2] = {&first, &second};
  for (int index = 0; index < 2; index++) {
    const VMDCameraKeyframe &actual = exported.camera_keyframes[index];
    const VMDCameraKeyframe &expected = *source[index];
    EXPECT_EQ(actual.frame, expected.frame) << "keyframe " << index;
    EXPECT_NEAR(actual.distance, expected.distance, 1.0e-4f) << "keyframe " << index;
    for (int axis = 0; axis < 3; axis++) {
      EXPECT_NEAR(actual.position[axis], expected.position[axis], 1.0e-4f)
          << "keyframe " << index << " position axis " << axis;
      EXPECT_NEAR(actual.rotation[axis], expected.rotation[axis], 1.0e-5f)
          << "keyframe " << index << " rotation axis " << axis;
    }
    EXPECT_EQ(actual.view_angle, expected.view_angle) << "keyframe " << index;
    EXPECT_EQ(actual.perspective, expected.perspective) << "keyframe " << index;
    for (int byte = 0; byte < VMD_CAMERA_INTERP_BYTES; byte++) {
      EXPECT_EQ(actual.interpolation[byte], expected.interpolation[byte])
          << "keyframe " << index << " interpolation byte " << byte;
    }
  }
  EXPECT_EQ(export_report.bezier_segment_count, 1);
}

/* Regression: MikuMikuDance writes Bezier segments whose two control points cross in frame space
 * (ax > bx), giving a curve that is non-monotonic in time. Export used to treat that as invalid
 * and silently fall back to linear, dropping the author's easing on every moving channel of such
 * a segment. The byte patterns below are the three crossed shapes observed in a real motion,
 * SLAY!/Camera.vmd. Bone export shares `bezier_control_from_curve()`, so this covers it too. */
TEST_F(VMDCameraActionTest, crossed_bezier_controls_survive_export)
{
  VMDCameraActionReport result;
  ASSERT_TRUE(create_vmd_camera_rig(
      bmain, *collection, "VMD Crossed Bezier", 0.08f, target_empty, camera, nullptr, result));

  VMDCameraKeyframe first;
  first.frame = 0;
  first.distance = -45.0f;
  first.position = {1.0f, 2.0f, 3.0f};
  first.rotation = {0.1f, 0.2f, 0.3f};
  first.view_angle = 30;
  first.perspective = true;
  first.interpolation = linear_camera_bytes();

  /* Every channel changes value, so every channel's shape has to be recoverable. */
  VMDCameraKeyframe second = first;
  second.frame = 12;
  second.distance = -40.0f;
  second.position = {4.0f, 5.0f, 6.0f};
  second.rotation = {0.4f, 0.5f, 0.6f};
  second.view_angle = 45;
  second.interpolation = {125, 124, 6, 7,  56, 0, 7, 125,  127, 87, 7, 114,
                          125, 124, 6, 7,  56, 0, 7, 125,  127, 87, 7, 114};

  VMDModel model;
  model.camera_keyframes = {first, second};

  VMDCameraActionOptions import_options;
  import_options.use_linear_interpolation = false;
  import_options.use_vmd_bezier_interpolation = true;
  ASSERT_TRUE(build_vmd_camera_action(bmain,
                                      *target_empty,
                                      *camera,
                                      model,
                                      "VMD Crossed Bezier Action",
                                      import_options,
                                      nullptr,
                                      result));

  const std::string filepath = camera_temp_path("vmd_camera_crossed_bezier.vmd");
  VMDCameraExportOptions export_options;
  export_options.frame_start = 0;
  export_options.frame_end = 12;
  export_options.model_name = "Camera";
  VMDCameraExportReport export_report;
  ASSERT_TRUE(export_vmd_camera(*camera, filepath, export_options, nullptr, export_report));

  const VMDModel exported = read_vmd(filepath);
  BLI_delete(filepath.c_str(), false, false);
  ASSERT_EQ(exported.camera_keyframes.size(), 2u);
  for (int byte = 0; byte < VMD_CAMERA_INTERP_BYTES; byte++) {
    EXPECT_EQ(exported.camera_keyframes[1].interpolation[byte], second.interpolation[byte])
        << "crossed control byte " << byte;
  }
}

/* A standalone Camera has no MMD target Empty, so its world pose is re-derived with distance
 * zero. The +90 degree import pitch must cancel exactly for a plain horizontal camera. */
TEST_F(VMDCameraActionTest, standalone_camera_exports_world_pose)
{
  Object *bare = add_standalone_camera(bmain, collection, "Bare Camera");
  bare->rotmode = ROT_MODE_XYZ;
  bare->loc[0] = 0.08f;
  bare->loc[1] = 0.24f;
  bare->loc[2] = 0.16f;
  bare->rot[0] = float(M_PI_2);
  bare->rot[1] = 0.0f;
  bare->rot[2] = 0.0f;

  const std::string filepath = camera_temp_path("vmd_camera_export_standalone.vmd");
  VMDCameraExportOptions export_options;
  export_options.frame_start = 0;
  export_options.frame_end = 30;
  export_options.model_name = "Camera";
  VMDCameraExportReport export_report;
  ASSERT_TRUE(export_vmd_camera(*bare, filepath, export_options, nullptr, export_report));
  EXPECT_FALSE(export_report.used_camera_rig);
  EXPECT_EQ(export_report.camera_frame_count, 2);

  VMDReadReport read_report;
  const VMDModel exported = read_vmd(filepath, &read_report);
  BLI_delete(filepath.c_str(), false, false);
  ASSERT_TRUE(read_report.ok());
  ASSERT_EQ(exported.camera_keyframes.size(), 2u);

  /* A camera with no Action still exports its static pose at both range boundaries. */
  EXPECT_EQ(exported.camera_keyframes[0].frame, 0u);
  EXPECT_EQ(exported.camera_keyframes[1].frame, 30u);
  const VMDCameraKeyframe &key = exported.camera_keyframes[0];
  EXPECT_NEAR(key.position[0], 1.0f, 1.0e-4f);
  EXPECT_NEAR(key.position[1], 2.0f, 1.0e-4f);
  EXPECT_NEAR(key.position[2], 3.0f, 1.0e-4f);
  EXPECT_NEAR(key.distance, 0.0f, 1.0e-6f);
  for (int axis = 0; axis < 3; axis++) {
    EXPECT_NEAR(key.rotation[axis], 0.0f, 1.0e-5f) << "rotation axis " << axis;
  }
  EXPECT_TRUE(key.perspective);
  EXPECT_EQ(key.interpolation, linear_camera_bytes());
}

/* The standalone rotation is a matrix recomposition, so assert the inverse against the importer's
 * forward chain (R_camera = R_empty * Rx(+90)) rather than restating the export math. */
TEST_F(VMDCameraActionTest, standalone_camera_rotation_inverts_the_import_chain)
{
  Object *bare = add_standalone_camera(bmain, collection, "Tilted Camera");
  bare->rotmode = ROT_MODE_XYZ;
  bare->rot[0] = float(M_PI_2) + 0.25f;
  bare->rot[1] = 0.3f;
  bare->rot[2] = -0.4f;

  const std::string filepath = camera_temp_path("vmd_camera_export_tilted.vmd");
  VMDCameraExportOptions export_options;
  export_options.frame_start = 0;
  export_options.frame_end = 0;
  export_options.model_name = "Camera";
  VMDCameraExportReport export_report;
  ASSERT_TRUE(export_vmd_camera(*bare, filepath, export_options, nullptr, export_report));

  VMDReadReport read_report;
  const VMDModel exported = read_vmd(filepath, &read_report);
  BLI_delete(filepath.c_str(), false, false);
  ASSERT_TRUE(read_report.ok());
  ASSERT_EQ(exported.camera_keyframes.size(), 1u);

  /* Replay the importer: MMD rotation -> Empty YXZ Euler -> Empty rotation * child pitch. */
  const std::array<float, 3> mmd = exported.camera_keyframes[0].rotation;
  const float empty_euler[3] = {mmd[0], mmd[2], mmd[1]};
  float empty_rotation[3][3];
  eulO_to_mat3(empty_rotation, empty_euler, ROT_MODE_YXZ);
  const float pitch_euler[3] = {float(M_PI_2), 0.0f, 0.0f};
  float pitch[3][3];
  eulO_to_mat3(pitch, pitch_euler, ROT_MODE_XYZ);
  float recomposed[3][3];
  mul_m3_m3m3(recomposed, empty_rotation, pitch);

  float original[3][3];
  eulO_to_mat3(original, bare->rot, ROT_MODE_XYZ);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(recomposed[i][j], original[i][j], 1.0e-5f) << "matrix element " << i << "," << j;
    }
  }
}

/* A Camera under a foreign hierarchy has local F-Curves that are not the world pose MMD needs.
 * Refuse it instead of exporting a silently wrong shot. */
TEST_F(VMDCameraActionTest, rejects_camera_under_a_foreign_parent)
{
  Object *root = BKE_object_add_only_object(bmain, OB_EMPTY, "Rig Root");
  Object *middle = BKE_object_add_only_object(bmain, OB_EMPTY, "Rig Middle");
  BKE_collection_object_add(bmain, collection, root);
  BKE_collection_object_add(bmain, collection, middle);
  middle->parent = root;
  middle->partype = PAROBJECT;

  Object *bare = add_standalone_camera(bmain, collection, "Nested Camera");
  bare->parent = middle;
  bare->partype = PAROBJECT;

  const std::string filepath = camera_temp_path("vmd_camera_export_rejected.vmd");
  VMDCameraExportOptions export_options;
  export_options.frame_start = 0;
  export_options.frame_end = 30;
  VMDCameraExportReport export_report;
  EXPECT_FALSE(export_vmd_camera(*bare, filepath, export_options, nullptr, export_report));
  EXPECT_FALSE(export_report.success);
  EXPECT_FALSE(export_report.errors.empty());
  EXPECT_FALSE(BLI_exists(filepath.c_str()));
}

/** \} */

TEST_F(VMDCameraActionTest, reuses_existing_camera)
{
  Camera *camera_data = BKE_camera_add(bmain, "Existing Camera Data");
  Object *existing_camera = BKE_object_add_only_object(bmain, OB_CAMERA, "Existing Camera");
  existing_camera->data = id_cast<ID *>(camera_data);
  id_us_plus(&camera_data->id);
  ASSERT_TRUE(BKE_collection_object_add(bmain, collection, existing_camera));

  Object *target_empty = nullptr;
  Object *target_camera = nullptr;
  VMDCameraActionReport result;
  ASSERT_TRUE(create_vmd_camera_rig(bmain,
                                    *collection,
                                    "Reused VMD Camera",
                                    0.08f,
                                    target_empty,
                                    target_camera,
                                    nullptr,
                                    result,
                                    existing_camera));

  EXPECT_EQ(target_camera, existing_camera);
  EXPECT_EQ(existing_camera->parent, target_empty);
  int camera_count = 0;
  for (Object *object = static_cast<Object *>(bmain->objects.first); object != nullptr;
       object = static_cast<Object *>(object->id.next))
  {
    if (object->type == OB_CAMERA) {
      camera_count++;
    }
  }
  EXPECT_EQ(camera_count, 1);
}

}  // namespace blender::io::vmd::tests
