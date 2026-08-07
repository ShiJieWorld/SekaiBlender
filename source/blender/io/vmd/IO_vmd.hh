/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace blender::io::vmd {

constexpr int VMD_BONE_NAME_BYTES = 15;
constexpr int VMD_MORPH_NAME_BYTES = 15;
constexpr int VMD_MODEL_NAME_BYTES = 20;
constexpr int VMD_BONE_RECORD_BYTES = 111;
constexpr int VMD_MORPH_RECORD_BYTES = 23;
constexpr int VMD_CAMERA_RECORD_BYTES = 61;
constexpr int VMD_LIGHT_RECORD_BYTES = 28;
constexpr int VMD_SHADOW_RECORD_BYTES = 9;
constexpr int VMD_PROPERTY_FIXED_RECORD_BYTES = 9;
constexpr int VMD_PROPERTY_IK_NAME_BYTES = 20;
constexpr int VMD_PROPERTY_IK_RECORD_BYTES = 21;

struct VMDBoneKeyframe {
  std::string bone_name;
  uint32_t frame = 0;
  std::array<float, 3> translation{};
  std::array<float, 4> rotation{}; /* VMD order: x, y, z, w. */
  std::array<int8_t, 64> interpolation{};
  uint64_t source_offset = 0;
};

struct VMDMorphKeyframe {
  std::string morph_name;
  uint32_t frame = 0;
  float weight = 0.0f;
  uint64_t source_offset = 0;
};

/* Number of interpolation channels in a camera keyframe and the byte stride of one channel.
 * A camera record packs 6 channels (x, y, z, rotation, distance, view angle) as 4 bytes each
 * (ax, bx, ay, by), unlike a bone keyframe which uses 4 channels of 16 bytes. */
constexpr int VMD_CAMERA_INTERP_CHANNELS = 6;
constexpr int VMD_CAMERA_INTERP_CHANNEL_BYTES = 4;
constexpr int VMD_CAMERA_INTERP_BYTES = VMD_CAMERA_INTERP_CHANNELS *
                                        VMD_CAMERA_INTERP_CHANNEL_BYTES;

/* Interpolation channel order inside VMDCameraKeyframe::interpolation. */
enum VMDCameraInterpChannel {
  VMD_CAMERA_INTERP_X = 0,
  VMD_CAMERA_INTERP_Y = 1,
  VMD_CAMERA_INTERP_Z = 2,
  VMD_CAMERA_INTERP_ROTATION = 3,
  VMD_CAMERA_INTERP_DISTANCE = 4,
  VMD_CAMERA_INTERP_ANGLE = 5,
};

/**
 * One MMD camera keyframe, 61 bytes on disk:
 * frame (4) + distance (4) + position (12) + rotation (12) + interpolation (24) +
 * view angle (4) + perspective flag (1).
 *
 * `position` is the camera's look-at center, not the lens origin. `distance` offsets the lens
 * backwards along the rotated view axis and is negative in most MMD motions. `rotation` holds
 * MMD Euler angles in radians. All values stay in MMD coordinate space; conversion to Blender
 * happens in the camera Action stage.
 */
struct VMDCameraKeyframe {
  uint32_t frame = 0;
  float distance = 0.0f;
  std::array<float, 3> position{};
  std::array<float, 3> rotation{};
  std::array<uint8_t, VMD_CAMERA_INTERP_BYTES> interpolation{};
  uint32_t view_angle = 0;
  bool perspective = true;
  uint64_t source_offset = 0;
};

struct VMDPropertyIKState {
  std::string bone_name;
  bool enabled = false;
};

struct VMDPropertyKeyframe {
  uint32_t frame = 0;
  bool visible = true;
  std::vector<VMDPropertyIKState> ik_states;
  uint64_t source_offset = 0;
};

struct VMDHeader {
  std::string signature;
  std::string model_name;
  bool compatible = false;
};

struct VMDModel {
  VMDHeader header;
  std::vector<VMDBoneKeyframe> bone_keyframes;
  std::vector<VMDMorphKeyframe> morph_keyframes;
  std::vector<VMDCameraKeyframe> camera_keyframes;
  std::vector<VMDPropertyKeyframe> property_keyframes;
  uint64_t file_size = 0;
  uint64_t parse_end_offset = 0;
  uint32_t morph_frame_count = 0;
  uint32_t camera_frame_count = 0;
  uint32_t light_frame_count = 0;
  uint32_t shadow_frame_count = 0;
  bool has_unsupported_sections = false;
};

struct VMDReadReport {
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
  uint64_t file_size = 0;
  uint64_t parse_end_offset = 0;
  uint32_t bone_frame_count = 0;
  uint32_t morph_frame_count = 0;
  uint32_t camera_frame_count = 0;
  uint32_t property_frame_count = 0;

  bool ok() const { return errors.empty(); }
};

struct VMDWriteReport {
  bool success = false;
  uint32_t bone_frame_count = 0;
  uint32_t morph_frame_count = 0;
  uint32_t camera_frame_count = 0;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

class VMDReaderError : public std::runtime_error {
 public:
  explicit VMDReaderError(const std::string &message) : std::runtime_error(message) {}
};

VMDModel read_vmd(const std::string &filepath, VMDReadReport *report = nullptr);
bool write_vmd(const std::string &filepath,
               const VMDModel &model,
               VMDWriteReport *report = nullptr);

}  // namespace blender::io::vmd
