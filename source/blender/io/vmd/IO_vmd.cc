/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "IO_vmd.hh"

#include "BLI_fileops.hh"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <set>
#include <utility>

#ifdef _WIN32
#  include <windows.h>
#endif

namespace blender::io::vmd {
namespace {

constexpr char kVMDSignature[] = "Vocaloid Motion Data 0002";
constexpr size_t kHeaderBytes = 30 + VMD_MODEL_NAME_BYTES;
constexpr uint32_t kMaxBoneFrames = 50000000;
constexpr uint32_t kMaxMorphFrames = 50000000;
constexpr uint32_t kMaxCameraFrames = 50000000;
constexpr uint32_t kMaxLightFrames = 50000000;
constexpr uint32_t kMaxShadowFrames = 50000000;
constexpr uint32_t kMaxPropertyFrames = 50000000;
constexpr uint32_t kMaxIKStatesPerProperty = 100000;

class Reader {
 public:
  Reader(const uint8_t *data, const size_t size) : data_(data), size_(size) {}

  size_t pos() const { return pos_; }
  size_t size() const { return size_; }
  size_t remaining() const { return size_ - pos_; }

  void require(const size_t bytes, const char *field)
  {
    if (pos_ > size_ || bytes > size_ - pos_) {
      throw VMDReaderError(std::string("Unexpected EOF reading ") + field + " at offset " +
                           std::to_string(pos_));
    }
  }

  uint8_t byte(const char *field = "byte")
  {
    require(1, field);
    return data_[pos_++];
  }

  uint32_t u32(const char *field)
  {
    require(4, field);
    const uint32_t value = uint32_t(data_[pos_]) | (uint32_t(data_[pos_ + 1]) << 8) |
                           (uint32_t(data_[pos_ + 2]) << 16) | (uint32_t(data_[pos_ + 3]) << 24);
    pos_ += 4;
    return value;
  }

  float f32(const char *field)
  {
    const size_t offset = pos_;
    const uint32_t raw = u32(field);
    float value;
    std::memcpy(&value, &raw, sizeof(value));
    if (!std::isfinite(value)) {
      throw VMDReaderError(std::string("Non-finite value for ") + field + " at offset " +
                           std::to_string(offset));
    }
    return value;
  }

  std::string bytes(const size_t count, const char *field)
  {
    require(count, field);
    std::string result(reinterpret_cast<const char *>(data_ + pos_), count);
    pos_ += count;
    return result;
  }

  void skip(const size_t count, const char *field)
  {
    require(count, field);
    pos_ += count;
  }

 private:
  const uint8_t *data_;
  size_t size_;
  size_t pos_ = 0;
};

bool encode_vmd_string(const std::string &utf8,
                       const size_t field_size,
                       std::string &r_bytes,
                       bool &r_used_gbk,
                       bool &r_truncated)
{
  r_bytes.clear();
  r_used_gbk = false;
  r_truncated = false;
#ifdef _WIN32
  if (utf8.empty()) {
    r_bytes.resize(field_size, '\0');
    return true;
  }
  const int wide_count = MultiByteToWideChar(
      CP_UTF8, MB_ERR_INVALID_CHARS, utf8.data(), int(utf8.size()), nullptr, 0);
  if (wide_count <= 0) {
    return false;
  }
  std::wstring wide(size_t(wide_count), L'\0');
  if (MultiByteToWideChar(CP_UTF8,
                          MB_ERR_INVALID_CHARS,
                          utf8.data(),
                          int(utf8.size()),
                          wide.data(),
                          wide_count) != wide_count)
  {
    return false;
  }
  auto encode = [&](const UINT codepage, std::string &result) {
    BOOL used_default = FALSE;
    const int count = WideCharToMultiByte(codepage,
                                          WC_NO_BEST_FIT_CHARS,
                                          wide.data(),
                                          wide_count,
                                          nullptr,
                                          0,
                                          nullptr,
                                          &used_default);
    if (count <= 0 || used_default) {
      return false;
    }
    result.assign(size_t(count), '\0');
    used_default = FALSE;
    return WideCharToMultiByte(codepage,
                               WC_NO_BEST_FIT_CHARS,
                               wide.data(),
                               wide_count,
                               result.data(),
                               count,
                               nullptr,
                               &used_default) == count &&
           !used_default;
  };
  if (!encode(932, r_bytes)) {
    if (!encode(936, r_bytes)) {
      return false;
    }
    r_used_gbk = true;
  }
#else
  for (const unsigned char value : utf8) {
    if (value >= 0x80) {
      return false;
    }
  }
  r_bytes = utf8;
#endif
  if (r_bytes.size() > field_size) {
    r_truncated = true;
    r_bytes.resize(field_size);
    const auto is_lead = [&](const unsigned char value) {
      if (r_used_gbk) {
        return value >= 0x81 && value <= 0xfe;
      }
      return (value >= 0x81 && value <= 0x9f) || (value >= 0xe0 && value <= 0xfc);
    };
    size_t i = 0;
    while (i < r_bytes.size()) {
      const unsigned char value = static_cast<unsigned char>(r_bytes[i]);
      const size_t char_size = is_lead(value) ? 2 : 1;
      if (i + char_size > r_bytes.size()) {
        r_bytes.resize(i);
        break;
      }
      i += char_size;
    }
  }
  r_bytes.resize(field_size, '\0');
  return true;
}

void write_u32(FILE *file, const uint32_t value)
{
  const uint8_t bytes[4] = {uint8_t(value),
                            uint8_t(value >> 8),
                            uint8_t(value >> 16),
                            uint8_t(value >> 24)};
  fwrite(bytes, 1, sizeof(bytes), file);
}

void write_f32(FILE *file, const float value)
{
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  write_u32(file, bits);
}

/* Decode VMD strings as UTF-8 without a third-party dependency. CP932 is the
 * standard encoding, but Chinese VMD tools also commonly write bone names in
 * GBK/CP936. Prefer strict CP932 and use strict GBK only when CP932 rejects the
 * bytes; this preserves valid Chinese names without silently replacing bytes. */
std::string decode_vmd_string(const std::string &raw,
                              const size_t offset,
                              bool *r_used_gbk,
                              bool *r_truncated_tail)
{
  std::string bytes = raw;
  const size_t terminator = bytes.find('\0');
  if (terminator != std::string::npos) {
    bytes.resize(terminator);
  }
  if (r_used_gbk != nullptr) {
    *r_used_gbk = false;
  }
  if (r_truncated_tail != nullptr) {
    *r_truncated_tail = false;
  }
  if (bytes.empty()) {
    return {};
  }

#ifdef _WIN32
  auto try_decode = [&](const UINT codepage,
                        std::string &r_result,
                        std::wstring *r_wide_result) {
    const int wide_count = MultiByteToWideChar(codepage,
                                               MB_ERR_INVALID_CHARS,
                                               bytes.data(),
                                               int(bytes.size()),
                                               nullptr,
                                               0);
    if (wide_count <= 0) {
      return false;
    }
    std::wstring wide(size_t(wide_count), L'\0');
    if (MultiByteToWideChar(codepage,
                            MB_ERR_INVALID_CHARS,
                            bytes.data(),
                            int(bytes.size()),
                            wide.data(),
                            wide_count) != wide_count) {
      return false;
    }
    const int utf8_count = WideCharToMultiByte(CP_UTF8,
                                               WC_ERR_INVALID_CHARS,
                                               wide.data(),
                                               wide_count,
                                               nullptr,
                                               0,
                                               nullptr,
                                               nullptr);
    if (utf8_count <= 0) {
      return false;
    }
    r_result.assign(size_t(utf8_count), '\0');
    const bool success = WideCharToMultiByte(CP_UTF8,
                                             WC_ERR_INVALID_CHARS,
                                             wide.data(),
                                             wide_count,
                                             r_result.data(),
                                             utf8_count,
                                             nullptr,
                                             nullptr) == utf8_count;
    if (success && r_wide_result != nullptr) {
      *r_wide_result = std::move(wide);
    }
    return success;
  };

  std::string cp932_result, gbk_result;
  std::wstring cp932_wide, gbk_wide;
  const bool cp932_ok = try_decode(932, cp932_result, &cp932_wide);
  const bool gbk_ok = try_decode(936, gbk_result, &gbk_wide);
  const bool cp932_has_private_use = std::any_of(
      cp932_wide.begin(), cp932_wide.end(), [](const wchar_t value) {
        return value >= 0xe000 && value <= 0xf8ff;
      });
  const bool gbk_has_cjk = std::any_of(gbk_wide.begin(), gbk_wide.end(), [](const wchar_t value) {
    return (value >= 0x3400 && value <= 0x4dbf) || (value >= 0x4e00 && value <= 0x9fff);
  });
  /* Some valid GBK names are also accepted by CP932, but decode there as
   * half-width characters plus CP932 user-defined/private-use glyphs. Prefer
   * the coherent GBK CJK result for this otherwise ambiguous byte sequence. */
  if (gbk_ok && cp932_ok && cp932_has_private_use && gbk_has_cjk) {
    if (r_used_gbk != nullptr) {
      *r_used_gbk = true;
    }
    return gbk_result;
  }
  if (cp932_ok) {
    return cp932_result;
  }
  if (gbk_ok) {
    if (r_used_gbk != nullptr) {
      *r_used_gbk = true;
    }
    return gbk_result;
  }
  const auto is_cp932_lead_byte = [](const unsigned char value) {
    return (value >= 0x81 && value <= 0x9f) || (value >= 0xe0 && value <= 0xfc);
  };
  if (is_cp932_lead_byte(static_cast<unsigned char>(bytes.back()))) {
    bytes.pop_back();
    if (!bytes.empty() && try_decode(932, cp932_result, nullptr)) {
      if (r_truncated_tail != nullptr) {
        *r_truncated_tail = true;
      }
      return cp932_result;
    }
  }
  throw VMDReaderError("Invalid CP932/GBK string at offset " + std::to_string(offset));
#else
  for (const unsigned char value : bytes) {
    if (value >= 0x80) {
      throw VMDReaderError("CP932/GBK decoding requires a platform converter at offset " +
                           std::to_string(offset));
    }
  }
  return bytes;
#endif
}

uint32_t read_count(Reader &reader,
                    const char *name,
                    const uint32_t hard_limit,
                    const size_t minimum_bytes_per_item)
{
  const size_t offset = reader.pos();
  const uint32_t count = reader.u32(name);
  if (count > hard_limit) {
    throw VMDReaderError(std::string("Count exceeds limit for ") + name + " at offset " +
                         std::to_string(offset) + ": " + std::to_string(count));
  }
  if (minimum_bytes_per_item != 0 &&
      size_t(count) > reader.remaining() / minimum_bytes_per_item) {
    throw VMDReaderError(std::string("Count cannot fit remaining data for ") + name +
                         " at offset " + std::to_string(offset) + ": " + std::to_string(count));
  }
  return count;
}

void skip_fixed_records(Reader &reader,
                        const char *name,
                        const uint32_t count,
                        const size_t record_bytes)
{
  if (size_t(count) > reader.remaining() / record_bytes) {
    throw VMDReaderError(std::string("Truncated ") + name + " records at offset " +
                         std::to_string(reader.pos()));
  }
  reader.skip(size_t(count) * record_bytes, name);
}

}  // namespace

VMDModel read_vmd(const std::string &filepath, VMDReadReport *report)
{
  VMDReadReport local_report;
  VMDReadReport &out_report = report ? *report : local_report;
  out_report = {};

  FILE *file = BLI_fopen(filepath.c_str(), "rb");
  if (!file) {
    throw VMDReaderError("Cannot open VMD file: " + filepath);
  }
  if (fseek(file, 0, SEEK_END) != 0) {
    fclose(file);
    throw VMDReaderError("Cannot seek VMD file: " + filepath);
  }
  const long file_size_long = ftell(file);
  if (file_size_long < 0) {
    fclose(file);
    throw VMDReaderError("Cannot determine VMD file size: " + filepath);
  }
  if (fseek(file, 0, SEEK_SET) != 0) {
    fclose(file);
    throw VMDReaderError("Cannot rewind VMD file: " + filepath);
  }
  const size_t file_size = size_t(file_size_long);
  std::vector<uint8_t> data(file_size);
  if (file_size != 0 && fread(data.data(), 1, file_size, file) != file_size) {
    fclose(file);
    throw VMDReaderError("Cannot read VMD file: " + filepath);
  }
  fclose(file);

  out_report.file_size = file_size;
  if (file_size < kHeaderBytes + 4) {
    throw VMDReaderError("VMD file is shorter than its fixed header");
  }

  Reader reader(data.data(), data.size());
  VMDModel model;
  model.file_size = file_size;

  const size_t signature_offset = reader.pos();
  const std::string signature_raw = reader.bytes(30, "VMD signature");
  const size_t signature_end = signature_raw.find('\0');
  const std::string signature = signature_raw.substr(
      0, signature_end == std::string::npos ? signature_raw.size() : signature_end);
  if (signature != kVMDSignature) {
    throw VMDReaderError("Invalid VMD signature at offset " + std::to_string(signature_offset));
  }
  model.header.signature = signature;
  const size_t model_name_offset = reader.pos();
  bool model_name_truncated = false;
  model.header.model_name = decode_vmd_string(
      reader.bytes(VMD_MODEL_NAME_BYTES, "VMD model name"),
      model_name_offset,
      nullptr,
      &model_name_truncated);
  if (model_name_truncated) {
    out_report.warnings.push_back("Dropped an incomplete trailing CP932 character from VMD model "
                                  "name at offset " +
                                  std::to_string(model_name_offset));
  }
  model.header.compatible = true;

  const uint32_t bone_count = read_count(
      reader, "bone frame", kMaxBoneFrames, VMD_BONE_RECORD_BYTES);
  out_report.bone_frame_count = bone_count;
  model.bone_keyframes.reserve(bone_count);
  for (uint32_t i = 0; i < bone_count; i++) {
    const size_t source_offset = reader.pos();
    const std::string raw_name = reader.bytes(VMD_BONE_NAME_BYTES, "bone name");
    VMDBoneKeyframe key;
    key.source_offset = source_offset;
    bool used_gbk = false;
    bool truncated_tail = false;
    key.bone_name = decode_vmd_string(raw_name, source_offset, &used_gbk, &truncated_tail);
    (void)used_gbk;
    /* GBK/CP936 decoding is silent — Chinese VMD files commonly use GBK and
     * the decoded name is correct; reporting every offset is noise. */
    if (key.bone_name.empty()) {
      out_report.warnings.push_back("Empty bone name at offset " + std::to_string(source_offset));
    }
    if (truncated_tail) {
      out_report.warnings.push_back("Dropped an incomplete trailing CP932 character from bone name "
                                    "at offset " +
                                    std::to_string(source_offset));
    }
    key.frame = reader.u32("bone frame number");
    for (float &value : key.translation) {
      value = reader.f32("bone translation");
    }
    for (float &value : key.rotation) {
      value = reader.f32("bone rotation");
    }
    float quaternion_length = 0.0f;
    for (const float value : key.rotation) {
      quaternion_length += value * value;
    }
    if (quaternion_length == 0.0f) {
      throw VMDReaderError("Zero-length bone quaternion at offset " +
                           std::to_string(source_offset));
    }
    for (int8_t &value : key.interpolation) {
      value = static_cast<int8_t>(reader.byte("bone interpolation"));
    }
    model.bone_keyframes.push_back(std::move(key));
  }

  model.morph_frame_count = read_count(
      reader, "morph frame", kMaxMorphFrames, VMD_MORPH_RECORD_BYTES);
  out_report.morph_frame_count = model.morph_frame_count;
  model.morph_keyframes.reserve(model.morph_frame_count);
  for (uint32_t i = 0; i < model.morph_frame_count; i++) {
    const size_t source_offset = reader.pos();
    VMDMorphKeyframe key;
    key.source_offset = source_offset;
    bool used_gbk = false;
    bool truncated_tail = false;
    key.morph_name = decode_vmd_string(
        reader.bytes(VMD_MORPH_NAME_BYTES, "morph name"), source_offset, &used_gbk, &truncated_tail);
    (void)used_gbk;
    /* GBK/CP936 decoding is silent (see bone name comment above). */
    if (key.morph_name.empty()) {
      out_report.warnings.push_back("Empty morph name at offset " +
                                    std::to_string(source_offset));
    }
    if (truncated_tail) {
      out_report.warnings.push_back("Dropped an incomplete trailing CP932 character from morph "
                                    "name at offset " +
                                    std::to_string(source_offset));
    }
    key.frame = reader.u32("morph frame number");
    key.weight = reader.f32("morph weight");
    model.morph_keyframes.push_back(std::move(key));
  }

  model.camera_frame_count = read_count(
      reader, "camera frame", kMaxCameraFrames, VMD_CAMERA_RECORD_BYTES);
  out_report.camera_frame_count = model.camera_frame_count;
  model.camera_keyframes.reserve(model.camera_frame_count);
  uint32_t camera_angle_out_of_range = 0;
  for (uint32_t i = 0; i < model.camera_frame_count; i++) {
    const size_t source_offset = reader.pos();
    VMDCameraKeyframe key;
    key.source_offset = source_offset;
    key.frame = reader.u32("camera frame number");
    key.distance = reader.f32("camera distance");
    for (float &value : key.position) {
      value = reader.f32("camera position");
    }
    for (float &value : key.rotation) {
      value = reader.f32("camera rotation");
    }
    for (uint8_t &value : key.interpolation) {
      value = reader.byte("camera interpolation");
    }
    key.view_angle = reader.u32("camera view angle");
    /* The on-disk byte is a "perspective disabled" flag: 0 means perspective is on. */
    key.perspective = reader.byte("camera perspective") == 0;
    if (key.view_angle == 0 || key.view_angle > 180) {
      camera_angle_out_of_range++;
    }
    model.camera_keyframes.push_back(std::move(key));
  }
  if (camera_angle_out_of_range != 0) {
    /* Reported once per file rather than per keyframe; the camera Action stage clamps them. */
    out_report.warnings.push_back(
        "VMD camera view angle is outside the 1-180 degree range on " +
        std::to_string(camera_angle_out_of_range) + " keyframe(s); clamped on import");
  }

  model.light_frame_count = read_count(
      reader, "light frame", kMaxLightFrames, VMD_LIGHT_RECORD_BYTES);
  skip_fixed_records(reader, "light", model.light_frame_count, VMD_LIGHT_RECORD_BYTES);
  if (model.light_frame_count != 0) {
    model.has_unsupported_sections = true;
    out_report.warnings.push_back("VMD light frames are present but not imported");
  }

  model.shadow_frame_count = read_count(
      reader, "self-shadow frame", kMaxShadowFrames, VMD_SHADOW_RECORD_BYTES);
  skip_fixed_records(reader, "self-shadow", model.shadow_frame_count, VMD_SHADOW_RECORD_BYTES);
  if (model.shadow_frame_count != 0) {
    model.has_unsupported_sections = true;
    out_report.warnings.push_back("VMD self-shadow frames are present but not imported");
  }

  const bool has_property_section = reader.remaining() != 0;
  const uint32_t property_count = has_property_section ?
                                      read_count(reader,
                                                 "property frame",
                                                 kMaxPropertyFrames,
                                                 VMD_PROPERTY_FIXED_RECORD_BYTES) :
                                      0;
  if (!has_property_section) {
    out_report.warnings.push_back(
        "VMD property/IK section is absent; treating it as zero property frames");
  }
  out_report.property_frame_count = property_count;
  model.property_keyframes.reserve(property_count);
  for (uint32_t i = 0; i < property_count; i++) {
    const size_t source_offset = reader.pos();
    VMDPropertyKeyframe key;
    key.source_offset = source_offset;
    key.frame = reader.u32("property frame number");
    key.visible = reader.byte("property visibility") != 0;
    const uint32_t ik_count = read_count(
        reader, "property IK state", kMaxIKStatesPerProperty, VMD_PROPERTY_IK_RECORD_BYTES);
    key.ik_states.reserve(ik_count);
    for (uint32_t j = 0; j < ik_count; j++) {
      const size_t name_offset = reader.pos();
      bool truncated_tail = false;
      key.ik_states.push_back(
          {decode_vmd_string(reader.bytes(VMD_PROPERTY_IK_NAME_BYTES, "property IK name"),
                             name_offset,
                             nullptr,
                             &truncated_tail),
           reader.byte("property IK state") != 0});
      if (truncated_tail) {
        out_report.warnings.push_back("Dropped an incomplete trailing CP932 character from property "
                                      "IK name at offset " +
                                      std::to_string(name_offset));
      }
    }
    model.property_keyframes.push_back(std::move(key));
  }

  model.parse_end_offset = reader.pos();
  out_report.parse_end_offset = model.parse_end_offset;
  if (model.parse_end_offset != model.file_size) {
    throw VMDReaderError("Unexpected trailing data at offset " +
                         std::to_string(model.parse_end_offset) + " (file size " +
                         std::to_string(model.file_size) + ")");
  }
  return model;
}

bool write_vmd(const std::string &filepath, const VMDModel &model, VMDWriteReport *report)
{
  VMDWriteReport local_report;
  VMDWriteReport &out = report ? *report : local_report;
  out = {};
  if (model.bone_keyframes.size() > std::numeric_limits<uint32_t>::max() ||
      model.morph_keyframes.size() > std::numeric_limits<uint32_t>::max() ||
      model.camera_keyframes.size() > std::numeric_limits<uint32_t>::max())
  {
    out.errors.push_back("VMD keyframe count exceeds uint32 range");
    return false;
  }
  for (const VMDBoneKeyframe &key : model.bone_keyframes) {
    for (const float value : key.translation) {
      if (!std::isfinite(value)) {
        out.errors.push_back("VMD bone translation contains a non-finite value");
        return false;
      }
    }
    float rotation_length_squared = 0.0f;
    for (const float value : key.rotation) {
      if (!std::isfinite(value)) {
        out.errors.push_back("VMD bone rotation contains a non-finite value");
        return false;
      }
      rotation_length_squared += value * value;
    }
    if (rotation_length_squared == 0.0f) {
      out.errors.push_back("VMD bone rotation contains a zero quaternion");
      return false;
    }
  }
  for (const VMDMorphKeyframe &key : model.morph_keyframes) {
    if (!std::isfinite(key.weight)) {
      out.errors.push_back("VMD morph weight contains a non-finite value");
      return false;
    }
  }
  for (const VMDCameraKeyframe &key : model.camera_keyframes) {
    if (!std::isfinite(key.distance)) {
      out.errors.push_back("VMD camera distance contains a non-finite value");
      return false;
    }
    for (const float value : key.position) {
      if (!std::isfinite(value)) {
        out.errors.push_back("VMD camera position contains a non-finite value");
        return false;
      }
    }
    for (const float value : key.rotation) {
      if (!std::isfinite(value)) {
        out.errors.push_back("VMD camera rotation contains a non-finite value");
        return false;
      }
    }
    /* MikuMikuDance rejects a zero or out-of-range field of view outright, so refuse to
     * write a file the reader would only be able to repair by clamping. */
    if (key.view_angle == 0 || key.view_angle > 180) {
      out.errors.push_back("VMD camera view angle must be within the 1-180 degree range");
      return false;
    }
  }
  FILE *file = BLI_fopen(filepath.c_str(), "wb");
  if (file == nullptr) {
    out.errors.push_back("Cannot open VMD file for writing: " + filepath);
    return false;
  }
  std::set<std::string> reported_truncated_names;
  auto write_name = [&](const std::string &name, const size_t size, const char *field) {
    std::string encoded;
    bool used_gbk = false, truncated = false;
    if (!encode_vmd_string(name, size, encoded, used_gbk, truncated)) {
      out.errors.push_back(std::string("Cannot encode ") + field + " as CP932 or GBK: " + name);
      return false;
    }
    const std::string warning_key = std::string(field) + '\n' + name;
    /* GBK is a supported compatibility encoding for Chinese MMD names, not a
     * lossy fallback. Only report truncation or an actual encoding failure. */
    if (truncated && reported_truncated_names.insert(warning_key).second) {
      out.warnings.push_back(std::string(field) + " was truncated to fit VMD: " + name);
    }
    return fwrite(encoded.data(), 1, size, file) == size;
  };

  char signature[30] = {};
  std::memcpy(signature, kVMDSignature, sizeof(kVMDSignature) - 1);
  bool ok = fwrite(signature, 1, sizeof(signature), file) == sizeof(signature) &&
            write_name(model.header.model_name, VMD_MODEL_NAME_BYTES, "model name");
  write_u32(file, uint32_t(model.bone_keyframes.size()));
  for (const VMDBoneKeyframe &key : model.bone_keyframes) {
    ok = ok && write_name(key.bone_name, VMD_BONE_NAME_BYTES, "bone name");
    write_u32(file, key.frame);
    for (const float value : key.translation) {
      write_f32(file, value);
    }
    for (const float value : key.rotation) {
      write_f32(file, value);
    }
    fwrite(key.interpolation.data(), 1, key.interpolation.size(), file);
  }
  write_u32(file, uint32_t(model.morph_keyframes.size()));
  for (const VMDMorphKeyframe &key : model.morph_keyframes) {
    ok = ok && write_name(key.morph_name, VMD_MORPH_NAME_BYTES, "morph name");
    write_u32(file, key.frame);
    write_f32(file, key.weight);
  }
  write_u32(file, uint32_t(model.camera_keyframes.size()));
  for (const VMDCameraKeyframe &key : model.camera_keyframes) {
    write_u32(file, key.frame);
    write_f32(file, key.distance);
    for (const float value : key.position) {
      write_f32(file, value);
    }
    for (const float value : key.rotation) {
      write_f32(file, value);
    }
    fwrite(key.interpolation.data(), 1, key.interpolation.size(), file);
    write_u32(file, key.view_angle);
    /* The on-disk byte is a "perspective disabled" flag: 0 means perspective is on. */
    fputc(key.perspective ? 0 : 1, file);
  }
  write_u32(file, 0); /* light */
  write_u32(file, 0); /* self-shadow */
  write_u32(file, 0); /* property / IK */
  const bool stream_ok = ferror(file) == 0;
  const bool close_ok = fclose(file) == 0;
  file = nullptr;
  ok = ok && stream_ok && close_ok;
  if (!ok || !out.errors.empty()) {
    if (ok && !out.errors.empty()) {
      ok = false;
    }
    if (!ok && out.errors.empty()) {
      out.errors.push_back("Failed while writing VMD file: " + filepath);
    }
    BLI_delete(filepath.c_str(), false, false);
    return false;
  }
  out.success = true;
  out.bone_frame_count = uint32_t(model.bone_keyframes.size());
  out.morph_frame_count = uint32_t(model.morph_keyframes.size());
  return true;
}

}  // namespace blender::io::vmd
