#include "pmx_reader.h"

#include "BLI_fileops.hh"

#include <cstring>
#include <algorithm>
#include <cstdio>
#include <vector>
#include <cwchar>
#include <sstream>
#include <iomanip>

namespace {
constexpr int kMaxVertices = 10000000;
constexpr int kMaxFaceIndices = 30000000;
constexpr int kMaxSectionItems = 1000000;
constexpr int kMaxNestedItems = 10000000;
}

PMXModel PMXReader::read(const std::string& filepath) {
    /* [世界的歌] BLI_fopen accepts Blender's UTF-8 filepath on every supported platform. */
    FILE *f = blender::BLI_fopen(filepath.c_str(), "rb");
    if (!f) throw PMXReaderError("Cannot open file: " + filepath);

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    std::vector<uint8_t> buf(file_size);
    if (fread(buf.data(), 1, file_size, f) != (size_t)file_size) {
        fclose(f);
        throw PMXReaderError("Cannot read file: " + filepath);
    }
    fclose(f);

    return read_from_memory(buf.data(), buf.size(), filepath);
}

PMXModel PMXReader::read_from_memory(const uint8_t* data,
                                     size_t size,
                                     const std::string& filepath) {
    PMXReader reader(data, size, filepath);
    return reader.read_internal();
}

PMXReader::PMXReader(const uint8_t* data, size_t size, const std::string& filepath)
    : r_(data, size), filepath_(filepath) {}

PMXModel PMXReader::read_internal() {
    model_.file_size = r_.size();
    read_section("header", &PMXReader::read_header);
    read_section("model info", &PMXReader::read_model_info);
    read_section("vertices", &PMXReader::read_vertices);
    read_section("faces", &PMXReader::read_faces);
    read_section("textures", &PMXReader::read_textures);
    read_section("materials", &PMXReader::read_materials);
    read_section("bones", &PMXReader::read_bones);
    read_section("morphs", &PMXReader::read_morphs);
    read_section("display frames", &PMXReader::read_display_frames);
    read_section("rigid bodies", &PMXReader::read_rigid_bodies);
    read_section("joints", &PMXReader::read_joints);
    read_section("reference validation", &PMXReader::validate_references);
    model_.parse_end_offset = r_.pos();
    if (r_.pos() != r_.size()) {
        throw PMXReaderError("Unexpected trailing data at offset " +
                             std::to_string(r_.pos()) + " (file size " +
                             std::to_string(r_.size()) + ", " +
                             std::to_string(r_.size() - r_.pos()) + " bytes)");
    }
    return std::move(model_);
}

void PMXReader::read_section(const char *section_name, void (PMXReader::*reader)())
{
    const size_t start_offset = r_.pos();
    try {
        (this->*reader)();
    }
    catch (const PMXReaderError &e) {
        std::ostringstream message;
        message << "PMX parse failed in " << section_name << " at offset " << r_.pos() << " (started "
                << start_offset << ", file size " << r_.size() << "): " << e.what();
        throw PMXReaderError(message.str());
    }
}

/* [世界的歌] Validate counts before reserve() or loop conversion. */
int PMXReader::read_count(const char *field, size_t minimum_bytes_per_item, int hard_limit)
{
    const size_t offset = r_.pos();
    const int count = r_.read_int();
    if (count < 0) {
        throw PMXReaderError(std::string("Invalid negative count for ") + field + " at offset " +
                             std::to_string(offset) + ": " + std::to_string(count));
    }
    if (count > hard_limit) {
        throw PMXReaderError(std::string("Count exceeds limit for ") + field + " at offset " +
                             std::to_string(offset) + ": " + std::to_string(count));
    }
    if (minimum_bytes_per_item != 0 &&
        static_cast<size_t>(count) > (r_.size() - r_.pos()) / minimum_bytes_per_item) {
        throw PMXReaderError(std::string("Count cannot fit remaining data for ") + field +
                             " at offset " + std::to_string(offset) + ": " +
                             std::to_string(count));
    }
    return count;
}

int PMXReader::read_index(uint8_t size, bool is_signed) {
    switch (size) {
    case 1: return is_signed ? (int)r_.read_signed_byte() : (int)r_.read_byte();
    case 2: return is_signed ? (int)r_.read_short() : (int)r_.read_unsigned_short();
    case 4: return is_signed ? r_.read_int() : (int)r_.read_unsigned_int();
    default: throw PMXReaderError("Invalid index size: " + std::to_string((int)size));
    }
}

int PMXReader::read_vertex_index() { return read_index(model_.header.vertex_idx_size, false); }
int PMXReader::read_bone_index() { return read_index(model_.header.bone_idx_size, true); }
int PMXReader::read_texture_index() { return read_index(model_.header.texture_idx_size, true); }
int PMXReader::read_material_index() { return read_index(model_.header.material_idx_size, true); }
int PMXReader::read_morph_index() { return read_index(model_.header.morph_idx_size, true); }
int PMXReader::read_rigid_index() { return read_index(model_.header.rigid_idx_size, true); }

void PMXReader::validate_index_value(const int value,
                                     const size_t count,
                                     const bool allow_minus_one,
                                     const char *field) const
{
    if (value == -1 && allow_minus_one) {
        return;
    }
    if (value < 0 || static_cast<size_t>(value) >= count) {
        throw PMXReaderError(std::string("Invalid ") + field + ": " + std::to_string(value) +
                             " (count " + std::to_string(count) + ")");
    }
}

void PMXReader::validate_references()
{
    for (size_t i = 0; i < model_.face_indices.size(); i++) {
        validate_index_value(model_.face_indices[i], model_.vertices.size(), false, "face vertex index");
    }
    for (const PMXVertex &vertex : model_.vertices) {
        for (const int index : vertex.bone_indices) {
            validate_index_value(index, model_.bones.size(), false, "vertex bone index");
        }
    }
    for (const PMXMaterial &material : model_.materials) {
        validate_index_value(material.texture_idx, model_.textures.size(), true, "material texture index");
        validate_index_value(material.sphere_texture_idx, model_.textures.size(), true, "sphere texture index");
        if (material.toon_flag == 1) {
            validate_index_value(material.toon_texture_idx, model_.textures.size(), true, "toon texture index");
        }
    }
    for (const PMXBone &bone : model_.bones) {
        validate_index_value(bone.parent_index, model_.bones.size(), true, "bone parent index");
        if (bone.tail_pos_bone != -2) {
            validate_index_value(bone.tail_pos_bone, model_.bones.size(), true, "bone tail index");
        }
        if (bone.inherit_parent_index != -1) {
            validate_index_value(bone.inherit_parent_index, model_.bones.size(), false, "inherit parent index");
        }
        if (bone.ik_target_index != -1) {
            validate_index_value(bone.ik_target_index, model_.bones.size(), false, "IK target index");
        }
        for (const PMXIKLink &link : bone.ik_links) {
            validate_index_value(link.bone_index, model_.bones.size(), false, "IK link bone index");
        }
    }
    for (const PMXMorph &morph : model_.morphs) {
        for (const PMXGroupMorphOffset &offset : morph.group_offsets) {
            validate_index_value(offset.morph_index, model_.morphs.size(), false, "group morph index");
        }
        for (const PMXVertexMorphOffset &offset : morph.vertex_offsets) {
            validate_index_value(offset.vertex_index, model_.vertices.size(), false, "vertex morph index");
        }
        for (const PMXUVMorphOffset &offset : morph.uv_offsets) {
            validate_index_value(offset.vertex_index, model_.vertices.size(), false, "UV morph vertex index");
        }
        for (const PMXBoneMorphOffset &offset : morph.bone_offsets) {
            validate_index_value(offset.bone_index, model_.bones.size(), false, "bone morph index");
        }
        for (const PMXMaterialMorphOffset &offset : morph.material_offsets) {
            validate_index_value(offset.material_index, model_.materials.size(), true, "material morph index");
        }
        for (const PMXImpulseMorphOffset &offset : morph.impulse_offsets) {
            validate_index_value(offset.rigid_index, model_.rigid_bodies.size(), false, "impulse rigid index");
        }
    }
    for (const PMXDisplayFrame &frame : model_.display_frames) {
        for (const PMXDisplayFrame::FrameItem &item : frame.items) {
            validate_index_value(item.index,
                                 item.type == 0 ? model_.bones.size() : model_.morphs.size(),
                                 false,
                                 item.type == 0 ? "display frame bone index" : "display frame morph index");
        }
    }
    for (const PMXRigidBody &rigid_body : model_.rigid_bodies) {
        validate_index_value(rigid_body.bone_index, model_.bones.size(), true, "rigid body bone index");
    }
    for (const PMXJoint &joint : model_.joints) {
        validate_index_value(joint.rigid_a_index, model_.rigid_bodies.size(), false, "joint rigid A index");
        validate_index_value(joint.rigid_b_index, model_.rigid_bodies.size(), false, "joint rigid B index");
    }
}

std::string PMXReader::read_string() {
    const size_t length_offset = r_.pos();
    const int len = r_.read_int();
    if (len < 0) {
        throw PMXReaderError("Negative string length at offset " +
                             std::to_string(length_offset) + ": " + std::to_string(len));
    }
    if (len == 0) {
        return "";
    }
    if (static_cast<size_t>(len) > r_.size() - r_.pos()) {
        throw PMXReaderError("String length exceeds remaining data at offset " +
                             std::to_string(length_offset) + ": " + std::to_string(len));
    }

    const size_t data_offset = r_.pos();
    if (model_.is_utf8()) {
        std::string result(reinterpret_cast<const char *>(&r_.data()[data_offset]),
                           static_cast<size_t>(len));
        r_.skip(static_cast<size_t>(len));
        return result;
    }

    if ((len & 1) != 0) {
        throw PMXReaderError("Odd UTF-16LE string length at offset " +
                             std::to_string(length_offset) + ": " + std::to_string(len));
    }

    const size_t unit_count = static_cast<size_t>(len) / 2;
    std::vector<uint16_t> units;
    units.reserve(unit_count);
    for (size_t i = 0; i < unit_count; i++) {
        const uint8_t *p = &r_.data()[data_offset + i * 2];
        units.push_back(static_cast<uint16_t>(p[0] | (p[1] << 8)));
    }

    std::string result;
    for (size_t i = 0; i < units.size(); i++) {
        uint32_t codepoint = units[i];
        if (codepoint >= 0xD800 && codepoint <= 0xDBFF) {
            if (i + 1 >= units.size() || units[i + 1] < 0xDC00 || units[i + 1] > 0xDFFF) {
                throw PMXReaderError("Unpaired UTF-16 high surrogate at offset " +
                                     std::to_string(data_offset + i * 2));
            }
            codepoint = 0x10000 + ((codepoint - 0xD800) << 10) + (units[++i] - 0xDC00);
        }
        else if (codepoint >= 0xDC00 && codepoint <= 0xDFFF) {
            throw PMXReaderError("Unpaired UTF-16 low surrogate at offset " +
                                 std::to_string(data_offset + i * 2));
        }

        if (codepoint < 0x80) {
            result.push_back(static_cast<char>(codepoint));
        }
        else if (codepoint < 0x800) {
            result.push_back(static_cast<char>(0xC0 | (codepoint >> 6)));
            result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
        }
        else if (codepoint < 0x10000) {
            result.push_back(static_cast<char>(0xE0 | (codepoint >> 12)));
            result.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
            result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
        }
        else {
            result.push_back(static_cast<char>(0xF0 | (codepoint >> 18)));
            result.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F)));
            result.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
            result.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
        }
    }
    r_.skip(static_cast<size_t>(len));
    return result;
}

// ===== Header =====

void PMXReader::read_header() {
    if (r_.read_byte() != 'P' || r_.read_byte() != 'M' ||
        r_.read_byte() != 'X' || r_.read_byte() != ' ') {
        throw PMXReaderError("Not a valid PMX file");
    }
    model_.header.version = r_.read_float();
    model_.header.header_size = r_.read_byte();
    model_.header.encoding = r_.read_byte();
    model_.header.add_uv_cnt = r_.read_byte();
    model_.header.vertex_idx_size = r_.read_byte();
    model_.header.texture_idx_size = r_.read_byte();
    model_.header.material_idx_size = r_.read_byte();
    model_.header.bone_idx_size = r_.read_byte();
    model_.header.morph_idx_size = r_.read_byte();
    model_.header.rigid_idx_size = r_.read_byte();

    if (model_.header.version != 2.0f && model_.header.version != 2.1f) {
        throw PMXReaderError("Unsupported PMX version: " + std::to_string(model_.header.version));
    }
    if (model_.header.header_size != 8) {
        throw PMXReaderError("Invalid PMX header size: " +
                             std::to_string(model_.header.header_size));
    }
    if (model_.header.encoding > 1) {
        throw PMXReaderError("Invalid PMX encoding: " +
                             std::to_string(model_.header.encoding));
    }
    if (model_.header.add_uv_cnt > 4) {
        throw PMXReaderError("Invalid additional UV count: " +
                             std::to_string(model_.header.add_uv_cnt));
    }
    const uint8_t index_sizes[] = {model_.header.vertex_idx_size,
                                   model_.header.texture_idx_size,
                                   model_.header.material_idx_size,
                                   model_.header.bone_idx_size,
                                   model_.header.morph_idx_size,
                                   model_.header.rigid_idx_size};
    for (const uint8_t index_size : index_sizes) {
        if (index_size != 1 && index_size != 2 && index_size != 4) {
            throw PMXReaderError("Invalid PMX index size: " + std::to_string(index_size));
        }
    }
}

void PMXReader::read_model_info() {
    model_.name_local = read_string();
    model_.name_universal = read_string();
    model_.comment_local = read_string();
    model_.comment_universal = read_string();
}

// ===== Vertices =====

void PMXReader::read_vertices() {
    const int count = read_count("vertices", 1, kMaxVertices);
    model_.vertices.reserve(static_cast<size_t>(count));
    int add_uv = model_.add_uv_count();

    for (int i = 0; i < count; i++) {
        PMXVertex v{};
        v.additional_uv_count = add_uv;
        r_.read_float3(v.pos);
        r_.read_float3(v.normal);
        v.uv[0] = r_.read_float();
        v.uv[1] = r_.read_float();
        for (int j = 0; j < add_uv; j++) {
            r_.read_float4(v.additional_uv[j].data());
        }
        const uint8_t weight_type = r_.read_byte();
        if (weight_type > static_cast<uint8_t>(BoneWeightType::QDEF)) {
            throw PMXReaderError("Invalid vertex weight type: " + std::to_string(weight_type));
        }
        v.weight_type = (BoneWeightType)weight_type;

        switch (v.weight_type) {
        case BoneWeightType::BDEF1:
            v.bone_indices.push_back(read_bone_index());
            v.bone_weights.push_back(1.0f);
            break;
        case BoneWeightType::BDEF2:
            v.bone_indices.push_back(read_bone_index());
            v.bone_indices.push_back(read_bone_index());
            { float w0 = r_.read_float();
              v.bone_weights.push_back(w0);
              v.bone_weights.push_back(1.0f - w0); }
            break;
        case BoneWeightType::BDEF4:
        case BoneWeightType::QDEF:
            for (int j = 0; j < 4; j++) v.bone_indices.push_back(read_bone_index());
            for (int j = 0; j < 4; j++) v.bone_weights.push_back(r_.read_float());
            break;
        case BoneWeightType::SDEF:
            v.bone_indices.push_back(read_bone_index());
            v.bone_indices.push_back(read_bone_index());
            { float w0 = r_.read_float();
              v.bone_weights.push_back(w0);
              v.bone_weights.push_back(1.0f - w0); }
            r_.read_float3(v.sdef_c);
            r_.read_float3(v.sdef_r0);
            r_.read_float3(v.sdef_r1);
            break;
        default:
            throw PMXReaderError("Unknown weight type");
        }
        v.edge_factor = r_.read_float();
        model_.vertices.push_back(std::move(v));
    }
}

void PMXReader::read_faces() {
    const int total = read_count("face indices", 1, kMaxFaceIndices);
    model_.face_indices.reserve(static_cast<size_t>(total));
    for (int i = 0; i < total; i++) {
        model_.face_indices.push_back(read_vertex_index());
    }
}

void PMXReader::read_textures() {
    const int count = read_count("textures", 1, kMaxSectionItems);
    model_.textures.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; i++) {
        PMXTexture t;
        t.path = read_string();
        model_.textures.push_back(std::move(t));
    }
}

void PMXReader::read_materials() {
    const int count = read_count("materials", 1, kMaxSectionItems);
    model_.materials.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; i++) {
        PMXMaterial mat;
        memset(&mat, 0, sizeof(mat));
        mat.texture_idx = mat.sphere_texture_idx = mat.toon_texture_idx = -1;

        mat.name_local = read_string();
        mat.name_universal = read_string();
        r_.read_float4(mat.diffuse);
        r_.read_float3(mat.specular);
        mat.specular_power = r_.read_float();
        r_.read_float3(mat.ambient);
        mat.flag = r_.read_byte();
        r_.read_float4(mat.edge_color);
        mat.edge_size = r_.read_float();
        mat.texture_idx = read_texture_index();
        mat.sphere_texture_idx = read_texture_index();
        const uint8_t sphere_mode = r_.read_byte();
        if (sphere_mode > static_cast<uint8_t>(SphereMode::Cube)) {
            throw PMXReaderError("Invalid sphere mode: " + std::to_string(sphere_mode));
        }
        mat.sphere_mode = (SphereMode)sphere_mode;
        mat.toon_flag = r_.read_byte();
        if (mat.toon_flag > 1) {
            throw PMXReaderError("Invalid toon flag: " + std::to_string(mat.toon_flag));
        }
        if (mat.toon_flag == 0) mat.toon_internal_value = r_.read_byte();
        else mat.toon_texture_idx = read_texture_index();
        mat.memo = read_string();
        mat.face_vertex_count = r_.read_int();
        model_.materials.push_back(std::move(mat));
    }
}

void PMXReader::read_bones() {
    const int count = read_count("bones", 1, kMaxSectionItems);
    model_.bones.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; i++) {
        PMXBone bone;
        memset(&bone, 0, sizeof(bone));
        bone.parent_index = bone.tail_pos_bone = -1;
        bone.inherit_parent_index = bone.ik_target_index = bone.external_parent_index = -1;

        bone.name_local = read_string();
        bone.name_universal = read_string();
        r_.read_float3(bone.pos);
        bone.parent_index = read_bone_index();
        bone.transform_order = r_.read_int();
        bone.flag = r_.read_unsigned_short();

        if (bone.flag & BONE_FLAG_TAIL_POS) {
            bone.tail_pos_bone = read_bone_index();
        } else {
            bone.tail_pos_bone = -2;
            r_.read_float3(bone.tail_pos_offset);
        }

        if (bone.flag & (BONE_FLAG_APPEND_ROTATION | BONE_FLAG_APPEND_TRANSLATE)) {
            bone.inherit_parent_index = read_bone_index();
            bone.inherit_parent_ratio = r_.read_float();
        }
        if (bone.flag & BONE_FLAG_FIXED_AXIS) r_.read_float3(bone.fixed_axis);
        if (bone.flag & BONE_FLAG_LOCAL_AXIS) { r_.read_float3(bone.local_x); r_.read_float3(bone.local_z); }
        if (bone.flag & BONE_FLAG_EXTERNAL_PARENT) bone.external_parent_index = r_.read_int();

        if (bone.flag & BONE_FLAG_IK) {
            bone.ik_target_index = read_bone_index();
            bone.ik_loop_count = r_.read_int();
            bone.ik_angle_limit = r_.read_float();
            const int link_count = read_count("IK links", 1, kMaxNestedItems);
            for (int j = 0; j < link_count; j++) {
                PMXIKLink link;
                memset(&link, 0, sizeof(link));
                link.bone_index = read_bone_index();
                const uint8_t limit_angle = r_.read_byte();
                if (limit_angle > 1) {
                    throw PMXReaderError("Invalid IK limit flag: " +
                                         std::to_string(limit_angle));
                }
                link.limit_angle = (limit_angle != 0);
                if (link.limit_angle) { r_.read_float3(link.limit_min); r_.read_float3(link.limit_max); }
                bone.ik_links.push_back(link);
            }
        }
        model_.bones.push_back(std::move(bone));
    }
}

void PMXReader::read_morphs() {
    const int count = read_count("morphs", 1, kMaxSectionItems);
    model_.morphs.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; i++) {
        PMXMorph m;
        memset(&m, 0, sizeof(m));
        m.name_local = read_string();
        m.name_universal = read_string();
        m.panel = r_.read_byte();
        const uint8_t morph_type = r_.read_byte();
        if (morph_type > static_cast<uint8_t>(MorphType::Impulse) || morph_type == 7) {
            throw PMXReaderError("Unsupported morph type: " + std::to_string(morph_type));
        }
        m.type = (MorphType)morph_type;
        const int off_count = read_count("morph offsets", 1, kMaxNestedItems);
        
        switch (m.type) {
        case MorphType::Group:
        case MorphType::Flip:
            for (int j = 0; j < off_count; j++) {
                PMXGroupMorphOffset o;
                o.morph_index = read_morph_index();
                o.influence = r_.read_float();
                m.group_offsets.push_back(o);
            }
            break;
        case MorphType::Vertex:
            for (int j = 0; j < off_count; j++) {
                PMXVertexMorphOffset o;
                o.vertex_index = read_vertex_index();
                r_.read_float3(o.offset);
                m.vertex_offsets.push_back(o);
            }
            break;
        case MorphType::Bone:
            for (int j = 0; j < off_count; j++) {
                PMXBoneMorphOffset o;
                o.bone_index = read_bone_index();
                r_.read_float3(o.pos);
                o.rot[0] = r_.read_float(); o.rot[1] = r_.read_float();
                o.rot[2] = r_.read_float(); o.rot[3] = r_.read_float();
                m.bone_offsets.push_back(o);
            }
            break;
        case MorphType::UV: case MorphType::UV_2nd:
        case MorphType::UV_3rd: case MorphType::UV_4th:
            for (int j = 0; j < off_count; j++) {
                PMXUVMorphOffset o;
                o.vertex_index = read_vertex_index();
                for (int k = 0; k < 4; k++) o.offset[k] = r_.read_float();
                m.uv_offsets.push_back(o);
            }
            break;
        case MorphType::Material:
            for (int j = 0; j < off_count; j++) {
                PMXMaterialMorphOffset o;
                memset(&o, 0, sizeof(o));
                o.material_index = read_material_index();
                o.calc_mode = r_.read_byte();
                if (o.calc_mode > 1) {
                    throw PMXReaderError("Invalid material morph calculation mode: " +
                                         std::to_string(o.calc_mode));
                }
                r_.read_float4(o.diffuse); r_.read_float3(o.specular);
                o.specular_power = r_.read_float(); r_.read_float3(o.ambient);
                r_.read_float4(o.edge_color); o.edge_size = r_.read_float();
                r_.read_float4(o.texture_factor); r_.read_float4(o.sphere_texture_factor);
                r_.read_float4(o.toon_texture_factor);
                m.material_offsets.push_back(o);
            }
            break;
        case MorphType::Impulse:
            for (int j = 0; j < off_count; j++) {
                PMXImpulseMorphOffset o;
                memset(&o, 0, sizeof(o));
                o.rigid_index = read_rigid_index();
                o.local_flag = r_.read_byte();
                if (o.local_flag > 1) {
                    throw PMXReaderError("Invalid impulse local flag: " +
                                         std::to_string(o.local_flag));
                }
                r_.read_float3(o.velocity); r_.read_float3(o.torque);
                m.impulse_offsets.push_back(o);
            }
            break;
        default:
            throw PMXReaderError("Unsupported morph type: " +
                                 std::to_string(static_cast<uint8_t>(m.type)));
        }
        model_.morphs.push_back(std::move(m));
    }
}

void PMXReader::read_display_frames() {
    const int count = read_count("display frames", 1, kMaxSectionItems);
    model_.display_frames.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; i++) {
        PMXDisplayFrame frame;
        frame.name_local = read_string();
        frame.name_universal = read_string();
        frame.flag = r_.read_byte();
        if (frame.flag > 1) {
            throw PMXReaderError("Invalid display frame flag: " +
                                 std::to_string(frame.flag));
        }
        const int item_count = read_count("display frame items", 1, kMaxNestedItems);
        frame.items.reserve(static_cast<size_t>(item_count));
        for (int j = 0; j < item_count; j++) {
            PMXDisplayFrame::FrameItem item;
            item.type = r_.read_byte();
            if (item.type > 1) {
                throw PMXReaderError("Invalid display frame item type: " +
                                     std::to_string(item.type));
            }
            item.index = (item.type == 0) ? read_bone_index() : read_morph_index();
            frame.items.push_back(item);
        }
        model_.display_frames.push_back(std::move(frame));
    }
}

void PMXReader::read_rigid_bodies() {
    const int count = read_count("rigid bodies", 1, kMaxSectionItems);
    model_.rigid_bodies.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; i++) {
        PMXRigidBody rigid_body{};
        rigid_body.bone_index = -1;
        rigid_body.name_local = read_string();
        rigid_body.name_universal = read_string();
        rigid_body.bone_index = read_bone_index();
        rigid_body.collision_group = r_.read_byte();
        rigid_body.no_collision_group = r_.read_unsigned_short();
        rigid_body.shape_type = r_.read_byte();
        if (rigid_body.shape_type > 2) {
            throw PMXReaderError("Invalid rigid body shape type: " +
                                 std::to_string(rigid_body.shape_type));
        }
        r_.read_float3(rigid_body.shape_size);
        r_.read_float3(rigid_body.pos);
        r_.read_float3(rigid_body.rot);
        rigid_body.mass = r_.read_float();
        rigid_body.linear_damping = r_.read_float();
        rigid_body.angular_damping = r_.read_float();
        rigid_body.restitution = r_.read_float();
        rigid_body.friction = r_.read_float();
        rigid_body.physics_type = r_.read_byte();
        if (rigid_body.physics_type > 2) {
            throw PMXReaderError("Invalid rigid body physics type: " +
                                 std::to_string(rigid_body.physics_type));
        }
        model_.rigid_bodies.push_back(std::move(rigid_body));
    }
}

void PMXReader::read_joints() {
    const int count = read_count("joints", 1, kMaxSectionItems);
    model_.joints.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; i++) {
        PMXJoint joint{};
        joint.name_local = read_string();
        joint.name_universal = read_string();
        joint.type = r_.read_byte();
        if (joint.type != 0) {
            throw PMXReaderError("Unsupported joint type: " + std::to_string(joint.type));
        }
        joint.rigid_a_index = read_rigid_index();
        joint.rigid_b_index = read_rigid_index();
        r_.read_float3(joint.pos);
        r_.read_float3(joint.rot);
        r_.read_float3(joint.translation_limit_min);
        r_.read_float3(joint.translation_limit_max);
        r_.read_float3(joint.rotation_limit_min);
        r_.read_float3(joint.rotation_limit_max);
        r_.read_float3(joint.spring_translation);
        r_.read_float3(joint.spring_rotation);
        model_.joints.push_back(std::move(joint));
    }
}
