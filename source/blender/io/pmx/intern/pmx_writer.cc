#include "pmx_writer.h"

#include "BLI_fileops.hh"

#include <cstdio>
#include <limits>
#include <sstream>

namespace {

/* [世界的歌] Decode one UTF-8 sequence. Mirrors the UTF-16 -> UTF-8 conversion in
 * PMXReader::read_string so a string survives read -> write -> read unchanged. */
uint32_t decode_utf8(const std::string &value, size_t &pos, const char *field)
{
    const auto fail = [&](const char *why) {
        throw PMXWriterError(std::string("Invalid UTF-8 in ") + field + " at byte " +
                             std::to_string(pos) + ": " + why);
    };

    const size_t remaining = value.size() - pos;
    const uint8_t b0 = static_cast<uint8_t>(value[pos]);
    size_t length;
    uint32_t codepoint;

    if (b0 < 0x80) {
        length = 1;
        codepoint = b0;
    }
    else if ((b0 & 0xE0) == 0xC0) {
        length = 2;
        codepoint = uint32_t(b0 & 0x1F);
    }
    else if ((b0 & 0xF0) == 0xE0) {
        length = 3;
        codepoint = uint32_t(b0 & 0x0F);
    }
    else if ((b0 & 0xF8) == 0xF0) {
        length = 4;
        codepoint = uint32_t(b0 & 0x07);
    }
    else {
        fail("unexpected leading byte");
        return 0;
    }

    if (remaining < length) {
        fail("truncated sequence");
    }
    for (size_t i = 1; i < length; i++) {
        const uint8_t b = static_cast<uint8_t>(value[pos + i]);
        if ((b & 0xC0) != 0x80) {
            fail("missing continuation byte");
        }
        codepoint = (codepoint << 6) | uint32_t(b & 0x3F);
    }

    /* Reject overlong encodings, UTF-16 surrogates and out-of-range codepoints so
     * the emitted UTF-16LE is always well formed. */
    static const uint32_t minimum[5] = {0, 0, 0x80, 0x800, 0x10000};
    if (codepoint < minimum[length]) {
        fail("overlong encoding");
    }
    if (codepoint >= 0xD800 && codepoint <= 0xDFFF) {
        fail("UTF-16 surrogate is not a valid codepoint");
    }
    if (codepoint > 0x10FFFF) {
        fail("codepoint above U+10FFFF");
    }

    pos += length;
    return codepoint;
}

/** Convert a UTF-8 string to UTF-16LE code units. */
std::vector<uint16_t> utf8_to_utf16le(const std::string &value, const char *field)
{
    std::vector<uint16_t> units;
    units.reserve(value.size());
    size_t pos = 0;
    while (pos < value.size()) {
        const uint32_t codepoint = decode_utf8(value, pos, field);
        if (codepoint < 0x10000) {
            units.push_back(static_cast<uint16_t>(codepoint));
        }
        else {
            const uint32_t offset = codepoint - 0x10000;
            units.push_back(static_cast<uint16_t>(0xD800 + (offset >> 10)));
            units.push_back(static_cast<uint16_t>(0xDC00 + (offset & 0x3FF)));
        }
    }
    return units;
}

bool is_valid_index_size(const uint8_t size)
{
    return size == 1 || size == 2 || size == 4;
}

}  // namespace

void PMXWriter::write(const PMXModel &model,
                      const std::string &filepath,
                      const PMXWriteOptions &options)
{
    const std::vector<uint8_t> buffer = write_to_memory(model, options);

    /* [世界的歌] BLI_fopen accepts Blender's UTF-8 filepath on every supported platform. */
    FILE *f = blender::BLI_fopen(filepath.c_str(), "wb");
    if (!f) {
        throw PMXWriterError("Cannot open file for writing: " + filepath);
    }
    if (!buffer.empty() && fwrite(buffer.data(), 1, buffer.size(), f) != buffer.size()) {
        fclose(f);
        throw PMXWriterError("Cannot write file: " + filepath);
    }
    if (fclose(f) != 0) {
        throw PMXWriterError("Cannot flush file: " + filepath);
    }
}

std::vector<uint8_t> PMXWriter::write_to_memory(const PMXModel &model,
                                                const PMXWriteOptions &options)
{
    PMXWriter writer(model, options);
    writer.write_internal();
    return writer.w_.take_buffer();
}

/* Mirrors mmd_tools Header.__getIndexSize (core/pmx/__init__.py) so a model
 * exported here uses the same index widths MMD tooling expects. It is one step
 * conservative for signed kinds, which is intentional. */
uint8_t PMXWriter::minimum_index_size(const size_t count, const bool is_signed)
{
    const size_t divisor = is_signed ? 2 : 1;
    if (size_t(1u << 8) / divisor > count) {
        return 1;
    }
    if (size_t(1u << 16) / divisor > count) {
        return 2;
    }
    return 4;
}

PMXWriter::PMXWriter(const PMXModel &model, const PMXWriteOptions &options)
    : model_(model), options_(options) {}

void PMXWriter::write_internal()
{
    header_ = resolve_header();

    const auto section = [&](const char *name, void (PMXWriter::*writer)()) {
        const size_t start_offset = w_.size();
        try {
            (this->*writer)();
        }
        catch (const PMXWriterError &e) {
            std::ostringstream message;
            message << "PMX write failed in " << name << " at offset " << w_.size() << " (started "
                    << start_offset << "): " << e.what();
            throw PMXWriterError(message.str());
        }
    };

    section("header", &PMXWriter::write_header);
    section("model info", &PMXWriter::write_model_info);
    section("vertices", &PMXWriter::write_vertices);
    section("faces", &PMXWriter::write_faces);
    section("textures", &PMXWriter::write_textures);
    section("materials", &PMXWriter::write_materials);
    section("bones", &PMXWriter::write_bones);
    section("morphs", &PMXWriter::write_morphs);
    section("display frames", &PMXWriter::write_display_frames);
    section("rigid bodies", &PMXWriter::write_rigid_bodies);
    section("joints", &PMXWriter::write_joints);
}

PMXHeader PMXWriter::resolve_header() const
{
    PMXHeader header = model_.header;

    /* The signature and header size are format constants, never model data. */
    header.signature[0] = 'P';
    header.signature[1] = 'M';
    header.signature[2] = 'X';
    header.signature[3] = ' ';
    header.header_size = 8;

    if (options_.recompute_index_sizes && header.version != 2.0f && header.version != 2.1f) {
        /* A default-constructed header carries version 0; assume base PMX 2.0. */
        header.version = 2.0f;
    }
    if (header.version != 2.0f && header.version != 2.1f) {
        throw PMXWriterError("Unsupported PMX version: " + std::to_string(header.version));
    }
    if (header.encoding > 1) {
        throw PMXWriterError("Invalid PMX encoding: " + std::to_string(header.encoding));
    }
    if (header.add_uv_cnt > 4) {
        throw PMXWriterError("Invalid additional UV count: " + std::to_string(header.add_uv_cnt));
    }

    if (options_.recompute_index_sizes) {
        header.vertex_idx_size = minimum_index_size(model_.vertices.size(), false);
        header.texture_idx_size = minimum_index_size(model_.textures.size(), true);
        header.material_idx_size = minimum_index_size(model_.materials.size(), true);
        header.bone_idx_size = minimum_index_size(model_.bones.size(), true);
        header.morph_idx_size = minimum_index_size(model_.morphs.size(), true);
        header.rigid_idx_size = minimum_index_size(model_.rigid_bodies.size(), true);
    }

    /* Index values are range-checked individually while writing, so only the
     * encoding widths themselves need validating here. */
    const uint8_t index_sizes[] = {header.vertex_idx_size,
                                   header.texture_idx_size,
                                   header.material_idx_size,
                                   header.bone_idx_size,
                                   header.morph_idx_size,
                                   header.rigid_idx_size};
    for (const uint8_t index_size : index_sizes) {
        if (!is_valid_index_size(index_size)) {
            throw PMXWriterError("Invalid PMX index size: " + std::to_string(index_size));
        }
    }

    return header;
}

void PMXWriter::write_count(const size_t count, const char *field)
{
    if (count > size_t(std::numeric_limits<int32_t>::max())) {
        throw PMXWriterError(std::string("Count exceeds PMX limit for ") + field + ": " +
                             std::to_string(count));
    }
    w_.write_int(static_cast<int32_t>(count));
}

void PMXWriter::write_index(const int value,
                            const uint8_t size,
                            const bool is_signed,
                            const char *field)
{
    const auto out_of_range = [&]() {
        throw PMXWriterError(std::string("Index out of range for ") + field + ": " +
                             std::to_string(value) + " does not fit " + std::to_string(size) +
                             (is_signed ? " signed byte(s)" : " unsigned byte(s)"));
    };

    if (is_signed) {
        switch (size) {
        case 1:
            if (value < -128 || value > 127) {
                out_of_range();
            }
            w_.write_signed_byte(static_cast<int8_t>(value));
            return;
        case 2:
            if (value < -32768 || value > 32767) {
                out_of_range();
            }
            w_.write_short(static_cast<int16_t>(value));
            return;
        case 4:
            w_.write_int(value);
            return;
        default:
            throw PMXWriterError("Invalid index size: " + std::to_string(int(size)));
        }
    }

    /* Unsigned kinds (vertex indices) have no "none" encoding. */
    if (value < 0) {
        out_of_range();
    }
    switch (size) {
    case 1:
        if (value > 0xFF) {
            out_of_range();
        }
        w_.write_byte(static_cast<uint8_t>(value));
        return;
    case 2:
        if (value > 0xFFFF) {
            out_of_range();
        }
        w_.write_unsigned_short(static_cast<uint16_t>(value));
        return;
    case 4:
        w_.write_unsigned_int(static_cast<uint32_t>(value));
        return;
    default:
        throw PMXWriterError("Invalid index size: " + std::to_string(int(size)));
    }
}

void PMXWriter::write_vertex_index(const int value, const char *field)
{
    write_index(value, header_.vertex_idx_size, false, field);
}
void PMXWriter::write_bone_index(const int value, const char *field)
{
    write_index(value, header_.bone_idx_size, true, field);
}
void PMXWriter::write_texture_index(const int value, const char *field)
{
    write_index(value, header_.texture_idx_size, true, field);
}
void PMXWriter::write_material_index(const int value, const char *field)
{
    write_index(value, header_.material_idx_size, true, field);
}
void PMXWriter::write_morph_index(const int value, const char *field)
{
    write_index(value, header_.morph_idx_size, true, field);
}
void PMXWriter::write_rigid_index(const int value, const char *field)
{
    write_index(value, header_.rigid_idx_size, true, field);
}

void PMXWriter::write_string(const std::string &value, const char *field)
{
    if (header_.encoding == uint8_t(PMXEncoding::UTF8)) {
        write_count(value.size(), field);
        w_.write_bytes(value.data(), value.size());
        return;
    }

    const std::vector<uint16_t> units = utf8_to_utf16le(value, field);
    write_count(units.size() * 2, field);
    for (const uint16_t unit : units) {
        w_.write_unsigned_short(unit);
    }
}

// ===== Header =====

void PMXWriter::write_header()
{
    w_.write_bytes(header_.signature, 4);
    w_.write_float(header_.version, "PMX version");
    w_.write_byte(header_.header_size);
    w_.write_byte(header_.encoding);
    w_.write_byte(header_.add_uv_cnt);
    w_.write_byte(header_.vertex_idx_size);
    w_.write_byte(header_.texture_idx_size);
    w_.write_byte(header_.material_idx_size);
    w_.write_byte(header_.bone_idx_size);
    w_.write_byte(header_.morph_idx_size);
    w_.write_byte(header_.rigid_idx_size);
}

void PMXWriter::write_model_info()
{
    write_string(model_.name_local, "model name");
    write_string(model_.name_universal, "model universal name");
    write_string(model_.comment_local, "model comment");
    write_string(model_.comment_universal, "model universal comment");
}

// ===== Vertices =====

void PMXWriter::write_vertices()
{
    write_count(model_.vertices.size(), "vertices");
    const int add_uv = header_.add_uv_cnt;

    for (size_t i = 0; i < model_.vertices.size(); i++) {
        const PMXVertex &v = model_.vertices[i];
        const auto require = [&](const size_t indices, const size_t weights, const char *type) {
            if (v.bone_indices.size() < indices || v.bone_weights.size() < weights) {
                throw PMXWriterError(std::string("Vertex ") + std::to_string(i) + " (" + type +
                                     ") needs " + std::to_string(indices) + " bone indices and " +
                                     std::to_string(weights) + " weights, got " +
                                     std::to_string(v.bone_indices.size()) + " and " +
                                     std::to_string(v.bone_weights.size()));
            }
        };

        w_.write_float3(v.pos, "vertex position");
        w_.write_float3(v.normal, "vertex normal");
        w_.write_float(v.uv[0], "vertex UV");
        w_.write_float(v.uv[1], "vertex UV");
        for (int j = 0; j < add_uv; j++) {
            w_.write_float4(v.additional_uv[j].data(), "vertex additional UV");
        }

        const uint8_t weight_type = static_cast<uint8_t>(v.weight_type);
        if (weight_type > uint8_t(BoneWeightType::QDEF)) {
            throw PMXWriterError("Invalid vertex weight type: " + std::to_string(weight_type));
        }
        w_.write_byte(weight_type);

        switch (v.weight_type) {
        case BoneWeightType::BDEF1:
            require(1, 0, "BDEF1");
            write_bone_index(v.bone_indices[0], "vertex bone index");
            break;
        case BoneWeightType::BDEF2:
            require(2, 1, "BDEF2");
            write_bone_index(v.bone_indices[0], "vertex bone index");
            write_bone_index(v.bone_indices[1], "vertex bone index");
            w_.write_float(v.bone_weights[0], "vertex bone weight");
            break;
        case BoneWeightType::BDEF4:
        case BoneWeightType::QDEF:
            require(4, 4, v.weight_type == BoneWeightType::QDEF ? "QDEF" : "BDEF4");
            for (int j = 0; j < 4; j++) {
                write_bone_index(v.bone_indices[j], "vertex bone index");
            }
            for (int j = 0; j < 4; j++) {
                w_.write_float(v.bone_weights[j], "vertex bone weight");
            }
            break;
        case BoneWeightType::SDEF:
            require(2, 1, "SDEF");
            write_bone_index(v.bone_indices[0], "vertex bone index");
            write_bone_index(v.bone_indices[1], "vertex bone index");
            w_.write_float(v.bone_weights[0], "vertex bone weight");
            w_.write_float3(v.sdef_c, "SDEF C");
            w_.write_float3(v.sdef_r0, "SDEF R0");
            w_.write_float3(v.sdef_r1, "SDEF R1");
            break;
        default:
            throw PMXWriterError("Unknown weight type");
        }
        w_.write_float(v.edge_factor, "vertex edge factor");
    }
}

void PMXWriter::write_faces()
{
    write_count(model_.face_indices.size(), "face indices");
    for (const int index : model_.face_indices) {
        write_vertex_index(index, "face vertex index");
    }
}

void PMXWriter::write_textures()
{
    write_count(model_.textures.size(), "textures");
    for (const PMXTexture &texture : model_.textures) {
        write_string(texture.path, "texture path");
    }
}

void PMXWriter::write_materials()
{
    write_count(model_.materials.size(), "materials");
    for (const PMXMaterial &mat : model_.materials) {
        write_string(mat.name_local, "material name");
        write_string(mat.name_universal, "material universal name");
        w_.write_float4(mat.diffuse, "material diffuse");
        w_.write_float3(mat.specular, "material specular");
        w_.write_float(mat.specular_power, "material specular power");
        w_.write_float3(mat.ambient, "material ambient");
        w_.write_byte(mat.flag);
        w_.write_float4(mat.edge_color, "material edge color");
        w_.write_float(mat.edge_size, "material edge size");
        write_texture_index(mat.texture_idx, "material texture index");
        write_texture_index(mat.sphere_texture_idx, "material sphere texture index");

        const uint8_t sphere_mode = static_cast<uint8_t>(mat.sphere_mode);
        if (sphere_mode > uint8_t(SphereMode::Cube)) {
            throw PMXWriterError("Invalid sphere mode: " + std::to_string(sphere_mode));
        }
        w_.write_byte(sphere_mode);

        if (mat.toon_flag > 1) {
            throw PMXWriterError("Invalid toon flag: " + std::to_string(mat.toon_flag));
        }
        w_.write_byte(mat.toon_flag);
        if (mat.toon_flag == 0) {
            w_.write_byte(mat.toon_internal_value);
        }
        else {
            write_texture_index(mat.toon_texture_idx, "material toon texture index");
        }

        write_string(mat.memo, "material memo");
        w_.write_int(mat.face_vertex_count);
    }
}

void PMXWriter::write_bones()
{
    write_count(model_.bones.size(), "bones");
    for (const PMXBone &bone : model_.bones) {
        write_string(bone.name_local, "bone name");
        write_string(bone.name_universal, "bone universal name");
        w_.write_float3(bone.pos, "bone position");
        write_bone_index(bone.parent_index, "bone parent index");
        w_.write_int(bone.transform_order);
        w_.write_unsigned_short(bone.flag);

        if (bone.flag & BONE_FLAG_TAIL_POS) {
            write_bone_index(bone.tail_pos_bone, "bone tail index");
        }
        else {
            w_.write_float3(bone.tail_pos_offset, "bone tail offset");
        }

        if (bone.flag & (BONE_FLAG_APPEND_ROTATION | BONE_FLAG_APPEND_TRANSLATE)) {
            write_bone_index(bone.inherit_parent_index, "bone inherit parent index");
            w_.write_float(bone.inherit_parent_ratio, "bone inherit ratio");
        }
        if (bone.flag & BONE_FLAG_FIXED_AXIS) {
            w_.write_float3(bone.fixed_axis, "bone fixed axis");
        }
        if (bone.flag & BONE_FLAG_LOCAL_AXIS) {
            w_.write_float3(bone.local_x, "bone local X axis");
            w_.write_float3(bone.local_z, "bone local Z axis");
        }
        if (bone.flag & BONE_FLAG_EXTERNAL_PARENT) {
            w_.write_int(bone.external_parent_index);
        }

        if (bone.flag & BONE_FLAG_IK) {
            write_bone_index(bone.ik_target_index, "IK target index");
            w_.write_int(bone.ik_loop_count);
            w_.write_float(bone.ik_angle_limit, "IK angle limit");
            write_count(bone.ik_links.size(), "IK links");
            for (const PMXIKLink &link : bone.ik_links) {
                write_bone_index(link.bone_index, "IK link bone index");
                w_.write_byte(link.limit_angle ? 1 : 0);
                if (link.limit_angle) {
                    w_.write_float3(link.limit_min, "IK link limit min");
                    w_.write_float3(link.limit_max, "IK link limit max");
                }
            }
        }
    }
}

void PMXWriter::write_morphs()
{
    write_count(model_.morphs.size(), "morphs");
    for (const PMXMorph &m : model_.morphs) {
        write_string(m.name_local, "morph name");
        write_string(m.name_universal, "morph universal name");
        w_.write_byte(m.panel);

        const uint8_t morph_type = static_cast<uint8_t>(m.type);
        if (morph_type > uint8_t(MorphType::Impulse) || morph_type == 7) {
            throw PMXWriterError("Unsupported morph type: " + std::to_string(morph_type));
        }
        w_.write_byte(morph_type);

        switch (m.type) {
        case MorphType::Group:
        case MorphType::Flip:
            write_count(m.group_offsets.size(), "group morph offsets");
            for (const PMXGroupMorphOffset &o : m.group_offsets) {
                write_morph_index(o.morph_index, "group morph index");
                w_.write_float(o.influence, "group morph influence");
            }
            break;
        case MorphType::Vertex:
            write_count(m.vertex_offsets.size(), "vertex morph offsets");
            for (const PMXVertexMorphOffset &o : m.vertex_offsets) {
                write_vertex_index(o.vertex_index, "vertex morph vertex index");
                w_.write_float3(o.offset, "vertex morph offset");
            }
            break;
        case MorphType::Bone:
            write_count(m.bone_offsets.size(), "bone morph offsets");
            for (const PMXBoneMorphOffset &o : m.bone_offsets) {
                write_bone_index(o.bone_index, "bone morph bone index");
                w_.write_float3(o.pos, "bone morph position");
                w_.write_float4(o.rot, "bone morph rotation");
            }
            break;
        case MorphType::UV:
        case MorphType::UV_2nd:
        case MorphType::UV_3rd:
        case MorphType::UV_4th:
            write_count(m.uv_offsets.size(), "UV morph offsets");
            for (const PMXUVMorphOffset &o : m.uv_offsets) {
                write_vertex_index(o.vertex_index, "UV morph vertex index");
                w_.write_float4(o.offset, "UV morph offset");
            }
            break;
        case MorphType::Material:
            write_count(m.material_offsets.size(), "material morph offsets");
            for (const PMXMaterialMorphOffset &o : m.material_offsets) {
                write_material_index(o.material_index, "material morph material index");
                if (o.calc_mode > 1) {
                    throw PMXWriterError("Invalid material morph calculation mode: " +
                                         std::to_string(o.calc_mode));
                }
                w_.write_byte(o.calc_mode);
                w_.write_float4(o.diffuse, "material morph diffuse");
                w_.write_float3(o.specular, "material morph specular");
                w_.write_float(o.specular_power, "material morph specular power");
                w_.write_float3(o.ambient, "material morph ambient");
                w_.write_float4(o.edge_color, "material morph edge color");
                w_.write_float(o.edge_size, "material morph edge size");
                w_.write_float4(o.texture_factor, "material morph texture factor");
                w_.write_float4(o.sphere_texture_factor, "material morph sphere factor");
                w_.write_float4(o.toon_texture_factor, "material morph toon factor");
            }
            break;
        case MorphType::Impulse:
            write_count(m.impulse_offsets.size(), "impulse morph offsets");
            for (const PMXImpulseMorphOffset &o : m.impulse_offsets) {
                write_rigid_index(o.rigid_index, "impulse morph rigid index");
                if (o.local_flag > 1) {
                    throw PMXWriterError("Invalid impulse local flag: " +
                                         std::to_string(o.local_flag));
                }
                w_.write_byte(o.local_flag);
                w_.write_float3(o.velocity, "impulse morph velocity");
                w_.write_float3(o.torque, "impulse morph torque");
            }
            break;
        default:
            throw PMXWriterError("Unsupported morph type: " + std::to_string(morph_type));
        }
    }
}

void PMXWriter::write_display_frames()
{
    write_count(model_.display_frames.size(), "display frames");
    for (const PMXDisplayFrame &frame : model_.display_frames) {
        write_string(frame.name_local, "display frame name");
        write_string(frame.name_universal, "display frame universal name");
        if (frame.flag > 1) {
            throw PMXWriterError("Invalid display frame flag: " + std::to_string(frame.flag));
        }
        w_.write_byte(frame.flag);

        write_count(frame.items.size(), "display frame items");
        for (const PMXDisplayFrame::FrameItem &item : frame.items) {
            if (item.type > 1) {
                throw PMXWriterError("Invalid display frame item type: " +
                                     std::to_string(item.type));
            }
            w_.write_byte(item.type);
            if (item.type == 0) {
                write_bone_index(item.index, "display frame bone index");
            }
            else {
                write_morph_index(item.index, "display frame morph index");
            }
        }
    }
}

void PMXWriter::write_rigid_bodies()
{
    write_count(model_.rigid_bodies.size(), "rigid bodies");
    for (const PMXRigidBody &rigid_body : model_.rigid_bodies) {
        write_string(rigid_body.name_local, "rigid body name");
        write_string(rigid_body.name_universal, "rigid body universal name");
        write_bone_index(rigid_body.bone_index, "rigid body bone index");
        w_.write_byte(rigid_body.collision_group);
        w_.write_unsigned_short(rigid_body.no_collision_group);

        if (rigid_body.shape_type > 2) {
            throw PMXWriterError("Invalid rigid body shape type: " +
                                 std::to_string(rigid_body.shape_type));
        }
        w_.write_byte(rigid_body.shape_type);
        w_.write_float3(rigid_body.shape_size, "rigid body shape size");
        w_.write_float3(rigid_body.pos, "rigid body position");
        w_.write_float3(rigid_body.rot, "rigid body rotation");
        w_.write_float(rigid_body.mass, "rigid body mass");
        w_.write_float(rigid_body.linear_damping, "rigid body linear damping");
        w_.write_float(rigid_body.angular_damping, "rigid body angular damping");
        w_.write_float(rigid_body.restitution, "rigid body restitution");
        w_.write_float(rigid_body.friction, "rigid body friction");

        if (rigid_body.physics_type > 2) {
            throw PMXWriterError("Invalid rigid body physics type: " +
                                 std::to_string(rigid_body.physics_type));
        }
        w_.write_byte(rigid_body.physics_type);
    }
}

void PMXWriter::write_joints()
{
    write_count(model_.joints.size(), "joints");
    for (const PMXJoint &joint : model_.joints) {
        write_string(joint.name_local, "joint name");
        write_string(joint.name_universal, "joint universal name");
        if (joint.type != 0) {
            throw PMXWriterError("Unsupported joint type: " + std::to_string(joint.type));
        }
        w_.write_byte(joint.type);
        write_rigid_index(joint.rigid_a_index, "joint rigid A index");
        write_rigid_index(joint.rigid_b_index, "joint rigid B index");
        w_.write_float3(joint.pos, "joint position");
        w_.write_float3(joint.rot, "joint rotation");
        w_.write_float3(joint.translation_limit_min, "joint translation limit min");
        w_.write_float3(joint.translation_limit_max, "joint translation limit max");
        w_.write_float3(joint.rotation_limit_min, "joint rotation limit min");
        w_.write_float3(joint.rotation_limit_max, "joint rotation limit max");
        w_.write_float3(joint.spring_translation, "joint spring translation");
        w_.write_float3(joint.spring_rotation, "joint spring rotation");
    }
}
