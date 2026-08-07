#include "pmx_reader.h"
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <vector>
#include <windows.h>
#include <windows.h>

PMXModel PMXReader::read(const std::string& filepath) {
    // Read entire file into memory
    FILE* f = nullptr;
#ifdef _MSC_VER
    fopen_s(&f, filepath.c_str(), "rb");
#else
    f = fopen(filepath.c_str(), "rb");
#endif
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

    PMXReader reader(buf.data(), buf.size(), filepath);
    return reader.read_internal();
}

PMXReader::PMXReader(const uint8_t* data, size_t size, const std::string& filepath)
    : r_(data, size), filepath_(filepath) {}

PMXModel PMXReader::read_internal() {
    read_header();
    read_model_info();
    read_vertices();
    read_faces();
    read_textures();
    read_materials();
    read_bones();
    read_morphs();
    read_display_frames();
    read_rigid_bodies();
    read_joints();
    if (r_.pos() != r_.size()) {
        throw PMXReaderError("Unexpected trailing data: " +
                             std::to_string(r_.size() - r_.pos()) + " bytes");
    }
    return std::move(model_);
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

std::string PMXReader::read_string() {
    int len = r_.read_int();
    if (len <= 0) return "";

    if (r_.pos() + len > r_.size()) throw PMXReaderError("Unexpected EOF reading string");

    if (model_.is_utf8()) {
        std::string result((const char*)&r_.data()[r_.pos()], len);
        r_.skip(len);
        return result;
    }
    else {
        // UTF-16LE: convert to UTF-8 via Windows API
        int wchar_len = len / 2;
        if (wchar_len > 0) {
            // Build aligned wchar_t buffer from raw bytes
            std::vector<wchar_t> wbuf(wchar_len + 1, 0);
            for (int i = 0; i < wchar_len; i++) {
                const uint8_t* p = &r_.data()[r_.pos() + i * 2];
                wbuf[i] = (wchar_t)(p[0] | (p[1] << 8));
            }
            // Count actual non-null chars (strings are null-terminated in PMX)
            int actual = 0;
            while (actual < wchar_len && wbuf[actual] != 0) actual++;
            if (actual > 0) {
                int utf8_len = WideCharToMultiByte(
                    CP_UTF8, 0, wbuf.data(), actual, NULL, 0, NULL, NULL);
                if (utf8_len > 0) {
                    std::string result(utf8_len, '\0');
                    WideCharToMultiByte(
                        CP_UTF8, 0, wbuf.data(), actual,
                        &result[0], utf8_len, NULL, NULL);
                    r_.skip(len);
                    return result;
                }
            }
        }
        r_.skip(len);
        return "(non-latin text)";
    }
}

// ===== Header =====

void PMXReader::read_header() {
    // Signature
    if (r_.read_byte() != 'P' || r_.read_byte() != 'M' ||
        r_.read_byte() != 'X' || r_.read_byte() != ' ') {
        throw PMXReaderError("Not a valid PMX file");
    }
    model_.header.version = r_.read_float();
    model_.header.reserved_byte = r_.read_byte();
    model_.header.encoding = r_.read_byte();
    model_.header.add_uv_cnt = r_.read_byte();
    model_.header.vertex_idx_size = r_.read_byte();
    model_.header.texture_idx_size = r_.read_byte();
    model_.header.material_idx_size = r_.read_byte();
    model_.header.bone_idx_size = r_.read_byte();
    model_.header.morph_idx_size = r_.read_byte();
    model_.header.rigid_idx_size = r_.read_byte();
}

void PMXReader::read_model_info() {
    model_.name_local = read_string();
    model_.name_universal = read_string();
    model_.comment_local = read_string();
    model_.comment_universal = read_string();
}

// ===== Vertices =====

void PMXReader::read_vertices() {
    int count = r_.read_int();
    model_.vertices.reserve(count);
    int add_uv = model_.add_uv_count();

    for (int i = 0; i < count; i++) {
        PMXVertex v;
        memset(&v, 0, sizeof(v));
        r_.read_float3(v.pos);
        r_.read_float3(v.normal);
        v.uv[0] = r_.read_float();
        v.uv[1] = r_.read_float();
        for (int j = 0; j < add_uv; j++) {
            r_.read_float4(v.uv);  // skip, reusing storage
        }
        v.weight_type = (BoneWeightType)r_.read_byte();

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
    int total = r_.read_int();
    model_.face_indices.reserve(total);
    for (int i = 0; i < total; i++) {
        model_.face_indices.push_back(read_vertex_index());
    }
}

void PMXReader::read_textures() {
    int count = r_.read_int();
    model_.textures.reserve(count);
    for (int i = 0; i < count; i++) {
        PMXTexture t;
        t.path = read_string();
        model_.textures.push_back(std::move(t));
    }
}

void PMXReader::read_materials() {
    int count = r_.read_int();
    model_.materials.reserve(count);
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
        mat.sphere_mode = (SphereMode)r_.read_byte();
        mat.toon_flag = r_.read_byte();
        if (mat.toon_flag == 0) mat.toon_internal_value = r_.read_byte();
        else mat.toon_texture_idx = read_texture_index();
        mat.memo = read_string();
        mat.face_vertex_count = r_.read_int();
        model_.materials.push_back(std::move(mat));
    }
}

void PMXReader::read_bones() {
    int count = r_.read_int();
    model_.bones.reserve(count);
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
            int link_count = r_.read_int();
            for (int j = 0; j < link_count; j++) {
                PMXIKLink link;
                memset(&link, 0, sizeof(link));
                link.bone_index = read_bone_index();
                link.limit_angle = (r_.read_byte() != 0);
                if (link.limit_angle) { r_.read_float3(link.limit_min); r_.read_float3(link.limit_max); }
                bone.ik_links.push_back(link);
            }
        }
        model_.bones.push_back(std::move(bone));
    }
}

void PMXReader::read_morphs() {
    int count = r_.read_int();
    model_.morphs.reserve(count);
    for (int i = 0; i < count; i++) {
        PMXMorph m;
        memset(&m, 0, sizeof(m));
        m.name_local = read_string();
        m.name_universal = read_string();
        m.panel = r_.read_byte();
        m.type = (MorphType)r_.read_byte();
        int off_count = r_.read_int();
        
        // Handle all morph types properly
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
                r_.read_float3(o.velocity); r_.read_float3(o.torque);
                m.impulse_offsets.push_back(o);
            }
            break;
        default:
            // Unknown morph type - skip data (fake read of off_count * 14 bytes)
            r_.skip((size_t)off_count * 14);
            break;
        }
        model_.morphs.push_back(std::move(m));
    }
}

void PMXReader::read_display_frames() {
    int count = r_.read_int();
    model_.display_frames.reserve(count);
    for (int i = 0; i < count; i++) {
        PMXDisplayFrame f;
        f.name_local = read_string();
        f.name_universal = read_string();
        f.flag = r_.read_byte();
        int item_count = r_.read_int();
        for (int j = 0; j < item_count; j++) {
            PMXDisplayFrame::FrameItem item;
            item.type = r_.read_byte();
            item.index = (item.type == 0) ? read_bone_index() : read_morph_index();
            f.items.push_back(item);
        }
        model_.display_frames.push_back(std::move(f));
    }
}

void PMXReader::read_rigid_bodies() {
    int count = r_.read_int();
    model_.rigid_bodies.reserve(count);
    for (int i = 0; i < count; i++) {
        PMXRigidBody rb;
        memset(&rb, 0, sizeof(rb));
        rb.bone_index = -1;
        rb.name_local = read_string();
        rb.name_universal = read_string();
        rb.bone_index = read_bone_index();
        rb.collision_group = r_.read_byte();
        rb.no_collision_group = r_.read_unsigned_short();
        rb.shape_type = r_.read_byte();
        r_.read_float3(rb.shape_size);
        r_.read_float3(rb.pos);
        r_.read_float3(rb.rot);
        rb.mass = r_.read_float();
        rb.linear_damping = r_.read_float();
        rb.angular_damping = r_.read_float();
        rb.restitution = r_.read_float();
        rb.friction = r_.read_float();
        rb.physics_type = r_.read_byte();
        model_.rigid_bodies.push_back(std::move(rb));
    }
}

void PMXReader::read_joints() {
    int count = r_.read_int();
    model_.joints.reserve(count);
    for (int i = 0; i < count; i++) {
        PMXJoint j;
        memset(&j, 0, sizeof(j));
        j.name_local = read_string();
        j.name_universal = read_string();
        j.type = r_.read_byte();
        j.rigid_a_index = read_rigid_index();
        j.rigid_b_index = read_rigid_index();
        r_.read_float3(j.pos);
        r_.read_float3(j.rot);
        r_.read_float3(j.translation_limit_min);
        r_.read_float3(j.translation_limit_max);
        r_.read_float3(j.rotation_limit_min);
        r_.read_float3(j.rotation_limit_max);
        r_.read_float3(j.spring_translation);
        r_.read_float3(j.spring_rotation);
        model_.joints.push_back(std::move(j));
    }
}
