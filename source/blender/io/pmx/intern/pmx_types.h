#ifndef PMX_TYPES_H
#define PMX_TYPES_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// PMX format data structures
// Based on PMX 2.0 / 2.1 specification and mmd_tools reference

#pragma pack(push, 1)

struct PMXHeader {
    char signature[4];   // "PMX "
    float version;       // 2.0 or 2.1
    uint8_t header_size; // PMX header size, currently 8
    uint8_t encoding;    // 0=UTF-16LE, 1=UTF-8
    uint8_t add_uv_cnt;  // additional UV count
    uint8_t vertex_idx_size;  // 1, 2, or 4
    uint8_t texture_idx_size;
    uint8_t material_idx_size;
    uint8_t bone_idx_size;
    uint8_t morph_idx_size;
    uint8_t rigid_idx_size;
};

#pragma pack(pop)

// String encoding info
enum class PMXEncoding : uint8_t {
    UTF16LE = 0,
    UTF8 = 1,
};

// Bone weight types
enum class BoneWeightType : uint8_t {
    BDEF1 = 0,
    BDEF2 = 1,
    BDEF4 = 2,
    SDEF = 3,
    QDEF = 4,  // PMX 2.1+
};

// --- PMX Model data ---

struct PMXVertex {
    float pos[3];
    float normal[3];
    float uv[2];
    // PMX permits up to four additional UV sets, each containing four floats.
    std::array<std::array<float, 4>, 4> additional_uv{};
    uint8_t additional_uv_count = 0;
    BoneWeightType weight_type;
    // Bone weights (depending on type)
    // BDEF1: bone_index[1], weight=1.0
    // BDEF2: bone_index[2], weight[1] (bone[0] = weight, bone[1] = 1-weight)
    // BDEF4: bone_index[4], weight[4]
    // SDEF: bone_index[2], weight[2], sdef_c[3], sdef_r0[3], sdef_r1[3]
    // QDEF: same as BDEF4 but quaternion-based
    std::vector<int> bone_indices;
    std::vector<float> bone_weights;
    float sdef_c[3];   // SDEF center (only for SDEF)
    float sdef_r0[3];  // SDEF R0
    float sdef_r1[3];  // SDEF R1
    float edge_factor;
};

struct PMXFace {
    int vertex_count;  // should be 3 (triangle)
    std::vector<int> vertex_indices;
};

struct PMXTexture {
    std::string path;
};

enum class SphereMode : uint8_t {
    None = 0,
    Sphere = 1,   // s = sphere (matcap)
    Cube = 2,     // spa = environment
};

// Material flags (PMXMaterial::flag bitfield, from the MMD spec).
// Mirrors mmd_tools (core/pmx/__init__.py: Material.load).
enum PMXMaterialFlag : uint8_t {
    PMX_MATERIAL_FLAG_DOUBLE_SIDED    = 0x01,
    PMX_MATERIAL_FLAG_DROP_SHADOW     = 0x02,
    PMX_MATERIAL_FLAG_SELF_SHADOW_MAP = 0x04,
    PMX_MATERIAL_FLAG_SELF_SHADOW     = 0x08,
    PMX_MATERIAL_FLAG_EDGE            = 0x10,
};

struct PMXMaterial {
    std::string name_local;
    std::string name_universal;
    float diffuse[4];
    float specular[3];
    float specular_power;
    float ambient[3];
    // Toon/edge flags (bitfield)
    uint8_t flag;
    // Edge
    float edge_color[4];
    float edge_size;
    // Texture indices
    int texture_idx;           // -1 = none
    int sphere_texture_idx;    // -1 = none
    SphereMode sphere_mode;
    // Toon
    uint8_t toon_flag;         // 0=internal toon, 1=external toon texture
    int toon_texture_idx;      // valid when toon_flag=1
    uint8_t toon_internal_value;  // valid when toon_flag=0
    // Meta
    std::string memo;
    // Face count for this material (not including header count int)
    int face_vertex_count;
};

// Bone flags (from MMD spec)
enum BoneFlag : uint16_t {
    BONE_FLAG_TAIL_POS         = 0x0001,  // Tail position is a position (else bone index)
    BONE_FLAG_ROTATABLE        = 0x0002,
    BONE_FLAG_TRANSLATABLE     = 0x0004,
    BONE_FLAG_VISIBLE          = 0x0008,
    BONE_FLAG_ENABLED          = 0x0010,
    BONE_FLAG_IK               = 0x0020,
    // 0x0040 = ?, 0x0080 = ?
    BONE_FLAG_APPEND_ROTATION  = 0x0100,  // Inherit rotation
    BONE_FLAG_APPEND_TRANSLATE = 0x0200,  // Inherit translation
    BONE_FLAG_FIXED_AXIS       = 0x0400,
    BONE_FLAG_LOCAL_AXIS       = 0x0800,  // Local coordinate frame
    BONE_FLAG_PHYSICS_AFTER_DEF = 0x1000, // Deform after physics
    BONE_FLAG_EXTERNAL_PARENT  = 0x2000,  // External parent transform
};

struct PMXIKLink {
    int bone_index;
    bool limit_angle;
    float limit_min[3];
    float limit_max[3];
};

struct PMXBone {
    std::string name_local;
    std::string name_universal;
    float pos[3];
    int parent_index;     // -1 = no parent
    int tail_pos_bone;    // if >= 0: bone index; if == -2: position offset; if == -1: none
    float tail_pos_offset[3];  // valid when tail_pos_bone == -2 (position mode)
    int transform_order;  // deformation层级
    uint16_t flag;
    // For 0x0080/0x0100 (inherit):
    int inherit_parent_index;
    float inherit_parent_ratio;
    // For 0x0200 (fixed axis):
    float fixed_axis[3];
    // For 0x0400 (local axis):
    float local_x[3];
    float local_z[3];
    // For 0x0020 (IK):
    int ik_target_index;
    int ik_loop_count;
    float ik_angle_limit;
    std::vector<PMXIKLink> ik_links;
    // For 0x1000 (external parent):
    int external_parent_index;
};

// Morph types
enum class MorphType : uint8_t {
    Group = 0,
    Vertex = 1,
    Bone = 2,
    UV = 3,       // UV (1st)
    UV_2nd = 4,
    UV_3rd = 5,
    UV_4th = 6,
    /* 7 is reserved by the PMX 2.x specification. */
    Material = 8,
    Flip = 9,     // PMX 2.1+
    Impulse = 10, // PMX 2.1+
};

struct PMXVertexMorphOffset {
    int vertex_index;
    float offset[3];
};

struct PMXUVMorphOffset {
    int vertex_index;
    float offset[4];
};

struct PMXBoneMorphOffset {
    int bone_index;
    float pos[3];
    float rot[4];
};

struct PMXMaterialMorphOffset {
    int material_index;  // -1 = all
    uint8_t calc_mode;   // 0=mul, 1=add
    float diffuse[4];
    float specular[3];
    float specular_power;
    float ambient[3];
    float edge_color[4];
    float edge_size;
    float texture_factor[4];
    float sphere_texture_factor[4];
    float toon_texture_factor[4];
};

struct PMXGroupMorphOffset {
    int morph_index;
    float influence;
};

struct PMXImpulseMorphOffset {
    int rigid_index;
    uint8_t local_flag;
    float velocity[3];
    float torque[3];
};

struct PMXMorph {
    std::string name_local;
    std::string name_universal;
    uint8_t panel;     // panel category (0=eyebrow, 1=eye, 2=mouth, 3=other)
    MorphType type;
    // Offset data (varies by type)
    std::vector<PMXVertexMorphOffset> vertex_offsets;
    std::vector<PMXUVMorphOffset> uv_offsets;
    std::vector<PMXBoneMorphOffset> bone_offsets;
    std::vector<PMXMaterialMorphOffset> material_offsets;
    std::vector<PMXGroupMorphOffset> group_offsets;
    std::vector<PMXImpulseMorphOffset> impulse_offsets;
};

struct PMXDisplayFrame {
    std::string name_local;
    std::string name_universal;
    uint8_t flag;  // 0=bone frame, 1=morph frame
    // Items:
    // For bones: (uint8_t type=0, int bone_index)
    // For morphs: (uint8_t type=1, int morph_index)
    struct FrameItem {
        uint8_t type;
        int index;
    };
    std::vector<FrameItem> items;
};

struct PMXRigidBody {
    std::string name_local;
    std::string name_universal;
    int bone_index;         // -1 = no parent bone
    uint8_t collision_group;
    uint16_t no_collision_group;  // bitfield, groups that don't collide
    uint8_t shape_type;     // 0=sphere, 1=box, 2=capsule
    float shape_size[3];    // depends on shape_type
    float pos[3];
    float rot[3];           // euler angles
    float mass;
    float linear_damping;
    float angular_damping;
    float restitution;      // bounce
    float friction;
    uint8_t physics_type;   // 0=static, 1=dynamic, 2=aligned (follow bone)
};

struct PMXJoint {
    std::string name_local;
    std::string name_universal;
    uint8_t type;           // PMX joint type (0=spring 6DOF)
    int rigid_a_index;
    int rigid_b_index;
    float pos[3];
    float rot[3];
    float translation_limit_min[3];
    float translation_limit_max[3];
    float rotation_limit_min[3];
    float rotation_limit_max[3];
    float spring_translation[3];
    float spring_rotation[3];
};

// --- PMX Model container ---

struct PMXModel {
    PMXHeader header;
    std::string name_local;
    std::string name_universal;
    std::string comment_local;
    std::string comment_universal;

    std::vector<PMXVertex> vertices;
    std::vector<int> face_indices;  // flat triangle index array
    std::vector<PMXTexture> textures;
    std::vector<PMXMaterial> materials;
    std::vector<PMXBone> bones;
    std::vector<PMXMorph> morphs;
    std::vector<PMXDisplayFrame> display_frames;
    std::vector<PMXRigidBody> rigid_bodies;
    std::vector<PMXJoint> joints;

    // Parse diagnostics populated by the formal Reader.
    size_t file_size = 0;
    size_t parse_end_offset = 0;

    // Convenience
    int add_uv_count() const { return header.add_uv_cnt; }
    bool is_pmx_20() const { return header.version < 2.1f; }
    bool is_utf8() const { return header.encoding == (uint8_t)PMXEncoding::UTF8; }
};

#endif // PMX_TYPES_H
