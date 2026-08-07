#include "pmx_reader.h"
#include <cstdio>
#include <iostream>
#include <iomanip>

// Helper to clamp strings for display
std::string trunc(const std::string& s, size_t maxlen = 40) {
    if (s.size() <= maxlen) return s;
    return s.substr(0, maxlen - 3) + "...";
}

void dump_model_info(const PMXModel& model) {
    printf("========================================\n");
    printf("  PMX Parser - Model Dump\n");
    printf("========================================\n\n");

    // Header
    printf("[Header]\n");
    printf("  Version:        %.1f\n", model.header.version);
    printf("  Encoding:       %s\n", model.is_utf8() ? "UTF-8" : "UTF-16LE");
    printf("  Add. UVs:       %d\n", model.add_uv_count());
    printf("  Index sizes:    Vtx=%d Tex=%d Mat=%d Bon=%d Mor=%d Rig=%d\n",
           model.header.vertex_idx_size,
           model.header.texture_idx_size,
           model.header.material_idx_size,
           model.header.bone_idx_size,
           model.header.morph_idx_size,
           model.header.rigid_idx_size);
    printf("\n");

    // Model Info
    printf("[Model Info]\n");
    printf("  Name (local):   %s\n", trunc(model.name_local).c_str());
    printf("  Name (uni.):    %s\n", trunc(model.name_universal).c_str());
    printf("  Comment (local): %s\n", trunc(model.comment_local, 60).c_str());
    printf("  Comment (uni.):  %s\n", trunc(model.comment_universal, 60).c_str());
    printf("\n");

    // Counts summary
    printf("[Counts]\n");
    printf("  Vertices:       %zu\n", model.vertices.size());
    printf("  Face indices:   %zu (triangles: %zu)\n",
           model.face_indices.size(),
           model.face_indices.size() / 3);
    printf("  Textures:       %zu\n", model.textures.size());
    printf("  Materials:      %zu\n", model.materials.size());
    printf("  Bones:          %zu\n", model.bones.size());
    printf("  Morphs:         %zu\n", model.morphs.size());
    printf("  Display frames: %zu\n", model.display_frames.size());
    printf("  Rigid bodies:   %zu\n", model.rigid_bodies.size());
    printf("  Joints:         %zu\n", model.joints.size());
    printf("\n");

    // Vertex summary (first 3 and last)
    printf("[Vertex Sample (first 3)]\n");
    for (size_t i = 0; i < std::min(size_t(3), model.vertices.size()); i++) {
        const auto& v = model.vertices[i];
        printf("  Vtx[%zu]: pos=(%.4f, %.4f, %.4f) uv=(%.4f, %.4f) weight=%d bones=[",
               i, v.pos[0], v.pos[1], v.pos[2], v.uv[0], v.uv[1], (int)v.weight_type);
        for (size_t b = 0; b < v.bone_indices.size(); b++) {
            if (b > 0) printf(", ");
            printf("%d:%.3f", v.bone_indices[b], v.bone_weights[b]);
        }
        printf("] edge=%.2f\n", v.edge_factor);
    }
    printf("\n");

    // Weight type distribution
    int bdef1 = 0, bdef2 = 0, bdef4 = 0, sdef = 0, qdef = 0;
    for (const auto& v : model.vertices) {
        switch (v.weight_type) {
        case BoneWeightType::BDEF1: bdef1++; break;
        case BoneWeightType::BDEF2: bdef2++; break;
        case BoneWeightType::BDEF4: bdef4++; break;
        case BoneWeightType::SDEF:  sdef++;  break;
        case BoneWeightType::QDEF:  qdef++;  break;
        }
    }
    printf("[Weight Distribution]\n");
    printf("  BDEF1: %d  BDEF2: %d  BDEF4: %d  SDEF: %d  QDEF: %d\n",
           bdef1, bdef2, bdef4, sdef, qdef);
    printf("\n");

    // Textures
    printf("[Textures]\n");
    for (size_t i = 0; i < model.textures.size(); i++) {
        printf("  Tex[%zu]: %s\n", i, trunc(model.textures[i].path, 60).c_str());
    }
    printf("\n");

    // Materials
    printf("[Materials]\n");
    for (size_t i = 0; i < model.materials.size(); i++) {
        const auto& m = model.materials[i];
        printf("  Mat[%zu]: '%s' faces=%d tex=%d sph=%d%s flag=0x%02x\n",
               i, trunc(m.name_local).c_str(),
               m.face_vertex_count,
               m.texture_idx, m.sphere_texture_idx,
               m.sphere_mode == SphereMode::Sphere ? "(sphere)" :
               m.sphere_mode == SphereMode::Cube ? "(cube)" : "",
               m.flag);
    }
    printf("\n");

    // Bones - first 5
    printf("[Bones (first 5)]\n");
    for (size_t i = 0; i < std::min(size_t(5), model.bones.size()); i++) {
        const auto& b = model.bones[i];
        printf("  Bone[%zu]: '%s' parent=%d order=%d flag=0x%04x IK=%s\n",
               i, trunc(b.name_local).c_str(),
               b.parent_index, b.transform_order, b.flag,
               (b.flag & BONE_FLAG_IK) ? "YES" : "no");
    }
    if (model.bones.size() > 5) {
        printf("  ... and %zu more bones\n", model.bones.size() - 5);
    }
    printf("\n");

    // Morphs
    printf("[Morphs]\n");
    for (size_t i = 0; i < std::min(size_t(8), model.morphs.size()); i++) {
        const auto& m = model.morphs[i];
        const char* type_names[] = {
            "Group", "Vertex", "Bone", "UV", "UV2", "UV3", "UV4",
            "Material", "Flip", "Impulse"
        };
        const char* type_name = (size_t)m.type < 10 ? type_names[(int)m.type] : "?";
        const char* panel_names[] = {"Eyebrow", "Eye", "Mouth", "Other"};
        const char* panel_name = m.panel < 4 ? panel_names[m.panel] : "?";
        printf("  Morph[%zu]: '%s' type=%s panel=%s\n",
               i, trunc(m.name_local).c_str(), type_name, panel_name);
    }
    if (model.morphs.size() > 8) {
        printf("  ... and %zu more morphs\n", model.morphs.size() - 8);
    }
    printf("\n");

    // Rigid bodies
    printf("[Rigid Bodies]\n");
    for (size_t i = 0; i < std::min(size_t(5), model.rigid_bodies.size()); i++) {
        const auto& rb = model.rigid_bodies[i];
        printf("  RB[%zu]: '%s' bone=%d group=%d shape=%d mass=%.1f\n",
               i, trunc(rb.name_local).c_str(),
               rb.bone_index, rb.collision_group, rb.shape_type, rb.mass);
    }
    if (model.rigid_bodies.size() > 5) {
        printf("  ... and %zu more rigid bodies\n", model.rigid_bodies.size() - 5);
    }
    printf("\n");

    // Joints
    printf("[Joints]\n");
    for (size_t i = 0; i < std::min(size_t(5), model.joints.size()); i++) {
        const auto& j = model.joints[i];
        printf("  Joint[%zu]: '%s' rigidA=%d rigidB=%d\n",
               i, trunc(j.name_local).c_str(), j.rigid_a_index, j.rigid_b_index);
    }
    if (model.joints.size() > 5) {
        printf("  ... and %zu more joints\n", model.joints.size() - 5);
    }

    printf("\n========================================\n");
    printf("  Parse complete.\n");
    printf("========================================\n");
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: pmx_reader <path_to_pmx>\n");
        return 1;
    }

    try {
        PMXModel model = PMXReader::read(argv[1]);
        dump_model_info(model);
        return 0;
    }
    catch (const PMXReaderError& e) {
        fprintf(stderr, "ERROR: %s\n", e.what());
        return 1;
    }
    catch (const std::exception& e) {
        fprintf(stderr, "UNEXPECTED ERROR: %s\n", e.what());
        return 1;
    }
}
