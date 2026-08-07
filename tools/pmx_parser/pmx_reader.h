#ifndef PMX_READER_H
#define PMX_READER_H

#include "pmx_types.h"
#include <cstdio>
#include <string>
#include <stdexcept>
#include <vector>
#include <cstdint>

class PMXReaderError : public std::runtime_error {
public:
    explicit PMXReaderError(const std::string& msg) : std::runtime_error(msg) {}
};

// Memory-based binary reader - no file I/O issues
class MemReader {
public:
    MemReader(const uint8_t* data, size_t size)
        : data_(data), size_(size), pos_(0) {}

    size_t pos() const { return pos_; }
    size_t size() const { return size_; }
    const uint8_t* data() const { return data_; }

    uint8_t read_byte() {
        if (pos_ + 1 > size_) throw PMXReaderError("Unexpected EOF");
        return data_[pos_++];
    }

    int8_t read_signed_byte() { return (int8_t)read_byte(); }

    int16_t read_short() {
        if (pos_ + 2 > size_) throw PMXReaderError("Unexpected EOF");
        int16_t v = (int16_t)(data_[pos_] | (data_[pos_+1] << 8));
        pos_ += 2;
        return v;
    }

    uint16_t read_unsigned_short() {
        if (pos_ + 2 > size_) throw PMXReaderError("Unexpected EOF");
        uint16_t v = (uint16_t)(data_[pos_] | (data_[pos_+1] << 8));
        pos_ += 2;
        return v;
    }

    int32_t read_int() {
        if (pos_ + 4 > size_) throw PMXReaderError("Unexpected EOF");
        int32_t v = (int32_t)(data_[pos_] | (data_[pos_+1] << 8) |
                              (data_[pos_+2] << 16) | (data_[pos_+3] << 24));
        pos_ += 4;
        return v;
    }

    uint32_t read_unsigned_int() {
        if (pos_ + 4 > size_) throw PMXReaderError("Unexpected EOF");
        uint32_t v = (uint32_t)(data_[pos_] | (data_[pos_+1] << 8) |
                                (data_[pos_+2] << 16) | (data_[pos_+3] << 24));
        pos_ += 4;
        return v;
    }

    float read_float() {
        uint32_t raw = read_unsigned_int();
        float f;
        memcpy(&f, &raw, sizeof(f));
        return f;
    }

    void read_float3(float out[3]) {
        out[0] = read_float();
        out[1] = read_float();
        out[2] = read_float();
    }

    void read_float4(float out[4]) {
        out[0] = read_float();
        out[1] = read_float();
        out[2] = read_float();
        out[3] = read_float();
    }

    void skip(size_t n) {
        if (pos_ + n > size_) throw PMXReaderError("Unexpected EOF");
        pos_ += n;
    }

    void seek(size_t p) {
        if (p > size_) throw PMXReaderError("Seek past end");
        pos_ = p;
    }

private:
    const uint8_t* data_;
    size_t size_;
    size_t pos_;
};

class PMXReader {
public:
    static PMXModel read(const std::string& filepath);

private:
    PMXReader(const uint8_t* data, size_t size, const std::string& filepath);
    ~PMXReader() = default;

    PMXModel read_internal();

    // Index helpers
    int read_vertex_index();
    int read_bone_index();
    int read_texture_index();
    int read_material_index();
    int read_morph_index();
    int read_rigid_index();
    int read_index(uint8_t size, bool is_signed);

    std::string read_string();

    // Section readers
    void read_header();
    void read_model_info();
    void read_vertices();
    void read_faces();
    void read_textures();
    void read_materials();
    void read_bones();
    void read_morphs();
    void read_display_frames();
    void read_rigid_bodies();
    void read_joints();

    MemReader r_;
    std::string filepath_;
    PMXModel model_;
};

#endif // PMX_READER_H
