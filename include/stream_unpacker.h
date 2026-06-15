#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include "memory_mapped_file.h"

namespace crdm {

#pragma pack(push, 1)
struct FileHeader {
    char magic[8];
    uint32_t version;
    uint32_t num_channels;
    uint32_t sample_rate_khz;
    uint64_t num_samples;
    uint64_t timestamp_resolution_ns;
    uint8_t reserved[28];
};
#pragma pack(pop)

static_assert(sizeof(FileHeader) == 64, "FileHeader must be 64 bytes");

struct FrameStats {
    uint64_t total_frames = 0;
    uint64_t good_frames = 0;
    uint64_t malformed_ts_frames = 0;
    uint64_t malformed_bounds_frames = 0;
    uint64_t malformed_value_frames = 0;
};

class StreamUnpacker {
public:
    StreamUnpacker() = default;

    bool init(const MemoryMappedFile& mmf);

    [[nodiscard]] uint32_t num_channels() const noexcept { return num_channels_; }
    [[nodiscard]] uint64_t num_samples() const noexcept { return safe_num_samples_; }
    [[nodiscard]] uint64_t declared_samples() const noexcept { return declared_samples_; }
    [[nodiscard]] uint32_t sample_rate_khz() const noexcept { return sample_rate_khz_; }
    [[nodiscard]] size_t frame_stride() const { return frame_stride_; }
    [[nodiscard]] const FrameStats& stats() const noexcept { return stats_; }
    [[nodiscard]] bool has_errors() const noexcept {
        return stats_.malformed_ts_frames > 0 ||
               stats_.malformed_bounds_frames > 0 ||
               stats_.malformed_value_frames > 0;
    }

    class Iterator {
    public:
        Iterator(const uint8_t* base, const uint8_t* end,
                 uint32_t num_channels, size_t stride,
                 uint64_t frame_idx, uint64_t max_frames, StreamUnpacker* owner)
            : ptr_(base), end_ptr_(end), num_channels_(num_channels), frame_stride_(stride),
              frame_idx_(frame_idx), max_frames_(max_frames), owner_(owner),
              prev_ts_(0), bad_(false) {}

        Iterator& operator++() {
            ++frame_idx_;
            ptr_ += frame_stride_;
            if (frame_idx_ >= max_frames_ || ptr_ >= end_ptr_) {
                ptr_ = end_ptr_;
                frame_idx_ = max_frames_;
            }
            return *this;
        }

        bool operator!=(const Iterator& other) const {
            if (frame_idx_ >= max_frames_ && other.frame_idx_ >= other.max_frames_) {
                return false;
            }
            if (frame_idx_ >= max_frames_ || ptr_ >= end_ptr_) {
                return other.frame_idx_ < other.max_frames_;
            }
            if (other.frame_idx_ >= other.max_frames_) {
                return frame_idx_ < max_frames_;
            }
            return ptr_ != other.ptr_;
        }

        bool read(uint64_t& ts, float* values) {
            if (frame_idx_ >= max_frames_ || ptr_ >= end_ptr_) {
                return false;
            }
            if (static_cast<size_t>(end_ptr_ - ptr_) < frame_stride_) {
                if (owner_) ++owner_->stats_.malformed_bounds_frames;
                bad_ = true;
                ptr_ = end_ptr_;
                frame_idx_ = max_frames_;
                return false;
            }

            const uint8_t* p = ptr_;
            std::memcpy(&ts, p, sizeof(uint64_t));
            p += sizeof(uint64_t);

            bool ts_ok = true;
            if (frame_idx_ == 0) {
                if (ts < 946684800000000000ULL || ts > 4102444800000000000ULL) {
                    ts_ok = false;
                }
            } else {
                uint64_t delta = (ts >= prev_ts_) ? (ts - prev_ts_) : (prev_ts_ - ts);
                if (delta > 3600ULL * 1000000000ULL) {
                    ts_ok = false;
                }
            }
            prev_ts_ = ts;

            bool values_ok = true;
            for (uint32_t i = 0; i < num_channels_; ++i) {
                int16_t raw;
                std::memcpy(&raw, p, sizeof(int16_t));
                p += sizeof(int16_t);
                float v = static_cast<float>(raw) * 3.051850947599719e-05f;
                values[i] = v;
                if (v < -2.0f || v > 2.0f) {
                    values_ok = false;
                }
            }

            if (!ts_ok)     { if (owner_) ++owner_->stats_.malformed_ts_frames;     bad_ = true; }
            if (!values_ok) { if (owner_) ++owner_->stats_.malformed_value_frames;  bad_ = true; }

            if (owner_) {
                ++owner_->stats_.total_frames;
                if (!bad_) ++owner_->stats_.good_frames;
            }

            return true;
        }

        uint64_t frame_index() const { return frame_idx_; }
        bool is_bad() const { return bad_; }

    private:
        const uint8_t* ptr_;
        const uint8_t* end_ptr_;
        uint32_t num_channels_;
        size_t frame_stride_;
        uint64_t frame_idx_;
        uint64_t max_frames_;
        StreamUnpacker* owner_;
        uint64_t prev_ts_;
        bool bad_;
    };

    Iterator begin() { return Iterator(data_start_, data_end_, num_channels_, frame_stride_, 0, safe_num_samples_, this); }
    Iterator end()   { return Iterator(data_end_,   data_end_, num_channels_, frame_stride_, safe_num_samples_, safe_num_samples_, this); }

private:
    uint32_t num_channels_ = 0;
    uint64_t declared_samples_ = 0;
    uint64_t safe_num_samples_ = 0;
    uint32_t sample_rate_khz_ = 0;
    size_t frame_stride_ = 0;
    const uint8_t* data_start_ = nullptr;
    const uint8_t* data_end_ = nullptr;
    FrameStats stats_;
};

} // namespace crdm
