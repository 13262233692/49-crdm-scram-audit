#include "stream_unpacker.h"
#include <cstring>

namespace crdm {

bool StreamUnpacker::init(const MemoryMappedFile& mmf) {
    if (!mmf.is_open() || mmf.size() < sizeof(FileHeader)) {
        return false;
    }

    const FileHeader* header = reinterpret_cast<const FileHeader*>(mmf.data());

    if (std::memcmp(header->magic, "CRDMAE01", 8) != 0) {
        return false;
    }

    num_channels_ = header->num_channels;
    num_samples_ = header->num_samples;
    sample_rate_khz_ = header->sample_rate_khz;

    const size_t frame_size = sizeof(uint64_t) + num_channels_ * sizeof(int16_t);
    const size_t expected_data_size = frame_size * num_samples_;

    if (mmf.size() < sizeof(FileHeader) + expected_data_size) {
        return false;
    }

    data_start_ = mmf.data() + sizeof(FileHeader);
    data_end_ = data_start_ + expected_data_size;

    return true;
}

} // namespace crdm
