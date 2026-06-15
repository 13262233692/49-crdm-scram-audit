#pragma once

#include <cstdint>
#include <cstddef>

namespace crdm {

class MemoryMappedFile {
public:
    MemoryMappedFile();
    ~MemoryMappedFile();

    MemoryMappedFile(const MemoryMappedFile&) = delete;
    MemoryMappedFile& operator=(const MemoryMappedFile&) = delete;

    bool open(const char* filename);
    void close();

    [[nodiscard]] const uint8_t* data() const noexcept { return data_; }
    [[nodiscard]] size_t size() const noexcept { return size_; }
    [[nodiscard]] bool is_open() const noexcept { return data_ != nullptr; }

private:
    uint8_t* data_;
    size_t size_;
#ifdef _WIN32
    void* file_handle_;
    void* map_handle_;
#else
    int fd_;
#endif
};

} // namespace crdm
