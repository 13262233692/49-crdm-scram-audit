#include "memory_mapped_file.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace crdm {

MemoryMappedFile::MemoryMappedFile()
    : data_(nullptr), size_(0)
#ifdef _WIN32
    , file_handle_(INVALID_HANDLE_VALUE), map_handle_(nullptr)
#else
    , fd_(-1)
#endif
{
}

MemoryMappedFile::~MemoryMappedFile() {
    close();
}

bool MemoryMappedFile::open(const char* filename) {
    close();

#ifdef _WIN32
    file_handle_ = CreateFileA(filename, GENERIC_READ, FILE_SHARE_READ,
        nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file_handle_ == INVALID_HANDLE_VALUE) {
        return false;
    }

    LARGE_INTEGER li;
    if (!GetFileSizeEx(file_handle_, &li)) {
        CloseHandle(file_handle_);
        file_handle_ = INVALID_HANDLE_VALUE;
        return false;
    }
    size_ = static_cast<size_t>(li.QuadPart);

    map_handle_ = CreateFileMappingA(file_handle_, nullptr, PAGE_READONLY,
        li.HighPart, li.LowPart, nullptr);
    if (map_handle_ == nullptr) {
        CloseHandle(file_handle_);
        file_handle_ = INVALID_HANDLE_VALUE;
        return false;
    }

    data_ = static_cast<uint8_t*>(MapViewOfFile(map_handle_, FILE_MAP_READ, 0, 0, 0));
    if (data_ == nullptr) {
        CloseHandle(map_handle_);
        map_handle_ = nullptr;
        CloseHandle(file_handle_);
        file_handle_ = INVALID_HANDLE_VALUE;
        return false;
    }
#else
    fd_ = ::open(filename, O_RDONLY);
    if (fd_ < 0) {
        return false;
    }

    struct stat st;
    if (::fstat(fd_, &st) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    size_ = static_cast<size_t>(st.st_size);

    data_ = static_cast<uint8_t*>(::mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd_, 0));
    if (data_ == MAP_FAILED) {
        data_ = nullptr;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    ::madvise(data_, size_, MADV_SEQUENTIAL);
#endif

    return true;
}

void MemoryMappedFile::close() {
    if (data_ != nullptr) {
#ifdef _WIN32
        UnmapViewOfFile(data_);
#else
        ::munmap(data_, size_);
#endif
        data_ = nullptr;
    }
    size_ = 0;

#ifdef _WIN32
    if (map_handle_ != nullptr) {
        CloseHandle(map_handle_);
        map_handle_ = nullptr;
    }
    if (file_handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(file_handle_);
        file_handle_ = INVALID_HANDLE_VALUE;
    }
#else
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
#endif
}

} // namespace crdm
