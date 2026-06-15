#pragma once

#include <cstdio>
#include <utility>

namespace crdm {

class RAIIFile {
public:
    RAIIFile() : fp_(nullptr), owns_(false) {}

    explicit RAIIFile(FILE* fp, bool owns = true) : fp_(fp), owns_(owns) {}

    RAIIFile(const char* path, const char* mode) : fp_(nullptr), owns_(true) {
        fp_ = std::fopen(path, mode);
    }

    ~RAIIFile() {
        close();
    }

    RAIIFile(const RAIIFile&) = delete;
    RAIIFile& operator=(const RAIIFile&) = delete;

    RAIIFile(RAIIFile&& other) noexcept : fp_(other.fp_), owns_(other.owns_) {
        other.fp_ = nullptr;
        other.owns_ = false;
    }

    RAIIFile& operator=(RAIIFile&& other) noexcept {
        if (this != &other) {
            close();
            fp_ = other.fp_;
            owns_ = other.owns_;
            other.fp_ = nullptr;
            other.owns_ = false;
        }
        return *this;
    }

    [[nodiscard]] bool is_open() const noexcept { return fp_ != nullptr; }
    [[nodiscard]] FILE* get() const noexcept { return fp_; }
    [[nodiscard]] operator FILE*() const noexcept { return fp_; }

    explicit operator bool() const noexcept { return fp_ != nullptr; }

    FILE* release() noexcept {
        FILE* tmp = fp_;
        fp_ = nullptr;
        owns_ = false;
        return tmp;
    }

    void close() noexcept {
        if (fp_ != nullptr && owns_) {
            std::fclose(fp_);
        }
        fp_ = nullptr;
        owns_ = false;
    }

private:
    FILE* fp_;
    bool owns_;
};

template <typename Func>
class ScopeGuard {
public:
    explicit ScopeGuard(Func&& f) noexcept : func_(std::move(f)), active_(true) {}
    explicit ScopeGuard(const Func& f) noexcept : func_(f), active_(true) {}

    ~ScopeGuard() noexcept {
        if (active_) {
            try { func_(); } catch (...) {}
        }
    }

    ScopeGuard(const ScopeGuard&) = delete;
    ScopeGuard& operator=(const ScopeGuard&) = delete;

    ScopeGuard(ScopeGuard&& other) noexcept
        : func_(std::move(other.func_)), active_(other.active_) {
        other.active_ = false;
    }

    void dismiss() noexcept { active_ = false; }

private:
    Func func_;
    bool active_;
};

template <typename F>
ScopeGuard<F> make_scope_guard(F&& f) {
    return ScopeGuard<F>(std::forward<F>(f));
}

} // namespace crdm
