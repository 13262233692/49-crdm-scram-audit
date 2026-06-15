#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <array>

namespace crdm {

class WaveletPacketDecomposer {
public:
    static constexpr int DB4_FILTER_LEN = 8;
    static constexpr int DECOMP_LEVELS = 3;
    static constexpr int NUM_SUBBANDS = 1 << DECOMP_LEVELS;

    WaveletPacketDecomposer();

    void reset(size_t max_samples_per_channel);

    void decompose_channel(const float* input, size_t n,
                           std::array<std::vector<float>, NUM_SUBBANDS>& subbands);

private:
    static const std::array<double, DB4_FILTER_LEN> db4_low_decomp;
    static const std::array<double, DB4_FILTER_LEN> db4_high_decomp;

    static void single_level_decompose(const float* input, size_t n,
                                       const double* coeffs_lo, const double* coeffs_hi,
                                       std::vector<double>& approx,
                                       std::vector<double>& detail,
                                       size_t& out_len);

    static void single_level_decompose_d(const double* input, size_t n,
                                         const double* coeffs_lo, const double* coeffs_hi,
                                         std::vector<double>& approx,
                                         std::vector<double>& detail,
                                         size_t& out_len);

    std::vector<double> work_buf_a_;
    std::vector<double> work_buf_d_;
    std::vector<double> work_buf_la_;
    std::vector<double> work_buf_ld_;
    std::vector<double> work_buf_ha_;
    std::vector<double> work_buf_hd_;
    std::vector<double> level1_approx_;
    std::vector<double> level1_detail_;
    std::vector<double> level2_[4];
    std::vector<double> level3_[8];
};

} // namespace crdm
