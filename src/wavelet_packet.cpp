#include "wavelet_packet.h"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace crdm {

const std::array<double, WaveletPacketDecomposer::DB4_FILTER_LEN>
WaveletPacketDecomposer::db4_low_decomp = {{
    -0.0544158422431072,
     0.3128715909143166,
    -0.6756307362973195,
     0.585354683654216,
     0.0158291052563823,
    -0.2840155429615824,
    -0.0004724845739124,
     0.1287474266204893
}};

const std::array<double, WaveletPacketDecomposer::DB4_FILTER_LEN>
WaveletPacketDecomposer::db4_high_decomp = {{
     0.1287474266204893,
     0.0004724845739124,
    -0.2840155429615824,
    -0.0158291052563823,
     0.585354683654216,
     0.6756307362973195,
     0.3128715909143166,
     0.0544158422431072
}};

WaveletPacketDecomposer::WaveletPacketDecomposer() = default;

void WaveletPacketDecomposer::reset(size_t max_samples_per_channel) {
    const size_t l1 = max_samples_per_channel / 2 + DB4_FILTER_LEN;
    const size_t l2 = l1 / 2 + DB4_FILTER_LEN;
    const size_t l3 = l2 / 2 + DB4_FILTER_LEN;

    work_buf_a_.resize(max_samples_per_channel + DB4_FILTER_LEN * 2);
    work_buf_d_.resize(max_samples_per_channel + DB4_FILTER_LEN * 2);
    work_buf_la_.resize(l1 * 2);
    work_buf_ld_.resize(l1 * 2);
    work_buf_ha_.resize(l1 * 2);
    work_buf_hd_.resize(l1 * 2);
    level1_approx_.resize(l1);
    level1_detail_.resize(l1);
    for (int i = 0; i < 4; ++i) level2_[i].resize(l2);
    for (int i = 0; i < 8; ++i) level3_[i].resize(l3);
}

void WaveletPacketDecomposer::single_level_decompose(
    const float* input, size_t n,
    const double* coeffs_lo, const double* coeffs_hi,
    std::vector<double>& approx, std::vector<double>& detail,
    size_t& out_len)
{
    out_len = n / 2;
    if (approx.size() < out_len) approx.resize(out_len);
    if (detail.size() < out_len) detail.resize(out_len);

    const int L = DB4_FILTER_LEN;

    for (size_t k = 0; k < out_len; ++k) {
        double sum_lo = 0.0;
        double sum_hi = 0.0;
        const size_t base = 2 * k;

        for (int j = 0; j < L; ++j) {
            long long idx = static_cast<long long>(base) + j - (L - 2);
            if (idx < 0) idx = ((idx % static_cast<long long>(n)) + n) % n;
            if (idx >= static_cast<long long>(n)) idx = idx % n;
            const double sample = static_cast<double>(input[static_cast<size_t>(idx)]);
            sum_lo += coeffs_lo[j] * sample;
            sum_hi += coeffs_hi[j] * sample;
        }
        approx[k] = sum_lo;
        detail[k] = sum_hi;
    }
}

void WaveletPacketDecomposer::single_level_decompose_d(
    const double* input, size_t n,
    const double* coeffs_lo, const double* coeffs_hi,
    std::vector<double>& approx, std::vector<double>& detail,
    size_t& out_len)
{
    out_len = n / 2;
    if (approx.size() < out_len) approx.resize(out_len);
    if (detail.size() < out_len) detail.resize(out_len);

    const int L = DB4_FILTER_LEN;

    for (size_t k = 0; k < out_len; ++k) {
        double sum_lo = 0.0;
        double sum_hi = 0.0;
        const size_t base = 2 * k;

        for (int j = 0; j < L; ++j) {
            long long idx = static_cast<long long>(base) + j - (L - 2);
            if (idx < 0) idx = ((idx % static_cast<long long>(n)) + n) % n;
            if (idx >= static_cast<long long>(n)) idx = idx % n;
            const double sample = input[static_cast<size_t>(idx)];
            sum_lo += coeffs_lo[j] * sample;
            sum_hi += coeffs_hi[j] * sample;
        }
        approx[k] = sum_lo;
        detail[k] = sum_hi;
    }
}

void WaveletPacketDecomposer::decompose_channel(
    const float* input, size_t n,
    std::array<std::vector<float>, NUM_SUBBANDS>& subbands)
{
    size_t len1;
    single_level_decompose(input, n,
                           db4_low_decomp.data(), db4_high_decomp.data(),
                           level1_approx_, level1_detail_, len1);

    size_t len2a, len2d;
    single_level_decompose_d(level1_approx_.data(), len1,
                             db4_low_decomp.data(), db4_high_decomp.data(),
                             level2_[0], level2_[1], len2a);
    single_level_decompose_d(level1_detail_.data(), len1,
                             db4_low_decomp.data(), db4_high_decomp.data(),
                             level2_[2], level2_[3], len2d);

    const size_t* len2_ptrs[4] = {&len2a, &len2a, &len2d, &len2d};
    size_t final_lens[8];

    for (int i = 0; i < 4; ++i) {
        size_t len3;
        single_level_decompose_d(level2_[i].data(), *len2_ptrs[i],
                                 db4_low_decomp.data(), db4_high_decomp.data(),
                                 level3_[2*i], level3_[2*i+1], len3);
        final_lens[2*i] = len3;
        final_lens[2*i+1] = len3;
    }

    for (int i = 0; i < NUM_SUBBANDS; ++i) {
        subbands[i].resize(final_lens[i]);
        for (size_t j = 0; j < final_lens[i]; ++j) {
            subbands[i][j] = static_cast<float>(level3_[i][j]);
        }
    }
}

} // namespace crdm
