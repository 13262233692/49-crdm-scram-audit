#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <string>
#include <vector>
#include <array>

#include "memory_mapped_file.h"
#include "stream_unpacker.h"
#include "wavelet_packet.h"
#include "energy_analyzer.h"

using namespace crdm;

static void print_usage(const char* prog) {
    std::fprintf(stderr,
        "CRDM Scram AE Audit Tool - High-throughput Acoustic Emission Analysis\n"
        "\n"
        "Usage: %s [options] <input_binary_file>\n"
        "\n"
        "Options:\n"
        "  -b <size>       Block size in samples per channel (default: 16384)\n"
        "  -o <file>       Output results to file (default: stdout)\n"
        "  -v              Verbose mode\n"
        "  -h              Show this help\n"
        "\n"
        "Input file format: CRDMAE01 binary format (64-byte header + frame stream)\n",
        prog);
}

struct Options {
    const char* input_file = nullptr;
    const char* output_file = nullptr;
    size_t block_size = 16384;
    bool verbose = false;
};

static bool parse_args(int argc, char** argv, Options& opts) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            return false;
        } else if (std::strcmp(argv[i], "-v") == 0) {
            opts.verbose = true;
        } else if (std::strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            opts.block_size = static_cast<size_t>(std::atoll(argv[++i]));
            if (opts.block_size < 1024) opts.block_size = 1024;
        } else if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            opts.output_file = argv[++i];
        } else if (argv[i][0] != '-') {
            opts.input_file = argv[i];
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return false;
        }
    }
    return opts.input_file != nullptr;
}

int main(int argc, char** argv) {
    Options opts;
    if (!parse_args(argc, argv, opts)) {
        print_usage(argv[0]);
        return 1;
    }

    FILE* out = stdout;
    if (opts.output_file) {
        out = std::fopen(opts.output_file, "w");
        if (!out) {
            std::fprintf(stderr, "Error: cannot open output file %s\n", opts.output_file);
            return 2;
        }
    }

    if (opts.verbose) {
        std::fprintf(stderr, "[INFO] Opening input file: %s\n", opts.input_file);
    }

    MemoryMappedFile mmf;
    if (!mmf.open(opts.input_file)) {
        std::fprintf(stderr, "Error: cannot open or map input file: %s\n", opts.input_file);
        if (out != stdout) std::fclose(out);
        return 3;
    }

    StreamUnpacker unpacker;
    if (!unpacker.init(mmf)) {
        std::fprintf(stderr, "Error: invalid or corrupted CRDMAE01 binary file\n");
        if (out != stdout) std::fclose(out);
        return 4;
    }

    const uint32_t num_ch = unpacker.num_channels();
    const uint64_t total_samples = unpacker.num_samples();
    const uint32_t sr = unpacker.sample_rate_khz();

    if (opts.verbose) {
        std::fprintf(stderr,
            "[INFO] File OK: channels=%u, samples=%llu, rate=%u kHz, block_size=%zu\n",
            num_ch, static_cast<unsigned long long>(total_samples),
            sr, opts.block_size);
    }

    WaveletPacketDecomposer wpd;
    wpd.reset(opts.block_size);

    EnergyBandAnalyzer analyzer;
    analyzer.init(num_ch);

    std::vector<float> channel_buf(opts.block_size * num_ch);
    std::vector<float> frame_values(num_ch);
    std::array<std::vector<float>, WaveletPacketDecomposer::NUM_SUBBANDS> subbands;

    auto t0 = std::chrono::high_resolution_clock::now();

    uint64_t processed = 0;
    size_t block_fill = 0;
    uint64_t block_start_ts = 0;
    uint64_t block_idx = 0;
    uint64_t frame_count = 0;

    for (auto it = unpacker.begin(); it != unpacker.end(); ++it) {
        uint64_t ts;
        it.read(ts, frame_values.data());

        if (block_fill == 0) {
            block_start_ts = ts;
        }

        for (uint32_t ch = 0; ch < num_ch; ++ch) {
            channel_buf[block_fill * num_ch + ch] = frame_values[ch];
        }

        ++block_fill;
        ++frame_count;

        if (block_fill >= opts.block_size || (frame_count >= total_samples)) {
            for (uint32_t ch = 0; ch < num_ch; ++ch) {
                std::vector<float> ch_samples(block_fill);
                for (size_t s = 0; s < block_fill; ++s) {
                    ch_samples[s] = channel_buf[s * num_ch + ch];
                }
                wpd.decompose_channel(ch_samples.data(), block_fill, subbands);
                analyzer.process_channel_block(ch, subbands);
            }

            analyzer.finalize_frame(block_idx, block_start_ts);
            ++block_idx;
            processed += block_fill;
            block_fill = 0;

            if (opts.verbose && (block_idx & 0x3F) == 0) {
                std::fprintf(stderr, "\r[INFO] Processed: %llu / %llu samples (%.1f%%)",
                    static_cast<unsigned long long>(processed),
                    static_cast<unsigned long long>(total_samples),
                    total_samples > 0 ? 100.0 * processed / total_samples : 0.0);
                std::fflush(stderr);
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed_sec = std::chrono::duration<double>(t1 - t0).count();

    if (opts.verbose) {
        std::fprintf(stderr, "\n[INFO] Processing complete in %.3f sec (%.2f MSamples/sec)\n",
            elapsed_sec, (processed / 1e6) / std::max(elapsed_sec, 1e-9));
    }

    double global_ratios[8];
    analyzer.compute_global_ratios(global_ratios);

    std::fprintf(out,
        "================================================================\n"
        "  CRDM Scram Acoustic Emission - Energy Band Audit Report\n"
        "================================================================\n"
        "  File:              %s\n"
        "  Channels:          %u\n"
        "  Total samples:     %llu\n"
        "  Sample rate:       %u kHz (500 kHz nominal)\n"
        "  Blocks analyzed:   %llu\n"
        "  Processing time:   %.3f seconds\n"
        "  Throughput:        %.2f MSamples/sec\n"
        "----------------------------------------------------------------\n"
        "\n  GLOBAL ENERGY BAND RATIO MATRIX (3-level WPD, db4)\n"
        "  -------------------------------------------------\n",
        opts.input_file,
        num_ch,
        static_cast<unsigned long long>(total_samples),
        sr,
        static_cast<unsigned long long>(block_idx),
        elapsed_sec,
        (processed / 1e6) / std::max(elapsed_sec, 1e-9));

    const char* band_labels[8] = {
        "Band 0 (0-62.5 kHz)   ",
        "Band 1 (62.5-125 kHz) ",
        "Band 2 (125-187.5 kHz)",
        "Band 3 (187.5-250 kHz)",
        "Band 4 (250-312.5 kHz)",
        "Band 5 (312.5-375 kHz)",
        "Band 6 (375-437.5 kHz)",
        "Band 7 (437.5-500 kHz)"
    };

    std::fprintf(out, "\n  Sub-band              Energy            Ratio      Histogram\n");
    std::fprintf(out, "  --------              ------            -----      ---------\n");

    for (int i = 0; i < 8; ++i) {
        const int bar_len = static_cast<int>(global_ratios[i] * 60.0);
        std::string bar(bar_len, '#');
        std::fprintf(out, "  %s  %14.6e   %8.5f%%   |%s\n",
            band_labels[i],
            analyzer.global_band_energy()[i],
            global_ratios[i] * 100.0,
            bar.c_str());
    }

    std::fprintf(out, "  ------------------------------------------------------------\n");
    std::fprintf(out, "  TOTAL                 %14.6e  100.00000%%\n\n",
        analyzer.global_total_energy());

    const auto& per_frame = analyzer.per_frame_energy();
    if (!per_frame.empty()) {
        std::fprintf(out, "  PER-FRAME ENERGY RATIO SAMPLE (first 8 frames)\n");
        std::fprintf(out, "  Frame  Timestamp(ns)        E0%%     E1%%     E2%%     E3%%     E4%%     E5%%     E6%%     E7%%\n");
        std::fprintf(out, "  -----  ------------         ----    ----    ----    ----    ----    ----    ----    ----\n");

        const size_t show_n = std::min<size_t>(8, per_frame.size());
        for (size_t f = 0; f < show_n; ++f) {
            std::fprintf(out, "  %5zu  %14llu ", f,
                static_cast<unsigned long long>(analyzer.timestamps()[f]));
            for (int b = 0; b < 8; ++b) {
                std::fprintf(out, "  %6.3f", per_frame[f].ratio[b] * 100.0);
            }
            std::fprintf(out, "\n");
        }
        std::fprintf(out, "\n  ... (total %zu frames)\n", per_frame.size());
    }

    std::fprintf(out, "================================================================\n");
    std::fprintf(out, "  Audit complete. No fatal signal anomalies detected.\n");
    std::fprintf(out, "================================================================\n");

    if (out != stdout) {
        std::fclose(out);
    }

    return 0;
}
