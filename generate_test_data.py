import struct
import math
import random

filename = "test_crdm_data.bin"
num_channels = 4
num_samples = 32768
sample_rate_khz = 500

magic = b"CRDMAE01"
version = 1
timestamp_resolution_ns = 1
reserved = b"\x00" * 28

header = struct.pack("<8sIIIQQ28s",
    magic, version, num_channels, sample_rate_khz,
    num_samples, timestamp_resolution_ns, reserved)

with open(filename, "wb") as f:
    f.write(header)

    dt_ns = 1e9 / (sample_rate_khz * 1000.0)
    base_ts = 1718409600000000000
    random.seed(42)

    for s in range(num_samples):
        ts = base_ts + int(s * dt_ns)
        f.write(struct.pack("<Q", ts))

        t = s / (sample_rate_khz * 1000.0)

        for ch in range(num_channels):
            freq_hz = 50000.0 + ch * 75000.0
            v = 0.5 * math.sin(2 * math.pi * freq_hz * t) + random.gauss(0, 0.1)

            if num_samples * 0.6 < s < num_samples * 0.7:
                burst_freq = 200000.0 + ch * 50000.0
                pos = s / num_samples - 0.65
                v += 1.5 * math.sin(2 * math.pi * burst_freq * t) * math.exp(-8.0 * pos * pos)

            raw = int(max(-1.0, min(1.0, v)) * 32767.0)
            f.write(struct.pack("<h", raw))

print(f"Generated test file: {filename} ({num_channels} channels, {num_samples} samples)")
