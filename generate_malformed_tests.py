import struct
import os

def write_header(f, num_channels, num_samples, sample_rate=500):
    magic = b"CRDMAE01"
    version = 1
    ts_res = 1
    reserved = b"\x00" * 28
    header = struct.pack("<8sIIIQQ28s",
        magic, version, num_channels, sample_rate,
        num_samples, ts_res, reserved)
    f.write(header)

def write_frame(f, ts_ns, values, num_channels):
    f.write(struct.pack("<Q", ts_ns))
    for i in range(num_channels):
        v = max(-1.0, min(1.0, values[i]))
        raw = int(v * 32767.0)
        f.write(struct.pack("<h", raw))

os.makedirs("malformed_tests", exist_ok=True)

num_channels = 4
sr_khz = 500
dt_ns = int(1e9 / (sr_khz * 1000))
base_ts = 1718409600000000000
N = 16384

# === Test 1: 文件头部num_samples被恶意篡改为超大值 (实际只有很少数据) ===
print("Test 1: Oversized declared num_samples (should trigger safe_num_samples clamping)")
with open("malformed_tests/test01_oversized_declare.bin", "wb") as f:
    write_header(f, num_channels, 1000000000)
    for s in range(N):
        ts = base_ts + s * dt_ns
        vals = [0.1 * (i + 1) for i in range(num_channels)]
        write_frame(f, ts, vals, num_channels)

# === Test 2: 文件被截断 (中途切断) ===
print("Test 2: Truncated file mid-frame")
import math
with open("malformed_tests/test02_truncated.bin", "wb") as f:
    write_header(f, num_channels, N)
    for s in range(N):
        ts = base_ts + s * dt_ns
        vals = [0.3 * math.sin(2 * math.pi * s / 50 + i) for i in range(num_channels)]
        write_frame(f, ts, vals, num_channels)
    f.seek(0, 2)
    size = f.tell()
    f.truncate(size - 30)

# === Test 3: 中间出现畸形时间戳跳变 ===
print("Test 3: Timestamp anomalies (jumps & time travel)")
import random
random.seed(1)
with open("malformed_tests/test03_bad_timestamps.bin", "wb") as f:
    write_header(f, num_channels, N)
    for s in range(N):
        if s == 3000:
            ts = 0
        elif s == 6000:
            ts = 0xFFFFFFFFFFFFFFFF
        elif s == 9000:
            ts = base_ts - 100000000000
        else:
            ts = base_ts + s * dt_ns
        vals = [0.2 * (i + 1) for i in range(num_channels)]
        write_frame(f, ts, vals, num_channels)

# === Test 4: 电压值超范围 ===
print("Test 4: Voltage values exceed +/-1 (clipping detector)")
with open("malformed_tests/test04_bad_voltage.bin", "wb") as f:
    write_header(f, num_channels, N)
    for s in range(N):
        ts = base_ts + s * dt_ns
        vals = [0.1 * i for i in range(num_channels)]
        if 4000 < s < 5000:
            vals = [5.0 for _ in range(num_channels)]
        write_frame(f, ts, vals, num_channels)

# === Test 5: 完全正确的好文件（用于对比） ===
print("Test 5: Perfect clean file")
with open("malformed_tests/test05_clean.bin", "wb") as f:
    write_header(f, num_channels, N)
    for s in range(N):
        ts = base_ts + s * dt_ns
        vals = [0.2 * math.sin(2 * math.pi * s / 100 + i * 0.7) for i in range(num_channels)]
        write_frame(f, ts, vals, num_channels)

# === Test 6-20: 批量20个文件混合（模拟数月历史审计，其中部分文件畸形） ===
print("Test 6-20: Batch of 15 mixed files")
for idx in range(15):
    fname = f"malformed_tests/batch_{idx:02d}.bin"
    bad_mode = idx % 5
    with open(fname, "wb") as f:
        write_header(f, num_channels, 8192 if bad_mode != 0 else 2000000000)
        for s in range(8192):
            if bad_mode == 2 and s in (1000, 2000, 3000):
                ts = 0
            else:
                ts = base_ts + (idx * 1000000 + s) * dt_ns
            amp = 0.3 + 0.1 * idx
            vals = [amp * math.sin(2 * math.pi * s / (50 + idx * 7) + i) for i in range(num_channels)]
            if bad_mode == 3 and 4000 < s < 4100:
                vals = [-10.0 for _ in range(num_channels)]
            write_frame(f, ts, vals, num_channels)
        if bad_mode == 4:
            f.seek(0, 2)
            sz = f.tell()
            f.truncate(sz - 17)

# === Generate batch file list ===
with open("malformed_tests/batch_list.txt", "w") as lst:
    lst.write("# Batch test file list for CRDM audit\n")
    for i in range(1, 6):
        lst.write(f"malformed_tests/test{i:02d}_*.bin\n".replace("*",
            {1:"oversized_declare", 2:"truncated", 3:"bad_timestamps",
             4:"bad_voltage", 5:"clean"}[i]))
    for idx in range(15):
        lst.write(f"malformed_tests/batch_{idx:02d}.bin\n")

print("All malformed test datasets generated in malformed_tests/")
