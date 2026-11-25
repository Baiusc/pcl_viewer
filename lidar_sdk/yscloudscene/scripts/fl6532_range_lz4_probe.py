#!/usr/bin/env python3
"""FL6532 Range LZ4 UDP 接收探针

绕过上位机 SDK 直接接收板端发送的 FL6532 Range LZ4 数据，用于定位
"上位机收到杂乱短包" 属于网络/板端问题还是上位机接收链路问题：

  - datagram 长度分布（新协议所有包固定 1400B，避免 IP 分片；末包补 0）
  - LZ4 头自洽性校验（sof==0xAA, version==1, data_len==len, 0<payload_len<=len-hdr）
  - 每个 frame_id 的收包数与 seq 缺失区间
  - 异常包的前 32 字节 hex（可直接看出是截断的满包还是别的数据）

用法：
  1. 关闭上位机 YSCloudScene（避免端口冲突）
  2. python fl6532_range_lz4_probe.py [port] [duration_seconds]
     默认 port=50001（上位机 PortAllocator base 50000，单设备数据口为 50001）

仅依赖 Python 标准库。
"""
import socket
import struct
import sys
import time
from collections import Counter

HDR_FMT = "<BBHHBBHHH"  # sof, ver, frame_id, seq, flag, sub, data_len, payload_len, reserved
HDR_LEN = struct.calcsize(HDR_FMT)  # 14
META_LEN = 72
FLAG_START, FLAG_NORMAL, FLAG_END = 0, 1, 2


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 50001
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64 * 1024 * 1024)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(1.0)
    print(f"listening udp 0.0.0.0:{port} for {duration:.0f}s ...")
    print("(若收不到数据：确认已关闭上位机，且板端 center_data_port 指向本机该端口)")

    total = 0
    len_counter = Counter()
    malformed = 0
    malformed_samples = []
    good_samples = []
    frames = {}  # frame_id -> {"seqs": {seq: payload_len}, "start": bool, "end": bool}
    t0 = time.time()

    while time.time() - t0 < duration:
        try:
            data, _addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        total += 1
        n = len(data)
        len_counter[n] += 1

        if n < HDR_LEN:
            malformed += 1
            if len(malformed_samples) < 8:
                malformed_samples.append((n, data[: min(n, 32)].hex(" ")))
            continue

        sof, ver, fid, seq, flag, sub, dlen, plen, _rsv = struct.unpack_from(HDR_FMT, data)
        hdr_len = HDR_LEN + (META_LEN if flag == FLAG_START else 0)
        # 新协议：包长固定 1400，末包不足时尾部补 0，payload_len 为有效长度
        ok = (sof == 0xAA and ver == 1 and dlen == n and 0 < plen <= n - hdr_len)
        if not ok:
            malformed += 1
            if len(malformed_samples) < 8:
                malformed_samples.append(
                    (n, f"fid={fid} seq={seq} flag={flag} sub={sub} dlen={dlen} plen={plen} "
                        f"hex={data[: min(n, 32)].hex(' ')}"))
            continue

        if len(good_samples) < 4:
            good_samples.append((n, fid, seq, flag, sub, dlen, plen))
        fr = frames.setdefault(fid, {"seqs": {}, "start": False, "end": False})
        if flag == FLAG_START:
            fr["start"] = True
        fr["seqs"][seq] = plen
        if flag == FLAG_END:
            fr["end"] = True

    elapsed = max(time.time() - t0, 1e-6)
    print(f"\n=== total={total} datagrams ({total / elapsed:.0f}/s), malformed={malformed} ===")

    print("length distribution (top 15):")
    for length, count in len_counter.most_common(15):
        print(f"  len={length:5d}: {count:6d} ({100.0 * count / total:.2f}%)")

    if good_samples:
        print("\nvalid samples (size, frame_id, seq, flag, sub, data_len, payload_len):")
        for item in good_samples:
            print(f"  size={item[0]} fid={item[1]} seq={item[2]} flag={item[3]} "
                  f"sub={item[4]} data_len={item[5]} payload_len={item[6]}")

    if malformed_samples:
        print("\nmalformed samples:")
        for n, desc in malformed_samples:
            print(f"  size={n}: {desc}")

    print(f"\nframes: seen={len(frames)}")
    for fid in sorted(frames.keys())[-8:]:
        fr = frames[fid]
        seqs = fr["seqs"]
        if not seqs:
            continue
        expected = max(seqs) + 1
        missing = expected - len(seqs)
        gap_desc = ""
        if missing:
            gaps = []
            gap_start = None
            for s in range(expected):
                if s not in seqs:
                    if gap_start is None:
                        gap_start = s
                elif gap_start is not None:
                    gaps.append((gap_start, s - 1))
                    gap_start = None
            if gap_start is not None:
                gaps.append((gap_start, expected - 1))
            gap_desc = ", missing=" + ",".join(
                f"{a}" if a == b else f"{a}-{b}" for a, b in gaps[:8])
            if len(gaps) > 8:
                gap_desc += ",..."
        print(f"  frm={fid} start={fr['start']} end={fr['end']} "
              f"pkts={len(seqs)}/{expected}{gap_desc}")


if __name__ == "__main__":
    main()
